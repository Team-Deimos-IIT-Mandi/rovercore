#!/usr/bin/env python3
"""MTF-01 optical flow + rangefinder driver — MSP v2 protocol → ROS 2.

Two packet types confirmed by inspection:

  0x1F01  Rangefinder  5 bytes:
      uint8   quality        (0-255)
      int32   distance_mm    (little-endian; 0 = invalid)

  0x1F02  Optical Flow  9 bytes:
      uint8   quality        (0-255)
      int32   motion_x       (little-endian; milliradians of angular displacement)
      int32   motion_y       (little-endian; milliradians of angular displacement)

  Velocity: v_m_s = (motion_mrad / 1000.0) * distance_m * update_rate_hz
  Scale is approximate; tune `motion_scale` parameter after movement tests.

MSP v2 frame:
    '$' 'X' direction flag(1) func(uint16LE) size(uint16LE) payload(size) crc(CRC8_DVB_S2)
    CRC covers: flag + func_lo + func_hi + size_lo + size_hi + payload bytes

Publishes:
    /optical_flow/odom  (nav_msgs/Odometry)  — ground-plane velocity, twist only
    /range/height       (sensor_msgs/Range)  — downward distance in metres
"""

import struct
import serial
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

MSP_HEADER   = b'$X'
FUNC_RANGE   = 0x1F01
FUNC_FLOW    = 0x1F02

FMT_RANGE = '<Bi'   # uint8 quality, int32 dist_mm  (5 bytes)
FMT_FLOW  = '<Bii'  # uint8 quality, int32 mx, int32 my  (9 bytes)


def _crc8_dvb_s2(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


class Mtf01DriverNode(Node):
    def __init__(self):
        super().__init__('mtf01_driver_node')

        self.declare_parameter('port',            '/dev/ttyUSB0')
        self.declare_parameter('baud',            115200)
        self.declare_parameter('frame_id',        'optical_flow_link')
        self.declare_parameter('min_flow_quality', 30)
        self.declare_parameter('min_range_quality', 10)
        self.declare_parameter('base_covariance', 0.01)
        # Tune this after movement tests: v = (motion_mrad/1000) * dist_m * rate_hz
        # If motion unit is NOT milliradians, adjust accordingly.
        self.declare_parameter('motion_scale',    1.0)
        # Axis remapping — tune to match physical sensor mounting orientation:
        #   swap_xy: true if sensor X points along rover Y (90° rotation)
        #   x_sign:  -1 to flip rover-forward direction
        #   y_sign:  -1 to flip rover-lateral direction
        self.declare_parameter('swap_xy',  False)
        self.declare_parameter('x_sign',   1)
        self.declare_parameter('y_sign',   1)

        port               = self.get_parameter('port').value
        baud               = self.get_parameter('baud').value
        self.frame_id      = self.get_parameter('frame_id').value
        self.min_flow_q    = self.get_parameter('min_flow_quality').value
        self.min_range_q   = self.get_parameter('min_range_quality').value
        self.base_cov      = self.get_parameter('base_covariance').value
        self.motion_scale  = self.get_parameter('motion_scale').value
        self.swap_xy       = self.get_parameter('swap_xy').value
        self.x_sign        = self.get_parameter('x_sign').value
        self.y_sign        = self.get_parameter('y_sign').value

        self.pub_odom  = self.create_publisher(Odometry, '/optical_flow/odom', 10)
        self.pub_range = self.create_publisher(Range,    '/range/height',       10)

        self._port = port
        self._baud = baud
        self.ser = None
        self._try_open()

        self._buf       = bytearray()
        self._dist_m    = 1.0    # last valid height
        self._last_flow = self.get_clock().now()

        self.create_timer(0.0, self._spin_once)

    def _try_open(self):
        try:
            self.ser = serial.Serial(self._port, self._baud, timeout=0.05)
            self.get_logger().info(f'MTF-01 opened {self._port} @ {self._baud} (MSP v2)')
        except serial.SerialException as e:
            self.get_logger().error(f'MTF-01 open failed: {e} — will retry')
            self.ser = None

    # ------------------------------------------------------------------
    def _spin_once(self):
        if self.ser is None:
            self._try_open()
            return
        try:
            chunk = self.ser.read(256)
        except serial.SerialException as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)
            self.ser = None
            return
        if chunk:
            self._buf.extend(chunk)
            self._parse()

    def _parse(self):
        while True:
            idx = self._buf.find(MSP_HEADER)
            if idx < 0:
                self._buf.clear()
                return
            if idx > 0:
                del self._buf[:idx]

            # Need 8 header bytes before we know payload length
            if len(self._buf) < 8:
                return

            direction = self._buf[2]
            if direction not in (0x3C, 0x3E):   # '<' or '>'
                del self._buf[0]
                continue

            flag     = self._buf[3]
            func_id  = struct.unpack_from('<H', self._buf, 4)[0]
            pay_len  = struct.unpack_from('<H', self._buf, 6)[0]
            frame_len = 8 + pay_len + 1

            if len(self._buf) < frame_len:
                return

            payload  = bytes(self._buf[8:8 + pay_len])
            crc_recv = self._buf[8 + pay_len]

            crc_data = bytes([flag,
                              func_id & 0xFF, (func_id >> 8) & 0xFF,
                              pay_len & 0xFF, (pay_len >> 8) & 0xFF]) + payload
            if _crc8_dvb_s2(crc_data) != crc_recv:
                del self._buf[0]
                continue

            del self._buf[:frame_len]

            if func_id == FUNC_RANGE and pay_len == struct.calcsize(FMT_RANGE):
                self._handle_range(payload)
            elif func_id == FUNC_FLOW and pay_len == struct.calcsize(FMT_FLOW):
                self._handle_flow(payload)

    # ------------------------------------------------------------------
    def _handle_range(self, payload: bytes):
        quality, dist_mm = struct.unpack(FMT_RANGE, payload)

        rng = Range()
        rng.header.stamp    = self.get_clock().now().to_msg()
        rng.header.frame_id = self.frame_id
        rng.radiation_type  = Range.INFRARED
        rng.field_of_view   = 0.052
        rng.min_range       = 0.01
        rng.max_range       = 8.0

        if quality >= self.min_range_q and dist_mm > 0:
            rng.range      = dist_mm / 1000.0
            self._dist_m   = rng.range
        else:
            rng.range = float('inf')

        self.pub_range.publish(rng)

    def _handle_flow(self, payload: bytes):
        quality, motion_x, motion_y = struct.unpack(FMT_FLOW, payload)

        now = self.get_clock().now()
        dt  = (now - self._last_flow).nanoseconds * 1e-9
        self._last_flow = now

        if dt <= 0.0 or dt > 1.0:
            return

        # Angular displacement (mrad) → linear velocity (m/s)
        # v = (motion_mrad / 1000) * height_m / dt
        # motion_scale lets you correct the unit if it's not milliradians
        raw_x = (motion_x / 1000.0) * self._dist_m * self.motion_scale / dt
        raw_y = (motion_y / 1000.0) * self._dist_m * self.motion_scale / dt

        if self.swap_xy:
            raw_x, raw_y = raw_y, raw_x
        vx = self.x_sign * raw_x
        vy = self.y_sign * raw_y

        if quality >= self.min_flow_q:
            cov = self.base_cov * (255.0 / max(quality, 1)) ** 2
        else:
            cov = 1e6

        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.linear.y = float(vy)
        odom.twist.covariance[0]  = cov
        odom.twist.covariance[7]  = cov
        odom.twist.covariance[14] = 1e6
        odom.twist.covariance[21] = 1e6
        odom.twist.covariance[28] = 1e6
        odom.twist.covariance[35] = 1e6
        self.pub_odom.publish(odom)

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Mtf01DriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
