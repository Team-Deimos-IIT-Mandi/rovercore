#!/usr/bin/env python3
"""GPS driver — NMEA serial → NavSatFix.

Parses $GNGGA (and $GNRMC for velocity) from serial port.

Publishes:
    /gps/fix  (sensor_msgs/NavSatFix)
    /gps/vel  (geometry_msgs/TwistStamped)  — from $GNRMC ground speed
"""

import math
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistStamped


def _nmea_checksum_ok(sentence: str) -> bool:
    if '*' not in sentence:
        return False
    data, cs = sentence.rsplit('*', 1)
    data = data.lstrip('$')
    computed = 0
    for ch in data:
        computed ^= ord(ch)
    return computed == int(cs.strip(), 16)


def _parse_lat(val: str, hemi: str) -> float:
    # ddmm.mmmm
    if not val:
        return float('nan')
    d = float(val[:2])
    m = float(val[2:])
    deg = d + m / 60.0
    return -deg if hemi == 'S' else deg


def _parse_lon(val: str, hemi: str) -> float:
    # dddmm.mmmm
    if not val:
        return float('nan')
    d = float(val[:3])
    m = float(val[3:])
    deg = d + m / 60.0
    return -deg if hemi == 'W' else deg


class GpsDriverNode(Node):
    def __init__(self):
        super().__init__('gps_driver_node')

        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baud', 38400)
        self.declare_parameter('frame_id', 'GPS')

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.frame_id = self.get_parameter('frame_id').value

        self.pub_fix = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.pub_vel = self.create_publisher(TwistStamped, '/gps/vel', 10)

        self._port = port
        self._baud = baud
        self.ser = None
        self._try_open()

        self._rmc_speed_ms = 0.0
        self._rmc_course_deg = 0.0

        self.create_timer(0.0, self._spin_once)

    def _try_open(self):
        try:
            self.ser = serial.Serial(self._port, self._baud, timeout=1.0)
            self.get_logger().info(f'GPS opened {self._port} @ {self._baud}')
        except serial.SerialException as e:
            self.get_logger().error(f'GPS open failed: {e} — will retry')
            self.ser = None

    def _spin_once(self):
        """Called repeatedly; reads one line per timer tick."""
        if self.ser is None:
            self._try_open()
            return
        try:
            raw = self.ser.readline()
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial read error: {e}')
            self.ser = None
            return

        try:
            line = raw.decode('ascii', errors='ignore').strip()
        except Exception:
            return

        if not line or not line.startswith('$'):
            return

        if not _nmea_checksum_ok(line):
            return

        tag = line.split(',')[0]
        if tag in ('$GNGGA', '$GPGGA'):
            self._handle_gga(line)
        elif tag in ('$GNRMC', '$GPRMC'):
            self._handle_rmc(line)

    def _handle_gga(self, line: str):
        parts = line.split(',')
        if len(parts) < 10:
            return

        fix_quality = int(parts[6]) if parts[6] else 0

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.status.service = NavSatStatus.SERVICE_GPS

        if fix_quality == 0 or not parts[2]:
            msg.status.status = NavSatStatus.STATUS_NO_FIX
            msg.latitude = float('nan')
            msg.longitude = float('nan')
            msg.altitude = float('nan')
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        else:
            msg.status.status = (NavSatStatus.STATUS_GBAS_FIX
                                  if fix_quality == 2 else NavSatStatus.STATUS_FIX)
            msg.latitude = _parse_lat(parts[2], parts[3])
            msg.longitude = _parse_lon(parts[4], parts[5])
            msg.altitude = float(parts[9]) if parts[9] else 0.0

            hdop = float(parts[8]) if parts[8] else 99.99
            # Approximate position covariance from HDOP: sigma ≈ hdop * 3m (CEP)
            cov = (hdop * 3.0) ** 2
            msg.position_covariance = [
                cov,  0.0,  0.0,
                0.0,  cov,  0.0,
                0.0,  0.0,  cov * 4.0,  # vertical typically 2x worse
            ]
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        self.pub_fix.publish(msg)

        # Publish velocity from last RMC
        vel_msg = TwistStamped()
        vel_msg.header.stamp = msg.header.stamp
        vel_msg.header.frame_id = self.frame_id
        course_rad = math.radians(self._rmc_course_deg)
        vel_msg.twist.linear.x = self._rmc_speed_ms * math.cos(course_rad)
        vel_msg.twist.linear.y = self._rmc_speed_ms * math.sin(course_rad)
        self.pub_vel.publish(vel_msg)

    def _handle_rmc(self, line: str):
        parts = line.split(',')
        if len(parts) < 8:
            return
        # Speed over ground in knots → m/s
        self._rmc_speed_ms = float(parts[7]) * 0.514444 if parts[7] else 0.0
        self._rmc_course_deg = float(parts[8]) if len(parts) > 8 and parts[8] else 0.0

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GpsDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
