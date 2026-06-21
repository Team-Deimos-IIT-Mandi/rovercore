#!/usr/bin/env python3
"""ICM-20948 hardware driver — I2C → sensor_msgs/Imu.

Reads accel + gyro from ICM-20948 at I2C address 0x69 on bus 1.
Configures ±16g / ±2000 dps ranges (hardware defaults).

Publishes:
    /imu/data  (sensor_msgs/Imu)  — raw, unfiltered

Pass through imu_filter_node for notch filtering before EKF.
"""

import struct
import time
import smbus2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# ---- Register map (Bank 0) ----
REG_BANK_SEL    = 0x7F
REG_WHO_AM_I    = 0x00
REG_PWR_MGMT_1  = 0x06
REG_PWR_MGMT_2  = 0x07
REG_ACCEL_XOUT_H = 0x2D  # 6 bytes: AX_H, AX_L, AY_H, AY_L, AZ_H, AZ_L
REG_GYRO_XOUT_H  = 0x33  # 6 bytes: GX_H, GX_L, GY_H, GY_L, GZ_H, GZ_L
REG_TEMP_OUT_H   = 0x39

# ---- Bank 2 config registers ----
REG_ACCEL_CONFIG  = 0x14
REG_GYRO_CONFIG_1 = 0x01

WHO_AM_I_VAL = 0xEA

# ---- Scale factors ----
# Accel: ±16g default → ACCEL_FS_SEL=3 → 2048 LSB/g
ACCEL_SCALE = 9.80665 / 2048.0   # m/s² per LSB
# Gyro:  ±2000 dps default → GYRO_FS_SEL=3 → 16.4 LSB/dps
GYRO_SCALE  = (3.14159265 / 180.0) / 16.4  # rad/s per LSB

# Covariance placeholders — tune from static calibration
ACCEL_COV = 0.01 ** 2   # (m/s²)²
GYRO_COV  = 0.005 ** 2  # (rad/s)²


def _sel_bank(bus, addr, bank: int):
    bus.write_byte_data(addr, REG_BANK_SEL, (bank & 0x03) << 4)


def _read_signed16(high: int, low: int) -> int:
    val = (high << 8) | low
    return val - 65536 if val >= 32768 else val


class Icm20948DriverNode(Node):
    def __init__(self):
        super().__init__('icm20948_driver_node')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_addr', 0x69)
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('frame_id', 'imu_link')

        bus_num  = self.get_parameter('i2c_bus').value
        self.addr = self.get_parameter('i2c_addr').value
        rate_hz  = self.get_parameter('rate_hz').value
        self.frame_id = self.get_parameter('frame_id').value

        self._bus_num = bus_num
        self.bus = None
        self._try_open()

        self.pub = self.create_publisher(Imu, '/imu/data', 50)
        self.create_timer(1.0 / rate_hz, self._read_and_publish)

    def _try_open(self):
        try:
            self.bus = smbus2.SMBus(self._bus_num)
            self._setup()
            self.get_logger().info(
                f'ICM-20948 ready on bus {self._bus_num} addr {hex(self.addr)} @ ready')
        except Exception as e:
            self.get_logger().error(f'I2C open failed: {e} — will retry')
            self.bus = None

    def _setup(self):
        _sel_bank(self.bus, self.addr, 0)
        who = self.bus.read_byte_data(self.addr, REG_WHO_AM_I)
        if who != WHO_AM_I_VAL:
            self.get_logger().error(f'WHO_AM_I = {hex(who)}, expected {hex(WHO_AM_I_VAL)}')

        # Wake up, auto-select best clock
        self.bus.write_byte_data(self.addr, REG_PWR_MGMT_1, 0x01)
        time.sleep(0.1)
        # Enable accel + gyro
        self.bus.write_byte_data(self.addr, REG_PWR_MGMT_2, 0x00)

        # Bank 2: accel ±16g (ACCEL_FS_SEL=3, bits [2:1]) → 0x06
        _sel_bank(self.bus, self.addr, 2)
        self.bus.write_byte_data(self.addr, REG_ACCEL_CONFIG, 0x06)
        # Bank 2: gyro ±2000 dps (GYRO_FS_SEL=3, bits [2:1]) → 0x06
        self.bus.write_byte_data(self.addr, REG_GYRO_CONFIG_1, 0x06)

        _sel_bank(self.bus, self.addr, 0)

    def _read_and_publish(self):
        if self.bus is None:
            self._try_open()
            return
        try:
            _sel_bank(self.bus, self.addr, 0)
            raw_a = self.bus.read_i2c_block_data(self.addr, REG_ACCEL_XOUT_H, 6)
            raw_g = self.bus.read_i2c_block_data(self.addr, REG_GYRO_XOUT_H, 6)
        except Exception as e:
            self.get_logger().warn(f'I2C read error: {e}', throttle_duration_sec=5.0)
            self.bus = None
            return

        ax = _read_signed16(raw_a[0], raw_a[1]) * ACCEL_SCALE
        ay = _read_signed16(raw_a[2], raw_a[3]) * ACCEL_SCALE
        az = _read_signed16(raw_a[4], raw_a[5]) * ACCEL_SCALE

        gx = _read_signed16(raw_g[0], raw_g[1]) * GYRO_SCALE
        gy = _read_signed16(raw_g[2], raw_g[3]) * GYRO_SCALE
        gz = _read_signed16(raw_g[4], raw_g[5]) * GYRO_SCALE

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Orientation unknown — signal with -1 in covariance[0]
        msg.orientation_covariance[0] = -1.0

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.angular_velocity_covariance = [
            GYRO_COV, 0.0, 0.0,
            0.0, GYRO_COV, 0.0,
            0.0, 0.0, GYRO_COV,
        ]

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.linear_acceleration_covariance = [
            ACCEL_COV, 0.0, 0.0,
            0.0, ACCEL_COV, 0.0,
            0.0, 0.0, ACCEL_COV,
        ]

        self.pub.publish(msg)

    def destroy_node(self):
        if hasattr(self, 'bus'):
            self.bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Icm20948DriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
