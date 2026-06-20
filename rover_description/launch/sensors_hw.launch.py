"""Hardware sensor bringup — GPS + IMU + MTF-01 optical flow.

GPS  : /dev/rover_gps  @ 38400  → /gps/fix, /gps/vel
IMU  : I2C bus 1, addr 0x69    → /imu/data  (raw)
                                 → /imu/filtered (after notch filter)
Flow : /dev/rover_flow @ 115200 → /optical_flow/odom, /range/height

Ports are fixed via udev rule /etc/udev/rules.d/99-rover-sensors.rules:
    /dev/rover_gps  → GPS serial adapter
    /dev/rover_flow → MTF-01 optical flow

Launch arguments
    gps_port    (default /dev/rover_gps)
    flow_port   (default /dev/rover_flow)
    imu_bus     (default 1)
    imu_addr    (default 105 = 0x69)
    imu_rate_hz (default 100.0)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_description')
    imu_cal   = os.path.join(pkg_share, 'config', 'imu_calibration.yaml')

    # ---- Launch arguments ----
    gps_port_arg = DeclareLaunchArgument(
        'gps_port', default_value='/dev/rover_gps',
        description='Serial port for GPS NMEA stream')

    flow_port_arg = DeclareLaunchArgument(
        'flow_port', default_value='/dev/rover_flow',
        description='Serial port for MTF-01 optical flow (MSP v2)')

    imu_bus_arg = DeclareLaunchArgument(
        'imu_bus', default_value='1',
        description='I2C bus number for ICM-20948')

    imu_addr_arg = DeclareLaunchArgument(
        'imu_addr', default_value='105',
        description='I2C address for ICM-20948 (decimal; 0x69 = 105)')

    imu_rate_arg = DeclareLaunchArgument(
        'imu_rate_hz', default_value='100.0',
        description='IMU polling rate in Hz')

    # ---- GPS driver ----
    gps_driver = Node(
        package='rover_description',
        executable='gps_driver_node.py',
        name='gps_driver_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('gps_port'),
            'baud': 38400,
            'frame_id': 'GPS',
            'use_sim_time': False,
        }]
    )

    # ---- ICM-20948 IMU driver ----
    imu_driver = Node(
        package='rover_description',
        executable='icm20948_driver_node.py',
        name='icm20948_driver_node',
        output='screen',
        parameters=[{
            'i2c_bus':  LaunchConfiguration('imu_bus'),
            'i2c_addr': LaunchConfiguration('imu_addr'),
            'rate_hz':  LaunchConfiguration('imu_rate_hz'),
            'frame_id': 'imu_link',
            'use_sim_time': False,
        }]
    )

    # ---- IMU notch filter (raw → filtered) ----
    imu_filter = Node(
        package='rover_description',
        executable='imu_filter_node.py',
        name='imu_filter_node',
        output='screen',
        parameters=[
            imu_cal,
            {'use_sim_time': False}
        ]
    )

    # ---- MTF-01 optical flow + rangefinder driver ----
    flow_driver = Node(
        package='rover_description',
        executable='mtf01_driver_node.py',
        name='mtf01_driver_node',
        output='screen',
        parameters=[{
            'port':            LaunchConfiguration('flow_port'),
            'baud':            115200,
            'frame_id':        'optical_flow_link',
            'min_quality':     50,
            'base_covariance': 0.01,
            'use_sim_time':    False,
            'swap_xy':         True,   # sensor heading = rover RIGHT → swap axes
            'x_sign':          1,
            'y_sign':          -1,     # sensor X = rover -Y → flip lateral
        }]
    )

    # ---- Flow derotation (IMU gyro removes camera rotation bias) ----
    flow_derotation = Node(
        package='rover_description',
        executable='flow_derotation_node.py',
        name='flow_derotation_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        gps_port_arg,
        flow_port_arg,
        imu_bus_arg,
        imu_addr_arg,
        imu_rate_arg,
        gps_driver,
        imu_driver,
        imu_filter,
        flow_driver,
        flow_derotation,
    ])
