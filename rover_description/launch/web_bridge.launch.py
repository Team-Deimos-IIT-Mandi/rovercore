from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch rosbridge_server for web GUI communication
    This enables WebSocket connection for real-time process tracking
    """
    return LaunchDescription([
        # ROS Bridge WebSocket Server
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{
                'port': 9090,
                'address': '0.0.0.0',
                'retry_startup_delay': 5.0,
                'fragment_timeout': 600,
                'delay_between_messages': 0.0,
                'max_message_size': 10000000,
                'unregister_timeout': 10.0
            }]
        ),
        
        # Web Video Server for camera streams
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen',
            parameters=[{
                'port': 8080,
                'address': '0.0.0.0',
                'server_threads': 4,
                'ros_threads': 4,
                'default_stream_type': 'mjpeg'
            }]
        ),
        
        # CMD Vel Relay
        Node(
            package='rover_description',
            executable='cmd_vel_relay.py',
            name='cmd_vel_relay',
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
    ])