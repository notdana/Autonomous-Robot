from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/lidar',
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Standard',
                'serial_baudrate': 115200
            }]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output="screen",
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser_frame']
        )

        
    ])
