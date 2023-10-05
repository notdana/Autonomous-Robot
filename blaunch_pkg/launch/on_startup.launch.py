from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    package_name = "blaunch_pkg"

    mqtt_node = Node(
        package='tebo_main_nodes',
        executable='mqtt_x',
        output="screen"
    )

    process_handler_node = Node(
        package='tebo_main_nodes',
        executable='process_handler_x',
        output="screen"
    )

    battery_node = Node(
        package='tebo_main_nodes',
        executable='battery_x',
        output="screen"
    )

    led_node = Node(
        package='tebo_main_nodes',
        executable='leds_x',
        output="screen"
    )

    tilting_node = Node(
        package='tebo_main_nodes',
        executable='tilt_x',
        output="screen"
    )

    rplidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rplidar.launch.py'
        )])
    )


    return LaunchDescription([
        rplidar_node,
        mqtt_node,
        process_handler_node,
        battery_node,
        tilting_node,
        led_node,
    ])
