import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    package_name='blaunch_pkg'

    controller_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'tebo_control.launch.py'
        )])
    )

    rplidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rplidar.launch.py'
        )])
    )

    delayed_rplidar = TimerAction(period=5.0, actions=[rplidar_node])

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'nav2.launch.py'
        )])
    )

    return LaunchDescription([
        controller_manager_launch,
        delayed_rplidar,
        navigation
    ])
