import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    package_name='blaunch_pkg'
    explore_package = "explore_lite"


    controller_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'tebo_move.launch.py'
        )])
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'slam_toolbox.launch.py'
        )])
    )

    delayed_slam = TimerAction(period=15.0, actions=[slam_launch])

    explore_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(explore_package), 'launch', 'explore.launch.py'
        )])
    )


    return LaunchDescription([
        controller_manager_launch,
        delayed_slam,
        explore_launch
    ])
