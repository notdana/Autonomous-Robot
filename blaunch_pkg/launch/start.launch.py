from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    package_name='blaunch_pkg'

    obstacles_node = Node(
        package='tebo_main_nodes',
        executable='tebo_aura_x',
        output="screen"
    )

    remapper_node = Node(
        package='tebo_main_nodes',
        executable='nav2_remapper_x',
        output="screen"
    )

    move_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'tebo_move.launch.py'
        )])
    )

    return LaunchDescription([
        obstacles_node,
        remapper_node,
        move_launch
    ])
