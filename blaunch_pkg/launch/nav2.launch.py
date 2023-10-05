from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    map_file="/home/tubo/saved_map.yaml"
    param_file="/home/tubo/tebo1_brain_ws/src/descriptions_pkg/config/nav2_params.yaml"


    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'), '/launch', '/bringup_launch.py']),
        launch_arguments={
            'map': map_file,
            'params_file': param_file
        }.items(),
    )

    remapper_node = Node(
        package='tebo_main_nodes',
        executable='nav2_remapper_x',
        output="screen"
    )

    delayed_navigation = TimerAction(period=10.0, actions=[navigation])

    send_pose = Node(
        package='tebo_main_nodes',
        executable='return_home_x',
        output="screen"
    )

    delayed_pose = TimerAction(period=15.0, actions=[send_pose])


    return LaunchDescription([
        delayed_navigation,
        remapper_node,
        delayed_pose
    ])
