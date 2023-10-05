import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    package_name='blaunch_pkg'

    controller_params_file= "/home/tubo/tebo1_brain_ws/src/descriptions_pkg/config/controllers.yaml"

    robot_description_path = os.path.join(
        get_package_share_directory('descriptions_pkg'),
        'descriptions',
        'tebo_main.urdf.xacro')
    robot_description_config = xacro.process_file(robot_description_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'state_publisher.launch.py'
        )])
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_params_file],
        output={
          'stdout': 'screen',
          'stderr': 'screen',
          },
        )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    


    return LaunchDescription([
        state_publisher_node,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner
    ])