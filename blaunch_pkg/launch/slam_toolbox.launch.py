import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    params_file = "/home/tubo/tebo1_brain_ws/src/descriptions_pkg/config/slam_toolbox.yaml"

    online_async_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("slam_toolbox"), 'launch', 'online_async_launch.py'
        )]),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': "false"

        }.items()
    )

    return LaunchDescription([
        online_async_launch
    ])
