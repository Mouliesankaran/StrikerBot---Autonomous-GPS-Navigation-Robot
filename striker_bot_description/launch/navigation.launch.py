import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': os.path.join(get_package_share_directory('striker_bot_description'), 'config', 'nav2_params.yaml')
            }.items()
        )
    ])