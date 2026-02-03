import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('striker_bot_description')
    
    return LaunchDescription([
        # Launch Gazebo simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_path, 'launch', 'gazebo.launch.py')
            ]),
        ),
        
        # Launch Localization (EKF + GPS)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_path, 'launch', 'localization.launch.py')
            ]),
        ),
        
        # Launch Navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_path, 'launch', 'navigation.launch.py')
            ]),
        ),
        
        # Fake GPS Publisher (for testing in Gazebo)
        Node(
            package='striker_bot_description',
            executable='fake_gps_publisher',
            name='fake_gps_publisher',
            output='screen',
        ),
    ])