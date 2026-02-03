import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('striker_bot_description')
    ekf_config = os.path.join(pkg_path, 'config', 'ekf.yaml')

    return LaunchDescription([
        # EKF Node: Fuses Odom and IMU to create the odom -> base_link transform
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': True}]
        ),

        # Navsat Transform Node: Fuses GPS to create the map -> odom transform
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': True}],
            remappings=[('gps/fix', 'gps/fix'),
                        ('imu', 'camera/imu')] # Ensure this matches your IMU topic
        )
    ])