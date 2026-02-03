import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. Get the path to the package and the xacro file
    pkg_path = get_package_share_directory('striker_bot_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'striker_bot.urdf.xacro')

    # 2. Process the xacro file (equivalent to running "xacro file.xacro" in terminal)
    robot_description_config = Command(['xacro ', xacro_file])
    
    # Create a parameter dictionary for the node
    params = {'robot_description': robot_description_config}

    # 3. Define the nodes
    
    # Robot State Publisher: Publishes the TF tree (transforms)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Joint State Publisher GUI: Allows you to move wheels manually in RViz
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # RViz2: Visualization tool
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # arguments=['-d', os.path.join(pkg_path, 'rviz', 'striker_bot.rviz')] # Uncomment if you have a saved config
    )

    # 4. Return the Launch Description
    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz
    ])