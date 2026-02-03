import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'striker_bot_description'
    
    # 1. Process URDF
    pkg_path = get_package_share_directory(pkg_name)
    xacro_file = os.path.join(pkg_path, 'urdf', 'striker_bot.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # 2. Create a custom world file with GPS coordinates
    world_file = os.path.join(pkg_path, 'worlds', 'gps_world.world')
    
    # If world file doesn't exist, create it
    if not os.path.exists(world_file):
        os.makedirs(os.path.dirname(world_file), exist_ok=True)
        with open(world_file, 'w') as f:
            f.write('''<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="gps_world">
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>37.7749</latitude_deg>
      <longitude_deg>-122.4194</longitude_deg>
      <elevation>0.0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>''')

    # 3. Include Gazebo Launch with custom world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file}.items()
    )

    # 4. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    # 5. Spawn the Robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'striker_bot', '-z', '0.1'],
        output='screen'
    )

    # 6. Joint State Publisher
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        node_joint_state_publisher,
        spawn_entity
    ])


# import os
# from launch import LaunchDescription
# from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
# from launch.event_handlers import OnProcessExit
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     # Get the package directory
#     pkg_path = get_package_share_directory('striker_bot_description')
    
#     # World file with GPS coordinates
#     world_file = os.path.join(pkg_path, 'worlds', 'gps_world.world')
    
#     # If the custom world doesn't exist, use empty world but set GPS via environment
#     if not os.path.exists(world_file):
#         world_file = 'empty.world'
    
#     # Start Gazebo with the world
#     gazebo = ExecuteProcess(
#         cmd=['gzserver', '--verbose', world_file,
#              '-s', 'libgazebo_ros_init.so',
#              '-s', 'libgazebo_ros_factory.so'],
#         output='screen'
#     )

#     # Gazebo client
#     gazebo_client = ExecuteProcess(
#         cmd=['gzclient'],
#         output='screen'
#     )

#     # Get URDF via xacro
#     robot_description_content = Command(
#         [
#             PathJoinSubstitution([FindExecutable(name="xacro")]),
#             " ",
#             PathJoinSubstitution(
#                 [FindPackageShare("striker_bot_description"), "urdf", "striker_bot.urdf.xacro"]
#             ),
#         ]
#     )
#     robot_description = {"robot_description": robot_description_content}

#     # Robot State Publisher
#     robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[robot_description]
#     )

#     # Joint State Publisher
#     joint_state_publisher = Node(
#         package='joint_state_publisher',
#         executable='joint_state_publisher',
#         name='joint_state_publisher',
#         output='screen',
#         parameters=[robot_description]
#     )

#     # Spawn Entity
#     spawn_entity = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=['-entity', 'striker_bot', '-topic', 'robot_description',
#                    '-x', '0.0', '-y', '0.0', '-z', '0.15'],  # Slightly above ground
#         output='screen'
#     )

#     # Set GPS coordinates via environment variable (alternative method)
#     # This sets the initial GPS coordinates for Gazebo
#     os.environ['GAZEBO_MODEL_PATH'] = '/usr/share/gazebo-11/models:' + os.environ.get('GAZEBO_MODEL_PATH', '')
    
#     # You can also try setting spherical coordinates via a service call
#     # We'll create a node to do this after Gazebo starts
    
#     # Node to set GPS coordinates after Gazebo starts
#     set_gps_node = Node(
#         package='striker_bot_description',
#         executable='set_gps_coordinates',
#         name='set_gps_coordinates',
#         output='screen',
#         parameters=[{'latitude': 37.7749, 'longitude': -122.4194, 'altitude': 0.0}]
#     )

#     return LaunchDescription([
#         # Set environment variable for GPS
#         # ExecuteProcess(
#         #     cmd=['bash', '-c', 'export GAZEBO_SPHERICAL_COORDINATES_LATITUDE=37.7749 && export GAZEBO_SPHERICAL_COORDINATES_LONGITUDE=-122.4194'],
#         #     output='screen'
#         # ),
        
#         gazebo,
#         gazebo_client,
#         robot_state_publisher,
#         joint_state_publisher,
#         spawn_entity,
#         # set_gps_node,  # Uncomment if you create this node
#     ])