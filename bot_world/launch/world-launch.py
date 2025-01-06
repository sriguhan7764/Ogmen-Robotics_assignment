from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the world file path
    world_file_path = os.path.join(get_package_share_directory('bot_world'), 
                                  'worlds', 
                                  'custom_world.world')
    
    # Get the URDF file path
    pkg_bot_description = get_package_share_directory('bot_description')
    xacro_file = os.path.join(pkg_bot_description, 'urdf', 'bot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]),
                                     value_type=str)

    # Launch Gazebo world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': world_file_path}.items()
    )

    # Spawn the robot
    spawn_entity = Node(package='gazebo_ros', 
                       executable='spawn_entity.py',
                       arguments=['-topic', 'robot_description',
                                '-entity', 'custom_bot',
                                '-x', '0',
                                '-y', '0',
                                '-z', '0.1'],
                       output='screen')

    # Launch robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
