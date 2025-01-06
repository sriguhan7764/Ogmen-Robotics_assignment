from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch RViz with custom configuration
    rviz_config = os.path.join(get_package_share_directory('bot_control'),
                              'rviz',
                              'laser_filter.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # Launch the laser filter node
    laser_filter_node = Node(
        package='bot_control',
        executable='reading_laser',
        name='laser_filter',
        output='screen'
    )

    return LaunchDescription([
        laser_filter_node,
        rviz_node
    ])
