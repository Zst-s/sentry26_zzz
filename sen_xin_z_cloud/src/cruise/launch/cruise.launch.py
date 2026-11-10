import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default = os.path.join(get_package_share_directory('cruise'), 'config', 'cruise.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default),
        Node(package='cruise', executable='cruise_node', name='cruise_controller_node',
             parameters=[LaunchConfiguration('params_file')], output='screen'),
    ])
