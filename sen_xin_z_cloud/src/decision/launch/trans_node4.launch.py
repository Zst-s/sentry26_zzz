from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=PathJoinSubstitution([
                FindPackageShare('decision'), 'config', 'trans_node4.yaml']),
            description='YAML file containing serial_chassis_node.ros__parameters.sender_id'),
        Node(
            package='decision', executable='trans_node4', name='serial_chassis_node',
            output='screen', respawn=False,
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
