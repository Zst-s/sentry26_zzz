from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    defaults = {
        'world': 'footprint', 'prior_map': '',
        'map': [PathJoinSubstitution([FindPackageShare('rm_nav_bringup'), 'map', LaunchConfiguration('world')]), '.yaml'],
        'serial_port': '/dev/ttyACM0', 'enable_serial': 'true',
        'enable_fake_velocity': 'false',
        'nav_rviz': 'true', 'driver': 'true',
        'cruise': 'false',
    }
    args = [DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()]
    nav = IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare('rm_nav_bringup'), 'launch', 'bringup_real.launch.py'])),
        launch_arguments=({
            key: LaunchConfiguration(key) for key in
            ('world', 'prior_map', 'map', 'enable_fake_velocity', 'nav_rviz', 'driver', 'cruise')
        } | {'mode': 'nav', 'lio': 'zlio', 'localization': 'zlio'}).items())
    serial = Node(package='decision', executable='trans_node4', output='screen',
        prefix=['gnome-terminal --wait --title=trans_node4 -- bash -c \'exec "$@" > /dev/tty 2>&1\' bash'],
        condition=IfCondition(LaunchConfiguration('enable_serial')),
        parameters=[PathJoinSubstitution([FindPackageShare('decision'), 'config', 'trans_node4.yaml']),
            {'serial_port': LaunchConfiguration('serial_port')}])
    referee_map = Node(package='decision', executable='referee_map_tf', output='screen')
    return LaunchDescription(args + [referee_map, nav, serial])
