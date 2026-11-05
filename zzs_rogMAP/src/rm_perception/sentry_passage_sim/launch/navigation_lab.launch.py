"""Isolated truth-pose navigation; never launches hardware, LIO or serial."""
import os
import socket
import tempfile
from pathlib import Path
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    if os.environ.get('ROS_DOMAIN_ID') != '229' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Use run_passage_navigation_sim.sh (isolated domain 229)')
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1', 11359)) == 0:
            raise RuntimeError('Gazebo port 11359 in use; stop the previous simulation first')
    share = Path(get_package_share_directory('sentry_passage_sim'))
    sys.path.insert(0, str(share/'scripts'))
    from generate_world import make_world
    from navigation_assets import (navigation_world, write_map, navigation_parameters,
                                   terrain_parameters, LIDAR_XYZ, LIDAR_RPY)
    from xml.etree import ElementTree as ET
    directory = Path(LaunchConfiguration('artifacts_dir').perform(context))
    directory.mkdir(parents=True, exist_ok=True)
    root = navigation_world(make_world(dense=True, mobile=True))
    ET.indent(root)
    world = directory/'navigation.world'
    ET.ElementTree(root).write(world, encoding='utf-8', xml_declaration=True)
    map_file = write_map(root, directory)
    config_path = LaunchConfiguration('base_nav_config').perform(context)
    config = navigation_parameters(yaml.safe_load(Path(config_path).read_text()))
    params = directory/'nav2.yaml'
    params.write_text(yaml.safe_dump(config))
    terrain = terrain_parameters(yaml.safe_load((share/'config/rog_map_dense_lab.yaml').read_text()))
    p = terrain['sentry_terrain']['ros__parameters']
    p.update(cloud_topic='/passage_sim/body_points', odom_topic='/passage_sim/base_odom',
             input_frame='base_link', sensor_offset=list(LIDAR_XYZ))
    p['self_filter'] = dict(enabled=True, box=[-.33, .31, -.30, .30, -.12, .20])
    terrain_file = directory/'terrain.yaml'
    terrain_file.write_text(yaml.safe_dump(terrain))
    nav_share = get_package_share_directory('rm_navigation')
    def fixed(name, parent, child, xyz, rpy):
        return Node(package='tf2_ros', executable='static_transform_publisher', name=name,
                    arguments=['--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
                               '--roll', str(rpy[0]), '--pitch', str(rpy[1]), '--yaw', str(rpy[2]),
                               '--frame-id', parent, '--child-frame-id', child],
                    parameters=[{'use_sim_time': True}])
    return [
        ExecuteProcess(cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
                            '-s', 'libgazebo_ros_factory.so', str(world)], output='screen'),
        ExecuteProcess(cmd=['gzclient'], condition=IfCondition(LaunchConfiguration('gui'))),
        fixed('map_to_world', 'map', 'world', (0,0,0), (0,0,0)),
        fixed('lidar_mount', 'base_link', 'passage_lidar', LIDAR_XYZ, LIDAR_RPY),
        Node(package='sentry_passage_sim', executable='sim_support.py',
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='sentry_terrain_bridge', executable='terrain_node', name='sentry_terrain',
             parameters=[str(terrain_file)], output='screen'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{'use_sim_time': True, 'yaml_filename': map_file}], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_map', parameters=[{'use_sim_time': True,
                  'autostart': True, 'node_names': ['map_server']}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'navigation_launch.py')),
            launch_arguments={'use_sim_time': 'true', 'params_file': str(params),
                              'use_composition': 'False', 'autostart': 'true',
                              'cmd_vel_output': '/passage_sim/smoothed_cmd_vel'}.items()),
        Node(package='rviz2', executable='rviz2', parameters=[{'use_sim_time': True}],
             arguments=['-d', str(share/'rviz/navigation_lab.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    bringup = get_package_share_directory('rm_nav_bringup')
    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MASTER_URI', 'http://127.0.0.1:11359'),
        SetEnvironmentVariable('GAZEBO_IP', '127.0.0.1'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('base_nav_config', default_value=os.path.join(
            bringup, 'config/reality/nav2_params_rog_map_real.yaml')),
        DeclareLaunchArgument('artifacts_dir', default_value=tempfile.mkdtemp(prefix='passage-nav-sim-')),
        OpaqueFunction(function=start),
    ])
