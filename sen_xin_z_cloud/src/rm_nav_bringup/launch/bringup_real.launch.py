"""实机入口：Livox → ZLIO → backend → Nav2。"""

import os
from pathlib import Path
import copy
import tempfile
import uuid
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def as_bool(value):
    if value.lower() not in ('true', 'false', '1', '0'):
        raise ValueError(f'Expected true/false, got {value!r}')
    return value.lower() in ('true', '1')


def rigid_transform(translation, rotation):
    result = np.eye(4)
    result[:3, :3] = rotation.as_matrix()
    result[:3, 3] = translation
    if not np.isfinite(result).all():
        raise ValueError('Non-finite sensor extrinsics')
    return result


def sensor_geometry(robot_description, frontend_config):
    """复用当前 URDF 安装位置与 ZLIO 雷达→IMU 外参，不引入第二套标定值。"""
    robot = ET.fromstring(robot_description)
    joint = robot.find("./joint[@name='livox_joint']")
    if (joint is None or joint.get('type') != 'fixed'
            or joint.find('parent').get('link') != 'base_link'
            or joint.find('child').get('link') != 'livox_frame'):
        raise ValueError('Expected fixed livox_joint: base_link -> livox_frame')
    origin = joint.find('origin')
    base_from_lidar = rigid_transform(
        [float(v) for v in origin.get('xyz', '0 0 0').split()],
        Rotation.from_euler('xyz', [float(v) for v in origin.get('rpy', '0 0 0').split()]))
    with open(frontend_config, encoding='utf-8') as stream:
        extrinsic = yaml.safe_load(stream)['sensor']['body_from_lidar']
    imu_from_lidar = rigid_transform(
        extrinsic['translation'], Rotation.from_quat(extrinsic['rotation_xyzw']))
    return imu_from_lidar @ np.linalg.inv(base_from_lidar)


def include(package, filename, arguments):
    # 每个子 launch 使用自己的参数作用域，避免 mode/config 污染其他子系统。
    return GroupAction(actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(package), 'launch', filename)),
        launch_arguments=arguments.items())])


def mapping_rviz_config(source):
    """建图时显示实时修正图，同时保留可手动开启的原始图。"""
    with open(source, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    displays = config['Visualization Manager']['Displays']
    maps = [display for display in displays
            if display.get('Class') == 'rviz_default_plugins/Map'
            and display.get('Topic', {}).get('Value') == '/map']
    if len(maps) != 1:
        raise ValueError('Expected one /map display in RViz config')
    corrected = maps[0]
    raw = copy.deepcopy(corrected)
    raw.update(Name='Raw SLAM map (debug)', Enabled=False, Value=False)
    displays.insert(displays.index(corrected) + 1, raw)
    terrain = copy.deepcopy(corrected)
    terrain.update({'Name': 'ROGMap live terrain (odom)', 'Alpha': 0.4,
                    'Color Scheme': 'costmap', 'Draw Behind': False,
                    'Enabled': True, 'Value': True})
    terrain['Topic']['Value'] = '/terrain/grid'
    terrain['Topic']['Durability Policy'] = 'Volatile'
    if isinstance(terrain.get('Update Topic'), dict):
        terrain['Update Topic']['Value'] = '/terrain/grid_updates'
    displays.insert(displays.index(corrected) + 2, terrain)
    corrected['Name'] = 'Passage corrected map'
    corrected['Topic']['Value'] = '/passage/static_corrected_map'
    if isinstance(corrected.get('Update Topic'), dict):
        corrected['Update Topic']['Value'] = '/passage/static_corrected_map_updates'
    with tempfile.NamedTemporaryFile(mode='w', prefix='sentry-mapping-rviz-',
                                     suffix='.rviz', delete=False) as stream:
        yaml.safe_dump(config, stream, allow_unicode=True)
        return stream.name


def setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    mode = value('mode')
    if mode not in ('nav', 'mapping'):
        raise ValueError('mode must be nav or mapping')
    if value('lio') != 'zlio' or value('localization') != 'zlio':
        raise ValueError('This entry now uses Z-Localization; remove old lio/localization arguments')
    share = get_package_share_directory('rm_nav_bringup')
    config = os.path.join(share, 'config', 'reality')
    sim = as_bool(value('use_sim_time'))
    frontend_config = os.path.abspath(value('frontend_config'))
    backend_config = os.path.abspath(value('backend_config'))
    prior_map = os.path.abspath(value('prior_map'))
    grid_map = os.path.abspath(value('map'))
    if not Path(backend_config).is_file():
        raise ValueError(f'Backend config does not exist: {backend_config}')
    if mode == 'nav':
        if not Path(prior_map).is_dir():
            raise ValueError(f'prior_map must be a Z-Localization map bundle directory: {prior_map}')
        if not Path(grid_map).is_file():
            raise ValueError(f'Nav2 grid map does not exist: {grid_map}')

    # 当前 xacro 内的实际安装外参为准；旧 measurement YAML 未被该 xacro 使用。
    robot_description = xacro.process_file(os.path.join(
        share, 'urdf', 'sentry_robot_real.xacro')).toxml()
    imu_from_base = sensor_geometry(robot_description, frontend_config)
    base_from_imu = np.linalg.inv(imu_from_base)
    translation = imu_from_base[:3, 3].tolist()
    quaternion = Rotation.from_matrix(imu_from_base[:3, :3]).as_quat().tolist()
    body_rpy = Rotation.from_matrix(base_from_imu[:3, :3]).as_euler('xyz', degrees=True)
    # 后端 offset 定义为 IMU 原点指向车体原点、在车体系表达。
    body_offset = -base_from_imu[:3, 3]
    overrides = {
        'extends': backend_config,
        'common': {
            'frontend': 'zlio', 'map_frame': 'map', 'odom_frame': 'odom',
            'sensor_frame': 'imu', 'frame_topic': '/frontend/frame',
            'prediction_topic': '/frontend/odometry', 'session_topic': '/frontend/session',
            'input_cloud_in_local_frame': True,
            'body': {'frame': 'base_link',
                     'rotation': dict(zip(('roll_deg', 'pitch_deg', 'yaw_deg'), map(float, body_rpy))),
                     'offset': dict(zip(('x', 'y', 'z'), map(float, body_offset)))},
        },
        'uwb': {'enabled': as_bool(value('use_uwb'))},
        # map -> base_link 会与 odom -> imu -> base_link 争抢父节点。
        'output': {'legacy': {'publish_tf': False}},
        'trajectory': {'save_directory': os.path.abspath(value('save_directory'))},
    }
    with tempfile.NamedTemporaryFile(mode='w', prefix='sentry_zlio_', suffix='.yaml',
                                     delete=False) as stream:
        yaml.safe_dump(overrides, stream)
        backend_overlay = stream.name
    fake_velocity = as_bool(value('enable_fake_velocity'))
    nav_overlay = None
    mapping_rviz = None
    if mode == 'nav' and fake_velocity:
        with open(value('params_file'), encoding='utf-8') as stream:
            nav_params = yaml.safe_load(stream)
        def replace_base_frame(item):
            if isinstance(item, dict):
                for key, entry in item.items():
                    if key == 'robot_base_frame' and entry == 'base_link':
                        item[key] = 'base_link_fake'
                    else:
                        replace_base_frame(entry)
            elif isinstance(item, list):
                for entry in item:
                    replace_base_frame(entry)
        replace_base_frame(nav_params)
        nav_params['controller_server']['ros__parameters']['odom_topic'] = '/base_imu_fake'
        follow = nav_params['controller_server']['ros__parameters']['FollowPath']
        follow['translation_only'] = True
        follow['follow_global_plan_orientation'] = False
        follow['enable_homotopy_class_planning'] = False
        nav_params['controller_server']['ros__parameters']['general_goal_checker']['yaw_goal_tolerance'] = 3.141593
        with tempfile.NamedTemporaryFile(mode='w', prefix='sentry_fake_nav_', suffix='.yaml', delete=False) as stream:
            yaml.safe_dump(nav_params, stream, allow_unicode=True)
            nav_overlay = stream.name

    def cleanup(_context):
        Path(backend_overlay).unlink(missing_ok=True)
        if nav_overlay:
            Path(nav_overlay).unlink(missing_ok=True)
        if mapping_rviz:
            Path(mapping_rviz).unlink(missing_ok=True)
        return []

    actions = [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'use_sim_time': sim, 'robot_description': robot_description}]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='imu_to_base_link', parameters=[{'use_sim_time': sim}],
             arguments=[str(v) for v in translation + quaternion] + ['imu', 'base_link']),
        include('bringup', 'bringup.launch.py', {
            'frontend_config': frontend_config, 'backend_config': backend_overlay,
            'driver': 'false', 'input_type': 'livox',
            'lidar_topic': value('lidar_topic'), 'imu_topic': value('imu_topic'),
            'imu_acceleration_scale': value('imu_acceleration_scale'),
            'use_sim_time': value('use_sim_time'), 'coordinated_reset': 'true',
            'mode': 'localization' if mode == 'nav' else 'mapping',
            'prior_map': prior_map if mode == 'nav' else '', 'rviz': value('lio_rviz'),
        }),
        Node(package='rm_nav_bringup', executable='zlio_nav_adapter.py', output='screen',
             parameters=[{'use_sim_time': sim,
                          'imu_from_base_translation': translation,
                          'imu_from_base_quaternion': quaternion}]),
        Node(package='linefit_ground_segmentation_ros', executable='ground_segmentation_node',
             parameters=[os.path.join(config, 'segmentation_real.yaml'),
                         {'use_sim_time': sim, 'input_topic': '/frontend/cloud_deskewed'}]),
        Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
             name='pointcloud_to_laserscan',
             remappings=[('cloud_in', '/segmentation/obstacle'), ('scan', '/scan')],
             parameters=[{'use_sim_time': sim, 'target_frame': 'base_footprint',
                          'transform_tolerance': 0.01, 'min_height': -0.20, 'max_height': 0.5,
                          'angle_min': -3.14159, 'angle_max': 3.14159, 'angle_increment': 0.003,
                          'scan_time': 0.1, 'range_min': 0.48, 'range_max': 10.0,
                          'use_inf': True, 'inf_epsilon': 1.0}]),
    ]
    with open(os.path.join(config, 'rog_map_real.yaml'), encoding='utf-8') as stream:
        terrain_params = yaml.safe_load(stream)['sentry_terrain']['ros__parameters']
    with open(frontend_config, encoding='utf-8') as stream:
        lidar_extrinsic = yaml.safe_load(stream)['sensor']['body_from_lidar']
    terrain_params.update({
        'use_sim_time': sim,
        'cloud_topic': '/frontend/cloud_deskewed',
        'input_frame': 'livox_frame',
        'cloud_coordinates': 'lidar',
        'odom_topic': '/lidar_odometry',
        'sensor_offset': list(map(float, lidar_extrinsic['translation'])),
        'lidar_rotation_xyzw': list(map(float, lidar_extrinsic['rotation_xyzw'])),
        'max_cloud_odom_dt': 0.005,
        'clearance.enabled': False,
        'mapping_scan_enabled': False,
    })
    actions.append(Node(package='sentry_terrain_bridge', executable='terrain_node',
                        name='sentry_terrain', output='screen', parameters=[terrain_params]))
    if as_bool(value('driver')):
        actions.append(Node(
            package='livox_ros_driver2', executable='livox_ros_driver2_node',
            name='livox_lidar_publisher', output='screen',
            parameters=[{'use_sim_time': sim, 'xfer_format': 1, 'multi_topic': 0,
                         'data_src': 0, 'publish_freq': 10.0, 'output_data_type': 0,
                         'frame_id': 'livox_frame', 'user_config_path': value('driver_config'),
                         'lvx_file_path': '', 'cmdline_input_bd_code': 'livox0000000001'}],
            remappings=[('/livox/lidar', value('lidar_topic')),
                        ('/livox/imu', value('imu_topic'))]))
    if mode == 'nav':
        params = nav_overlay or value('params_file')
        navigation = include('rm_navigation', 'bringup_rm_navigation.py', {
            'use_sim_time': value('use_sim_time'), 'map': grid_map,
            'params_file': params, 'nav_rviz': value('nav_rviz'),
            'nav_cmd_vel_output': '/cmd_vel_nav_smoothed' if fake_velocity else '/cmd_vel'})
        actions.extend([
            include('rm_navigation', 'map_server_launch.py', {
                'use_sim_time': value('use_sim_time'), 'map': grid_map,
                'params_file': params, 'use_composition': 'False'}),
            navigation,
        ])
        if fake_velocity:
            actions.append(Node(package='rm_nav_bringup', executable='fake_vel_transform.py',
                output='screen', parameters=[{'use_sim_time': sim,
                    'input_cmd_vel_topic': '/cmd_vel_nav_smoothed',
                    'output_cmd_vel_topic': '/cmd_vel'}]))
        if as_bool(value('cruise')):
            actions.append(include('cruise', 'cruise.launch.py', {
                'params_file': value('cruise_params_file')}))
    else:
        # Z-Localization 独占 map -> odom；slam_toolbox 仅根据全局位姿生成二维图。
        actions.append(Node(
            package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
            parameters=[os.path.join(config, 'mapper_params_online_async_real.yaml'),
                        {'use_sim_time': sim, 'odom_frame': 'map', 'map_frame': 'map',
                         'transform_publish_period': 0.0, 'use_scan_matching': False,
                         # Bounded exact-time TF wait for delayed backend projection.
                         'scan_queue_size': 20, 'transform_timeout': 1.0,
                         'do_loop_closing': False}]))
        actions.append(Node(
            package='sentry_passage_fusion_test', executable='live_memory_runtime.py',
            namespace='passage', name='passage_memory',
            output='screen', parameters=[{
                'use_sim_time': sim,
                'map_session_id': 'mapping-' + uuid.uuid4().hex,
                'record_path': os.path.abspath(value('save_directory')) +
                               '_passage_memory.json',
                'continuous_structure_memory': True,
                'wall_bounded_correction': True,
                'require_localization_health': False,
            }]))
        if as_bool(value('nav_rviz')):
            mapping_rviz = mapping_rviz_config(os.path.join(
                get_package_share_directory('rm_navigation'), 'rviz', 'nav2.rviz'))
            actions.append(include('rm_navigation', 'rviz_launch.py',
                                   {'use_sim_time': value('use_sim_time'),
                                    'rviz_config': mapping_rviz}))
    return actions


def generate_launch_description():
    share = FindPackageShare('rm_nav_bringup')
    defaults = {
        'world': '328', 'mode': 'nav', 'use_sim_time': 'false',
        'lio': 'zlio', 'localization': 'zlio', 'lio_rviz': 'false', 'nav_rviz': 'true',
        'driver': 'true', 'use_uwb': 'false', 'cruise': 'true',
        'enable_fake_velocity': 'true',
        'lidar_topic': '/livox/lidar', 'imu_topic': '/livox/imu',
        'imu_acceleration_scale': '9.81',
        'frontend_config': PathJoinSubstitution([FindPackageShare('frontend'), 'config', 'lio.yaml']),
        'backend_config': PathJoinSubstitution([FindPackageShare('backend'), 'config', 'backend.yaml']),
        'driver_config': PathJoinSubstitution([share, 'config', 'reality', 'MID360_config.json']),
        'prior_map': PathJoinSubstitution([share, 'PCD', LaunchConfiguration('world')]),
        'map': [PathJoinSubstitution([share, 'map', LaunchConfiguration('world')]), '.yaml'],
        'save_directory': [os.path.join(os.getcwd(), 'maps'), '/', LaunchConfiguration('world'), '_raw'],
        'params_file': os.environ.get('SENTRY_NAV_PARAMS_FILE') or PathJoinSubstitution(
            [share, 'config', 'reality', 'nav2_params_real.yaml']),
        'cruise_params_file': PathJoinSubstitution(
            [FindPackageShare('cruise'), 'config', 'cruise.yaml']),
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(key, default_value=default) for key, default in defaults.items()),
        OpaqueFunction(function=setup),
    ])
