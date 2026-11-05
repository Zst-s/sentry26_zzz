"""One entry point for measured perception, memory, Nav2, gate and optional serial.

Default is a complete dry-run chain: planner and gate run, chassis output zero,
serial absent. With enable_motion=true, Nav2 feeds trans_node2 directly; the
legacy gate and IO remain diagnostic only. No virtual evidence is accepted.
"""
import os
import copy
import hashlib
import json
import tempfile
import uuid
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def saved_map_identity(map_dir, name, localization, pcd_dir=None):
    """Bind cross-restart memory to the bytes actually loaded by localization."""
    if not name or os.path.basename(name) != name or name in ('.', '..'):
        raise ValueError('world must be a map basename')
    prefix = os.path.join(map_dir, name)
    yaml_path = prefix + '.yaml'
    with open(yaml_path, 'rb') as stream:
        yaml_bytes = stream.read()
    metadata = yaml.safe_load(yaml_bytes)
    image_name = metadata.get('image') if isinstance(metadata, dict) else None
    if not isinstance(image_name, str) or not image_name:
        raise ValueError('Selected map YAML has no image')
    image_path = image_name if os.path.isabs(image_name) else os.path.join(map_dir, image_name)
    files = [yaml_path, image_path]
    if localization == 'slam_toolbox':
        files.extend((prefix + '.posegraph', prefix + '.data'))
    elif localization == 'icp':
        if pcd_dir is None:
            raise ValueError('ICP requires the selected PCD directory')
        files.append(os.path.join(pcd_dir, name + '.pcd'))
    elif localization not in ('amcl', 'icp'):
        raise ValueError('localization must be slam_toolbox, amcl or icp')
    registration_path = prefix + '.registration.json'
    registration = None
    if os.path.lexists(registration_path):
        if os.path.getsize(registration_path) > 65536:
            raise ValueError('Map registration manifest is too large')
        with open(registration_path, encoding='utf-8') as stream:
            registration = json.load(stream)
        if (registration.get('version') != 1 or registration.get('lio') != 'pointlio'
                or registration.get('frame') != 'map'
                or registration.get('association', {}).get('frame') != 'map'
                or registration.get('association', {}).get('source_frame') != 'odom'):
            raise ValueError('Invalid map/PCD frame association')
    digest = hashlib.sha256()
    for path in files:
        with open(path, 'rb') as stream:
            digest.update(os.path.basename(path).encode())
            file_digest = hashlib.sha256()
            while chunk := stream.read(1024 * 1024):
                digest.update(chunk)
                file_digest.update(chunk)
        if registration is not None:
            folder = 'PCD' if localization == 'icp' and path == os.path.join(pcd_dir, name+'.pcd') else 'map'
            key = folder + '/' + os.path.basename(path)
            if registration.get('files', {}).get(key) != file_digest.hexdigest():
                raise ValueError('Map bundle changed after export: ' + key)
    if registration is not None:
        with open(registration_path, 'rb') as stream:
            digest.update(os.path.basename(registration_path).encode())
            digest.update(stream.read())
    return 'saved-map-' + digest.hexdigest()


def mapping_rviz_config(source):
    """Reuse the normal RViz layout, replacing only its static-map display."""
    with open(source,encoding='utf-8') as stream:
        config=yaml.safe_load(stream)
    changed=0
    def visit(displays):
        nonlocal changed
        for display in list(displays):
            topic=display.get('Topic',{})
            if (display.get('Class')=='rviz_default_plugins/Map' and
                    isinstance(topic,dict) and topic.get('Value') in ('/map','map')):
                raw=copy.deepcopy(display)
                raw.update(Name='Raw SLAM map (debug)',Enabled=False,Value=False)
                displays.append(raw)
                display['Name']='Static map (passage corrected)'
                topic['Value']='/passage/static_corrected_map'
                if isinstance(display.get('Update Topic'),dict):
                    display['Update Topic']['Value']='/passage/static_corrected_map_updates'
                changed+=1
            if (display.get('Class')=='rviz_default_plugins/Map' and
                    isinstance(topic,dict) and topic.get('Value')=='/terrain/grid'):
                topic['Value']='/passage/mapping_terrain'
            visit(display.get('Displays',[]))
    visit(config['Visualization Manager']['Displays'])
    if changed!=1:raise ValueError('Expected one static map display in RViz config')
    with tempfile.NamedTemporaryFile(mode='w',prefix='passage-mapping-',suffix='.rviz',delete=False) as stream:
        yaml.safe_dump(config,stream,allow_unicode=True)
        return stream.name


def setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)
    motion = arg('enable_motion') == 'true'
    sensors = arg('start_sensors') == 'true'
    height, length, width = (float(arg(key)) for key in
                             ('robot_height', 'robot_length', 'robot_width'))
    ground_offset = float(arg('ground_z_offset'))
    if (not all(.1 <= value <= 2 for value in (height, length, width)) or
            not -2 <= ground_offset <= 1):
        raise ValueError('Invalid geometry. These are physical dimensions, not preview dimensions.')
    bringup = get_package_share_directory('rm_nav_bringup')
    navigation = get_package_share_directory('rm_navigation')
    if arg('mode') not in ('mapping', 'nav'):
        raise ValueError('mode must be mapping or nav')
    saved_session = (saved_map_identity(
        os.path.join(bringup, 'map'), arg('world'), arg('localization'),
        os.path.join(bringup, 'PCD')) if arg('mode') == 'nav' else None)
    if arg('mode') == 'mapping' and arg('map_session_id'):
        raise ValueError('Mapping session ID is generated per run; use explicit handover after saving')
    if saved_session and arg('map_session_id') and arg('map_session_id') != saved_session:
        raise ValueError('Navigation map_session_id must match the loaded map files')
    source = os.path.join(bringup, 'config', 'reality', 'nav2_params_rog_map_real.yaml')
    with open(source, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    global_costmap = config['global_costmap']['global_costmap']['ros__parameters']
    footprint = yaml.safe_load(global_costmap['footprint'])
    center_x = (min(p[0] for p in footprint)+max(p[0] for p in footprint))/2
    center_y = (min(p[1] for p in footprint)+max(p[1] for p in footprint))/2
    mapping = arg('mode') == 'mapping'
    global_costmap['plugins'] = ['static_layer', 'terrain_layer', 'inflation_layer']
    global_costmap['static_layer']['map_topic'] = '/map'
    if mapping:
        global_costmap['static_layer'].update({
            'plugin': 'sentry_terrain_bridge::PairedStaticLayer',
            'allow_verified_evidence': True, 'evidence_timeout': 1.0,
            'map_subscribe_transient_local': True, 'subscribe_to_updates': False,
            'footprint_clearing_enabled': False})
        global_costmap.pop('static_correction_layer', None)
        for costmap in (global_costmap, config['local_costmap']['local_costmap']['ros__parameters']):
            costmap['terrain_layer']['topic'] = '/passage/mapping_terrain'
    else:
        global_costmap['static_correction_layer'] = {
            'plugin': 'sentry_terrain_bridge::StaticCorrectionLayer',
            'enabled': False, 'allow_verified_evidence': False, 'evidence_timeout': 1.0,
            'evidence_topic': '/passage/verified_static_clearing'}
    navigation_map = ''
    if not mapping:
        prefix = os.path.join(bringup, 'map', arg('world'))
        navigation_map = prefix + '.yaml'
        for costmap in (global_costmap, config['local_costmap']['local_costmap']['ros__parameters']):
            costmap['plugins'] = ['static_layer', 'terrain_layer', 'inflation_layer']
            costmap['track_unknown_space'] = True
            costmap['static_layer'] = {
                'plugin': 'nav2_costmap_2d::StaticLayer', 'enabled': True,
                'map_topic': '/map', 'map_subscribe_transient_local': True,
                'subscribe_to_updates': False, 'footprint_clearing_enabled': False,
                'use_maximum': False}
            costmap['terrain_layer']['marking_only'] = True
    # Full snapshots are required by the independent runtime corridor checker.
    config['local_costmap']['local_costmap']['ros__parameters']['always_send_full_costmap'] = True
    with tempfile.NamedTemporaryFile(mode='w', prefix='passage-nav2-', suffix='.yaml', delete=False) as stream:
        yaml.safe_dump(config, stream)
        params = stream.name
    session = saved_session or ('mapping-'+uuid.uuid4().hex)
    record_path = os.path.abspath(os.path.expanduser(arg('record_path')))
    common = {'use_sim_time': False}
    actions = []
    if not mapping:
        actions.append(LogInfo(msg='Navigation static base locked: '+navigation_map))
        actions.append(LogInfo(msg='Original static map preserved; live terrain only adds temporary obstacles.'))
    if sensors:
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(bringup, 'launch', 'bringup_real.launch.py')),
            launch_arguments={'mode': arg('mode'), 'lio': arg('lio'), 'world': arg('world'),
                'localization': arg('localization'), 'perception': 'rog_map', 'nav_rviz': 'false',
                'lio_rviz': 'false', 'use_sim_time': 'false', 'start_nav2': 'false',
                'passage_runtime': 'true', 'passage_virtual_preview': 'false',
                'navigation_map': navigation_map if not mapping else os.path.join(bringup, 'map', arg('world')+'.yaml'),
                'freeze_navigation_map': 'false' if mapping else 'true',
                'passage_robot_height': str(height),
                'passage_robot_length': str(length),
                'passage_robot_width': str(width)}.items()))
    rviz_config=os.path.join(navigation,'rviz','nav2.rviz')
    if mapping and arg('rviz')=='true':rviz_config=mapping_rviz_config(rviz_config)
    actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(navigation, 'launch', 'bringup_rm_navigation.py')),
        launch_arguments={'params_file': params, 'use_sim_time': 'false',
                          'nav_rviz': arg('rviz'), 'rviz_config': rviz_config, 'use_composition': 'False',
                          'cmd_vel_output': '/passage/nav_cmd_vel'}.items()))
    for component, extra in (
            ('memory', {'map_session_id': session, 'record_path': record_path,
                        'continuous_structure_memory': True,
                        'wall_bounded_correction': True,
                        'require_localization_health': arg('lio') == 'pointlio'}),
            ('probe', {'ground_z_offset': ground_offset}),
            ('gate', {}), ('io', {'enable_motion': False,
                                'require_localization_health': arg('lio') == 'pointlio',
                                'robot_length': length, 'robot_width': width,
                                'footprint_padding': global_costmap['footprint_padding'],
                                'footprint_center_x': center_x, 'footprint_center_y': center_y})):
        if not mapping and component != 'io':
            continue  # Navigation consumes the saved base, never learns or publishes clearing masks.
        actions.append(Node(package='sentry_passage_fusion_test', executable='passage_runtime.py',
            namespace='passage', arguments=[component], parameters=[common, extra], output='screen'))
    if motion:
        actions.append(Node(package='decision', executable='trans_node2', name='passage_serial',
            parameters=[{'serial_port': arg('serial_port'), 'command_timeout': .25,
                         'require_odometry': arg('lio') == 'pointlio', 'odometry_timeout': .25,
                         'require_localization_health': arg('lio') == 'pointlio'}],
            # Only this process opens the port; legacy permits do not gate Nav2.
            remappings=[('/cmd_vel', '/passage/nav_cmd_vel')], output='screen'))
    return actions


def generate_launch_description():
    arguments = [
        ('mode', 'mapping'), ('lio', 'pointlio'), ('world', 'RMUL'),
        ('localization', 'slam_toolbox'), ('start_sensors', 'true'), ('rviz', 'true'),
        ('enable_motion', 'false'), ('serial_port', '/dev/ttyUSB0'),
        ('robot_height', '0.30'), ('robot_length', '0.62'),
        ('robot_width', '0.58'), ('ground_z_offset', '-0.11'),
        ('map_session_id', ''), ('record_path', '~/.ros/passage_memory/runtime.json')]
    return LaunchDescription([DeclareLaunchArgument(name, default_value=value)
                              for name, value in arguments] + [OpaqueFunction(function=setup)])
