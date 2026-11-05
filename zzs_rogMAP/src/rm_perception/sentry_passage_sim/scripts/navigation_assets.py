"""Derive isolated navigation inputs; never rewrite the real vehicle YAML/map."""
import copy
import math
from pathlib import Path
import yaml
from generate_world import tag

FOOTPRINT = '[[-0.32, -0.29], [0.30, -0.29], [0.30, 0.29], [-0.32, 0.29]]'
LIDAR_XYZ = (-.32, 0., .066)
LIDAR_RPY = (-.6109, 0., math.pi / 2)


def navigation_world(root):
    """Reuse passage geometry/ray sensor, replacing only the mobile proxy."""
    rig = root.find(".//model[@name='sensor_rig']")
    rig.find('pose').text = '0 0 .11 0 0 0'
    body = rig.find('link')
    body.set('name', 'base_link')
    body.find('pose').text = '0 0 0 0 0 0'
    for kind in ('collision', 'visual'):
        part = body.find(f"{kind}[@name='robot_body']")
        part.find('pose').text = '-.01 0 .04 0 0 0'
        part.find('geometry/box/size').text = '.62 .58 .30'
    # Sensor is a distinct rigid link; its truth pose includes the actual offset.
    sensor = tag(rig, 'link', name='passage_lidar')
    tag(sensor, 'pose', ' '.join(map(str, (*LIDAR_XYZ, *LIDAR_RPY))))
    tag(sensor, 'gravity', 'false')
    sensor.append(body.find('sensor'))
    # CPU ray simulation: 34,560 rays/frame, not the dense perception fixture.
    sensor.find('sensor/ray/scan/horizontal/samples').text = '720'
    sensor.find('sensor/ray/scan/vertical/samples').text = '48'
    body.remove(body.find('sensor'))
    joint = tag(rig, 'joint', name='lidar_mount', type='fixed')
    tag(joint, 'parent', 'base_link')
    tag(joint, 'child', 'passage_lidar')
    drive = rig.find("plugin[@name='planar_move']")
    drive.find('publish_odom').text = 'true'
    drive.find('publish_odom_tf').text = 'true'
    tag(drive, 'publish_rate', 50)
    tag(drive, 'odometry_frame', 'world')
    tag(drive, 'robot_base_frame', 'base_link')
    tag(drive.find('ros'), 'remapping', 'odom:=base_odom')
    # Collision/contact observation is separate from point clouds.
    contact = tag(body, 'sensor', name='body_contact', type='contact')
    tag(contact, 'always_on', 'true')
    tag(contact, 'update_rate', 50)
    tag(tag(contact, 'contact'), 'collision', 'robot_body')
    plugin = tag(contact, 'plugin', name='contacts', filename='libgazebo_ros_bumper.so')
    tag(tag(plugin, 'ros'), 'namespace', '/passage_sim')
    tag(plugin, 'frame_name', 'base_link')
    return root


def obstacle_boxes(root):
    """Floor navigation truth: all static boxes intersecting the 30 cm body."""
    result = []
    for model in root.findall('.//world/model'):
        if model.get('name') == 'ground' or model.findtext('static') != 'true':
            continue
        mx, my, mz, *rotation = map(float, model.findtext('pose').split())
        if any(abs(v) > 1e-9 for v in rotation):
            raise ValueError('Only axis-aligned synthetic structures are supported')
        for collision in model.findall('link/collision'):
            px, py, pz, *_ = map(float, collision.findtext('pose').split())
            sx, sy, sz = map(float, collision.findtext('geometry/box/size').split())
            if mz + pz - sz/2 >= .30 or mz + pz + sz/2 <= 0:
                continue
            result.append((mx+px-sx/2, my+py-sy/2, mx+px+sx/2, my+py+sy/2))
    return result


def write_map(root, directory):
    # Derive BOTH map walls and physical walls from the same SDF, not a mask.
    resolution, size, origin = .02, 600, -6.
    obstacles = obstacle_boxes(root)
    data = bytearray([254]) * (size*size)
    for row in range(size):
        y = origin + (size-row-.5)*resolution
        for col in range(size):
            x = origin + (col+.5)*resolution
            if any(x+resolution/2 > x0 and x-resolution/2 < x1 and
                   y+resolution/2 > y0 and y-resolution/2 < y1
                   for x0, y0, x1, y1 in obstacles):
                data[row*size+col] = 0
    image = Path(directory)/'navigation.pgm'
    image.write_bytes(f'P5\n{size} {size}\n255\n'.encode()+data)
    path = Path(directory)/'navigation.yaml'
    path.write_text(yaml.safe_dump(dict(image=str(image), mode='trinary', resolution=resolution,
                                       origin=[origin, origin, 0.], negate=0,
                                       occupied_thresh=.65, free_thresh=.25)))
    return str(path)


def navigation_parameters(source):
    config = copy.deepcopy(source)
    def simulated(value):
        if isinstance(value, dict):
            for key, item in value.items():
                if key == 'use_sim_time':
                    value[key] = True
                else:
                    simulated(item)
    simulated(config)
    for name in ('controller_server', 'bt_navigator', 'velocity_smoother'):
        config[name]['ros__parameters']['odom_topic'] = '/passage_sim/base_odom'
    # Fail a displaced global endpoint rather than accepting a substitute 1 m away.
    config['planner_server']['ros__parameters']['GridBased']['tolerance'] = 0.0
    config['behavior_server'] = {'ros__parameters': dict(use_sim_time=True, global_frame='map',
        robot_base_frame='base_link', costmap_topic='local_costmap/costmap_raw',
        footprint_topic='local_costmap/published_footprint')}
    teb = config['controller_server']['ros__parameters']['FollowPath']
    teb['footprint_model'] = dict(type='polygon', vertices=FOOTPRINT)
    teb['feasibility_check_no_poses'] = -1
    teb['feasibility_check_lookahead_distance'] = -1.
    for name in ('local_costmap', 'global_costmap'):
        costmap = config[name][name]['ros__parameters']
        costmap.update(use_sim_time=True, global_frame='map', robot_base_frame='base_link',
                       footprint=FOOTPRINT, footprint_padding=0.,
                       plugins=['static_layer', 'terrain_layer', 'inflation_layer'])
        costmap['static_layer'] = dict(plugin='nav2_costmap_2d::StaticLayer', map_topic='/map',
                                      map_subscribe_transient_local=True,
                                      footprint_clearing_enabled=False)
        costmap['terrain_layer'] = dict(plugin='sentry_terrain_bridge::TerrainLayer',
                                       enabled=True, topic='/terrain/grid',
                                       data_timeout=1.5, marking_only=True)
    return config


def terrain_parameters(source):
    config = copy.deepcopy(source)
    params = config['sentry_terrain']['ros__parameters']
    params['clearance']['enabled'] = False
    # No legacy clearance permit, no navigation static correction.
    params['rog_map']['decay'] = dict(enable=True, keep_time=1., clear_time=3.,
                                     active_list_enable=True)
    return config
