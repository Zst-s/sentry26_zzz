#!/usr/bin/env python3
"""Original static-map export, identity and navigation overlay regressions."""
import copy
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace
from launch import LaunchContext
from ament_index_python.packages import get_package_share_directory
from slam_toolbox.srv import SerializePoseGraph
import export_pointlio_map as exporter_module

from nav_msgs.msg import OccupancyGrid
import yaml
from export_pointlio_map import Exporter, digest, publish_file
from paired_grid import freeze_static_pair
from grid_fingerprint import grid_fingerprint


def load_launch():
    p = Path(__file__).resolve().parents[1] / 'launch/passage_real.launch.py'
    spec = importlib.util.spec_from_file_location('frozen_launch', p)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class FrozenMapTests(unittest.TestCase):
    def setUp(self):
        self.grid = OccupancyGrid()
        self.grid.header.frame_id = 'map'
        self.grid.header.stamp.sec = 10
        self.grid.info.width, self.grid.info.height = 4, 2
        self.grid.info.resolution = .1
        self.grid.info.origin.orientation.w = 1.
        self.grid.info.map_load_time.sec = 2
        self.grid.data = [100, 100, 0, -1, 100, 80, 0, -1]
    def bundle(self, root):
        maps, pcd = root/'map', root/'PCD'
        maps.mkdir(); pcd.mkdir()
        meta = dict(image='demo.pgm', resolution=.1, origin=[0., 0., 0.],
                    negate=0, occupied_thresh=.65, free_thresh=.25, mode='trinary')
        (maps/'demo.yaml').write_text(yaml.safe_dump(meta))
        (maps/'demo.pgm').write_bytes(b'P5\n2 1\n255\n'+bytes([0, 254]))
        (maps/'demo_nav.yaml').write_text(yaml.safe_dump({**meta, 'image':'demo_nav.pgm'}))
        (maps/'demo_nav.pgm').write_bytes(b'P5\n2 1\n255\n'+bytes([254, 254]))
        (pcd/'demo.pcd').write_text('one shared test pcd')
        (maps/'demo.posegraph').write_bytes(b'graph'); (maps/'demo.data').write_bytes(b'data')
        manifest = dict(version=1, lio='pointlio', frame='map',
                        association=dict(frame='map', source_frame='odom'),
                        navigation_map=dict(yaml='demo_nav.yaml', image='demo_nav.pgm'), files={})
        for p in [*maps.iterdir(), *pcd.iterdir()]:
            manifest['files'][p.parent.name+'/'+p.name] = digest(p)
        (maps/'demo.registration.json').write_text(json.dumps(manifest))
        return maps, pcd, manifest

    def test_identity_validates_original_map_for_all_localizers(self):
        mod = load_launch()
        with tempfile.TemporaryDirectory() as directory:
            maps, pcd, _ = self.bundle(Path(directory))
            original = (maps/'demo.pgm').read_bytes()
            for mode in ('icp', 'amcl', 'slam_toolbox'):
                identity = mod.saved_map_identity(str(maps), 'demo', mode, str(pcd))
                self.assertTrue(identity.startswith('saved-map-'))
                (maps/'demo.pgm').write_bytes(b'tampered original')
                with self.assertRaises(ValueError):
                    mod.saved_map_identity(str(maps), 'demo', mode, str(pcd))
                (maps/'demo.pgm').write_bytes(original)

    def test_unused_baked_files_are_not_required_or_loaded(self):
        mod = load_launch()
        with tempfile.TemporaryDirectory() as directory:
            maps, pcd, _ = self.bundle(Path(directory))
            before = mod.saved_map_identity(str(maps), 'demo', 'icp', str(pcd))
            (maps/'demo_nav.yaml').unlink()
            (maps/'demo_nav.pgm').unlink()
            self.assertEqual(mod.saved_map_identity(str(maps), 'demo', 'icp', str(pcd)), before)

    def test_localization_artifact_tampering_is_still_rejected(self):
        mod = load_launch()
        with tempfile.TemporaryDirectory() as directory:
            maps, pcd, _ = self.bundle(Path(directory))
            for artifact, mode in ((pcd/'demo.pcd', 'icp'), (maps/'demo.data', 'slam_toolbox')):
                old = artifact.read_bytes()
                artifact.write_bytes(b'tampered localization artifact')
                with self.assertRaises(ValueError):
                    mod.saved_map_identity(str(maps), 'demo', mode, str(pcd))
                artifact.write_bytes(old)

    def test_exporter_subscribes_to_raw_map_and_coherent_mask_status(self):
        topics = []
        fake_node = SimpleNamespace(create_subscription=lambda kind, topic, *args:
                                    topics.append(topic))
        with patch.object(exporter_module.rclpy, 'create_node', return_value=fake_node):
            Exporter()
        self.assertEqual(topics, ['/map','/passage/verified_static_clearing','/passage/memory_status'])

    def test_publication_never_overwrites_existing_file(self):
        with tempfile.TemporaryDirectory() as directory:
            src, dst = Path(directory)/'source', Path(directory)/'dest'
            src.write_bytes(b'new'); dst.write_bytes(b'old')
            with self.assertRaises(FileExistsError):
                publish_file(src, dst)
            self.assertEqual(dst.read_bytes(), b'old')


    def fake_exporter(self, workspace):
        exporter = Exporter.__new__(Exporter)
        client = SimpleNamespace(wait_for_service=lambda timeout_sec: True)
        exporter.node = SimpleNamespace(create_client=lambda *args: client,
                                        destroy_client=lambda client: None)
        def call(kind, name, request, **kwargs):
            prefix = Path(request.filename)
            prefix.with_suffix('.posegraph').write_bytes(b'original graph')
            prefix.with_suffix('.data').write_bytes(b'original graph data')
            return SimpleNamespace(result=SerializePoseGraph.Response.RESULT_SUCCESS)
        exporter.call = call
        def snapshot(path):
            msg = copy.deepcopy(self.grid)
            path.write_bytes(b'one PCD, same map frame')
            mask=copy.deepcopy(msg);mask.data=[0]*len(msg.data)
            status=dict(virtual_preview=False,observation_source='rog_map_hollow_structure',map_associated=True,
                state='UNKNOWN',map_session_id='mapping-export-test',map_fingerprint=grid_fingerprint(msg),
                mask_fingerprint=grid_fingerprint(mask))
            exporter.static_snapshot=freeze_static_pair(msg,mask,status,10.)
            return msg, dict(frame='map', source_frame='odom', points=1,
                             stamp=[msg.header.stamp.sec, msg.header.stamp.nanosec])
        exporter.snapshot = snapshot
        def save_grid(grid, prefix):
            self.assertEqual(list(grid.data), list(self.grid.data))
            pixels = bytes(0 if x >= 65 else 205 if x < 0 else 254 for x in reversed(grid.data))
            prefix.with_suffix('.pgm').write_bytes(b'P5\n4 2\n255\n'+pixels)
            prefix.with_suffix('.yaml').write_text(yaml.safe_dump(dict(
                image=str(prefix.with_suffix('.pgm')), resolution=.1, origin=[0.,0.,0.],
                negate=0, occupied_thresh=.65, free_thresh=.25, mode='trinary')))
        exporter.save_grid = save_grid
        (workspace/'src/rm_nav_bringup').mkdir(parents=True)
        (workspace/'install/rm_nav_bringup/share/rm_nav_bringup').mkdir(parents=True)
        return exporter

    def test_export_keeps_raw_archive_and_uses_canonical_static_map(self):
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            exporter = self.fake_exporter(workspace)
            manifest = exporter.export(workspace, 'newmap')
            for base in (workspace/'src/rm_nav_bringup',
                         workspace/'install/rm_nav_bringup/share/rm_nav_bringup'):
                self.assertEqual(len(list((base/'PCD').iterdir())), 1)
                self.assertEqual(len(list((base/'map').iterdir())), 7)
                self.assertTrue((base/'map/newmap_raw.pgm').exists())
                self.assertIn('static_correction',manifest)
                self.assertFalse((base/'map/newmap_nav.pgm').exists())
                self.assertFalse((base/'map/newmap_nav.yaml').exists())
                self.assertNotIn('navigation_map', manifest)
                self.assertTrue(load_launch().saved_map_identity(
                    str(base/'map'), 'newmap', 'icp', str(base/'PCD')))
            original = (workspace/'src/rm_nav_bringup/map/newmap.pgm').read_bytes()
            with self.assertRaises(FileExistsError):
                exporter.export(workspace, 'newmap')
            self.assertEqual((workspace/'src/rm_nav_bringup/map/newmap.pgm').read_bytes(), original)

    def test_partial_publication_failure_rolls_back_entire_new_bundle(self):
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            exporter = self.fake_exporter(workspace)
            real = exporter_module.publish_file
            def fail(source, destination):
                if destination.name.endswith('.registration.json'):
                    raise OSError('injected publication failure')
                real(source, destination)
            with patch.object(exporter_module, 'publish_file', side_effect=fail):
                with self.assertRaises(OSError):
                    exporter.export(workspace, 'newmap')
            self.assertEqual(list(workspace.rglob('newmap*')), [])

    def test_actual_launch_policy_separates_mapping_and_navigation(self):
        mod = load_launch()
        with tempfile.TemporaryDirectory() as directory:
            bringup = Path(directory)
            maps, pcd, _ = self.bundle(bringup)
            config = Path(get_package_share_directory('rm_nav_bringup'))/'config/reality/nav2_params_rog_map_real.yaml'
            destination = bringup/'config/reality'
            destination.mkdir(parents=True)
            (destination/config.name).write_text(config.read_text())
            for mode in ('mapping', 'nav'):
                ctx = LaunchContext()
                ctx.launch_configurations.update(dict(mode=mode, world='demo', localization='icp',
                    lio='pointlio', enable_motion='false', start_sensors='true', robot_height='.3',
                    robot_length='.62', robot_width='.58', ground_z_offset='-.11', map_session_id='',
                    record_path=str(bringup/'memory.json'), rviz='false'))
                with patch.object(mod, 'get_package_share_directory', return_value=str(bringup)):
                    actions = mod.setup(ctx)
                includes = [a for a in actions if hasattr(a, 'launch_arguments')]
                nav_args = next(dict(a.launch_arguments) for a in includes
                                if 'params_file' in dict(a.launch_arguments))
                generated = Path(nav_args['params_file'])
                params = yaml.safe_load(generated.read_text())
                generated.unlink()
                global_map = params['global_costmap']['global_costmap']['ros__parameters']
                local_map = params['local_costmap']['local_costmap']['ros__parameters']
                if mode == 'nav':
                    for costmap in (global_map, local_map):
                        self.assertEqual(costmap['plugins'], ['static_layer','terrain_layer','inflation_layer'])
                        self.assertTrue(costmap['terrain_layer']['marking_only'])
                        self.assertFalse(costmap['static_layer']['footprint_clearing_enabled'])
                        self.assertFalse(costmap['static_layer']['subscribe_to_updates'])
                    self.assertFalse(global_map['static_correction_layer']['enabled'])
                    sensor_args = dict(includes[0].launch_arguments)
                    self.assertEqual(sensor_args['navigation_map'], str(maps/'demo.yaml'))
                    self.assertEqual(sensor_args['freeze_navigation_map'], 'true')
                else:
                    self.assertEqual(global_map['plugins'], ['static_layer','terrain_layer','inflation_layer'])
                    self.assertEqual(global_map['static_layer']['plugin'], 'sentry_terrain_bridge::PairedStaticLayer')
                    self.assertTrue(global_map['static_layer']['allow_verified_evidence'])
                    self.assertFalse(global_map['static_layer']['footprint_clearing_enabled'])
                    self.assertEqual(local_map['plugins'], ['terrain_layer','inflation_layer'])


if __name__ == '__main__':
    unittest.main()
