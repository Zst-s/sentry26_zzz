#!/usr/bin/env python3
"""Frozen range extraction, publication rollback and map-bound reader checks."""
import copy
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from nav_msgs.msg import OccupancyGrid
from passage_ranges import passage_regions, load_passage_ranges, file_digest
from paired_grid import freeze_static_pair
from grid_fingerprint import grid_fingerprint

# Source-tree tests also work without installing the exporter executable.
source = Path(__file__).resolve()
if source.parent.name == 'scripts':
    sys.path.insert(0, str(source.parents[3] / 'rm_nav_bringup/scripts'))
import export_mapping_map as exporter


def frozen(cells=(0, 1, 5, 15)):
    raw = OccupancyGrid()
    raw.header.frame_id = 'map'
    raw.header.stamp.sec = 10
    raw.info.width = raw.info.height = 4
    raw.info.resolution = 0.5
    raw.info.origin.position.x = -1.0
    raw.info.origin.position.y = -2.0
    raw.info.origin.orientation.w = 1.0
    raw.data = [100] * 15 + [-1]
    mask = copy.deepcopy(raw)
    mask.header.stamp.sec = 11
    mask.data = [100 if i in cells and raw.data[i] == 100 else 0 for i in range(16)]
    status = dict(virtual_preview=False, observation_source='rog_map_hollow_structure',
                  map_associated=True, state='KNOWN_PASSAGE' if cells else 'UNKNOWN',
                  map_session_id='mapping-ranges', map_fingerprint=grid_fingerprint(raw),
                  mask_fingerprint=grid_fingerprint(mask))
    raw, corrected, proof = freeze_static_pair(raw, mask, status, 11.1)
    return raw, corrected, proof, passage_regions(mask, proof)


class FakeWorker:
    def __init__(self):
        self.node = SimpleNamespace(destroy_node=lambda: None)

    def call(self, service, name, request, timeout):
        if name == '/backend_node/save_map':
            bundle = self.workspace / 'maps' / (self.name + '_raw')
            bundle.mkdir(parents=True)
            for item in ('map.pcd', 'keyframes.txt'):
                (bundle / item).write_text('fixture\n')
            return SimpleNamespace(success=True, message='saved runtime map to ' + str(bundle))
        if name == '/slam_toolbox/serialize_map':
            for ext in ('.posegraph', '.data'):
                Path(request.filename + ext).write_text('graph\n')
            return SimpleNamespace(result=0)
        return SimpleNamespace(success=True)

    def capture(self):
        return frozen()

    def save_grid(self, grid, prefix):
        import yaml
        prefix.with_suffix('.pgm').write_bytes(b'P5\n4 4\n255\n' + bytes([0] * 16))
        prefix.with_suffix('.yaml').write_text(yaml.safe_dump(dict(
            image=prefix.name + '.pgm', resolution=float(grid.info.resolution),
            origin=[grid.info.origin.position.x, grid.info.origin.position.y, 0.0])))


class PassageRangesTests(unittest.TestCase):
    def export(self, workspace, name='trial', fail_link=False):
        worker = FakeWorker()
        worker.workspace, worker.name = workspace, name
        real_link = exporter.os.link
        count = 0

        def link(src, dst):
            nonlocal count
            count += 1
            if fail_link and count == 3:
                raise OSError('injected publication failure')
            return real_link(src, dst)

        with patch.object(exporter, 'Exporter', return_value=worker), \
                patch.object(exporter.rclpy, 'init'), patch.object(exporter.rclpy, 'shutdown'), \
                patch.object(exporter.os, 'link', side_effect=link):
            exporter.export(workspace, name)

    def workspace(self, directory):
        workspace = Path(directory)
        for folder in ('map', 'PCD'):
            (workspace / 'src/rm_nav_bringup' / folder).mkdir(parents=True)
        return workspace

    def test_exact_regions_no_diagonal_merge_and_unknown_preserved(self):
        raw, corrected, proof, ranges = frozen((0, 1, 5, 10, 15))
        self.assertEqual([r['cell_count'] for r in ranges['regions']], [3, 1])
        self.assertEqual(ranges['regions'][0]['cell_spans_y_xbegin_xend'], [[0, 0, 2], [1, 1, 2]])
        self.assertEqual(ranges['regions'][0]['bounds_xy'], [-1., -2., 0., -1.])
        self.assertEqual(corrected.data[15], -1)
        self.assertEqual(raw.data[0], 100)
        self.assertEqual(corrected.data[0], 0)

    def test_empty_and_rotated_ranges(self):
        self.assertEqual(frozen(())[3]['regions'], [])
        raw = frozen()[0]
        raw.info.origin.orientation.z = math.sin(math.pi / 4)
        raw.info.origin.orientation.w = math.cos(math.pi / 4)
        raw.data = [100] + [0] * 15
        ranges = passage_regions(raw, frozen()[2])
        self.assertAlmostEqual(ranges['regions'][0]['center_xy'][0], -1.25)
        self.assertAlmostEqual(ranges['regions'][0]['center_xy'][1], -1.75)

    def test_export_reader_and_all_hashes(self):
        with tempfile.TemporaryDirectory() as directory:
            w = self.workspace(directory)
            self.export(w)
            m = w / 'src/rm_nav_bringup/map'
            manifest = json.loads((m / 'trial.registration.json').read_text())
            root = m.parent
            for rel, digest in manifest['files'].items():
                self.assertEqual(file_digest(root / rel), digest)
            self.assertIn('map/trial.passages.json', manifest['files'])
            self.assertEqual(load_passage_ranges(m / 'trial.yaml')['regions'][0]['cell_count'], 3)
            # A changed YAML, image or range must never silently reach the decision reader.
            for suffix in ('.yaml', '.pgm', '.passages.json'):
                path = m / ('trial' + suffix)
                previous = path.read_bytes()
                path.write_bytes(previous + b'\n')
                with self.assertRaises(ValueError):
                    load_passage_ranges(m / 'trial.yaml')
                path.write_bytes(previous)

    def test_stale_proof_rejected_even_with_updated_hash(self):
        with tempfile.TemporaryDirectory() as directory:
            w = self.workspace(directory)
            self.export(w)
            m = w / 'src/rm_nav_bringup/map'
            p = m / 'trial.passages.json'
            ranges = json.loads(p.read_text())
            ranges['map_session_id'] = 'mapping-other'
            p.write_text(json.dumps(ranges))
            reg = m / 'trial.registration.json'
            manifest = json.loads(reg.read_text())
            manifest['files']['map/' + p.name] = file_digest(p)
            reg.write_text(json.dumps(manifest))
            with self.assertRaisesRegex(ValueError, 'proof'):
                load_passage_ranges(m / 'trial.yaml')

    def test_range_only_collision_prevents_service_calls(self):
        with tempfile.TemporaryDirectory() as directory:
            w = self.workspace(directory)
            p = w / 'src/rm_nav_bringup/map/trial.passages.json'
            p.write_text('existing')
            with patch.object(exporter, 'Exporter') as worker:
                with self.assertRaisesRegex(RuntimeError, '拒绝覆盖'):
                    exporter.export(w, 'trial')
                worker.assert_not_called()
            self.assertEqual(p.read_text(), 'existing')

    def test_publication_failure_rolls_back_ranges_and_maps(self):
        with tempfile.TemporaryDirectory() as directory:
            w = self.workspace(directory)
            with self.assertRaisesRegex(OSError, 'injected'):
                self.export(w, fail_link=True)
            root = w / 'src/rm_nav_bringup'
            self.assertEqual(list((root / 'map').iterdir()), [])
            self.assertEqual(list((root / 'PCD').iterdir()), [])


if __name__ == '__main__':
    unittest.main(verbosity=2)
