"""Static correction must survive display/export without changing raw evidence."""
import copy
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import yaml
from paired_grid import freeze_static_pair
from grid_fingerprint import grid_fingerprint
import test_frozen_navigation_map as fixtures
from test_frozen_navigation_map import load_launch
from live_memory_shadow import LiveMemoryShadow
from continuous_passage_memory import ContinuousPassageMemory


class StaticMapFlowTests(unittest.TestCase):
    def fixture(self):
        fixture = fixtures.FrozenMapTests(); fixture.setUp()
        grid = fixture.grid
        mask = copy.deepcopy(grid); mask.data = [100,0,0,100,100,0,0,0]
        status = dict(virtual_preview=False, observation_source='rog_map_hollow_structure',
            map_associated=True, state='KNOWN_PASSAGE', map_session_id='mapping-static-test',
            map_fingerprint=grid_fingerprint(grid), mask_fingerprint=grid_fingerprint(mask))
        return fixture, grid, mask, status

    def test_static_pair_only_changes_confirmed_occupied_cells(self):
        _, grid, mask, status = self.fixture(); before=copy.deepcopy(grid)
        raw, corrected, proof=freeze_static_pair(grid,mask,status,10.)
        self.assertEqual(grid,before);self.assertEqual(raw,before)
        self.assertEqual(list(corrected.data),[0,100,0,-1,0,80,0,-1])
        self.assertEqual(proof['cleared_static_cells'],2)
        self.assertEqual(corrected.header,raw.header)

    def test_stale_unassociated_or_wrong_pair_cannot_export(self):
        _, grid, mask, status=self.fixture()
        for changes in [dict(map_associated=False),dict(state='SUSPENDED'),
                        dict(map_fingerprint='bad'),dict(mask_fingerprint='bad'),
                        dict(state='UNKNOWN'),dict(virtual_preview=True)]:
            with self.assertRaises(ValueError):
                freeze_static_pair(grid,mask,{**status,**changes},10.)
        with self.assertRaises(ValueError):freeze_static_pair(grid,mask,status,11.01)
        mask.info.origin.position.x+=.01
        with self.assertRaises(ValueError):freeze_static_pair(grid,mask,status,10.)

    def test_corrected_bundle_and_raw_archive_are_distinct(self):
        fixture,grid,mask,status=self.fixture()
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);exporter=fixture.fake_exporter(root)
            snapshot=exporter.snapshot
            def corrected_snapshot(path):
                msg,association=snapshot(path)
                exporter.static_snapshot=freeze_static_pair(grid,mask,status,10.)
                return msg,association
            exporter.snapshot=corrected_snapshot
            saved={}
            def save_grid(msg,prefix):
                saved[prefix.name]=list(msg.data)
                prefix.with_suffix('.pgm').write_bytes(bytes(v+1 for v in msg.data))
                prefix.with_suffix('.yaml').write_text(yaml.safe_dump(dict(
                    image='temporary.pgm',resolution=.1,origin=[0,0,0],negate=0,
                    occupied_thresh=.65,free_thresh=.25,mode='trinary')))
            exporter.save_grid=save_grid
            proof=exporter.export(root,'corrected')
            self.assertEqual(saved['corrected'],[0,100,0,-1,0,80,0,-1])
            self.assertEqual(saved['corrected_raw'],list(grid.data))
            base=root/'src/rm_nav_bringup'
            self.assertTrue(load_launch().saved_map_identity(str(base/'map'),'corrected','icp',str(base/'PCD')))
            self.assertEqual(proof['association']['stamp'],[10,0])
            self.assertEqual(proof['static_correction']['cleared_static_cells'],2)

    def test_rviz_uses_corrected_mapping_layers_and_keeps_raw_disabled(self):
        config={'Visualization Manager':{'Displays':[
            {'Class':'rviz_default_plugins/Map','Name':'Map','Enabled':True,'Topic':{'Value':'/map'}},
            {'Class':'rviz_default_plugins/Map','Name':'Terrain','Topic':{'Value':'/terrain/grid'}}]}}
        with tempfile.TemporaryDirectory() as folder:
            source=Path(folder)/'normal.rviz';source.write_text(yaml.safe_dump(config))
            output=Path(load_launch().mapping_rviz_config(str(source)))
            try:
                displays=yaml.safe_load(output.read_text())['Visualization Manager']['Displays']
                self.assertEqual([d['Topic']['Value']for d in displays],
                    ['/passage/static_corrected_map','/passage/mapping_terrain','/map'])
                self.assertFalse(displays[-1]['Enabled'])
                self.assertEqual(yaml.safe_load(source.read_text()),config)
            finally:output.unlink()

    def test_real_status_producer_publishes_correction_then_raw_on_fault(self):
        _,grid,mask,_=self.fixture(); maps=[];statuses=[]
        memory=ContinuousPassageMemory();memory.state='KNOWN_PASSAGE'
        adapter=SimpleNamespace(memory=memory,last_entry_observation_reason='',
            map_session_id='mapping-static-test',current_map_associated=True,
            accept_virtual_preview=False,runtime=True,terrain_age=0.,wall_reason='',
            wall_plane_count=2,grid=grid,
            static_map_pub=SimpleNamespace(publish=maps.append),
            status=SimpleNamespace(publish=lambda msg:statuses.append(json.loads(msg.data))))
        LiveMemoryShadow.publish_status(adapter,'test',mask)
        self.assertEqual(list(maps[-1].data),[0,100,0,-1,0,80,0,-1])
        self.assertTrue(statuses[-1]['static_output_ready'])
        memory.suspend('MAP_ODOM_ASSOCIATION_MOVED')
        LiveMemoryShadow.publish_status(adapter,'test',mask)
        self.assertEqual(maps[-1],grid)
        self.assertFalse(statuses[-1]['static_output_ready'])


if __name__=='__main__':unittest.main(verbosity=2)
