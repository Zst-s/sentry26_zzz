import copy,tempfile,unittest
from dataclasses import replace
from pathlib import Path
from continuous_passage_memory import ContinuousPassageMemory, compact_regions
from global_passage_memory import MapContext
from passage_memory_v2 import _is_mapping_expansion

class MemoryReassociationTests(unittest.TestCase):
 def fixture(self):
  c=MapContext('mapping-retain','map',20,20,.05,-.5,-.5,0,0,0)
  g=[100]*400;m=ContinuousPassageMemory()
  o=dict(frame='odom',measured_at=10.,resolution=.05,columns=[(.025+x*.05,.025+y*.05,0.,.35)for x in range(3)for y in range(3)])
  m.add_structure(context=c,grid=g,observation=o,cells=(),map_transform=(0,0,0,0,0,0,1),now=10.1)
  m._add_regions(c,{y*20+x for y in range(8,15)for x in range(7,14)})
  return c,g,m
 def test_same_size_micro_origin_shift_reprojects_without_losing_world_regions(self):
  for dy in [-.0209495984318145,-.014969914395491,.049,-.049]:
   c,g,m=self.fixture();regions=set(m.regions);groups=copy.deepcopy(m.groups)
   moved=replace(c,origin_y=c.origin_y+dy)
   self.assertFalse(_is_mapping_expansion(c,moved,max_edge_retraction=.05)) # legacy unchanged
   self.assertTrue(m.check(moved,g));self.assertEqual(regions,m.regions);self.assertEqual(groups,m.groups)
   for x0,y0,x1,y1 in regions:
    i=int(((x0+x1)/2-moved.origin_x)//moved.resolution);j=int(((y0+y1)/2-moved.origin_y)//moved.resolution)
    self.assertIn(j*moved.width+i,m.cells)
 def test_resolution_reload_and_tf_still_reject(self):
  c,g,_=self.fixture()
  for bad in [replace(c,resolution=.1),replace(c,epoch='mapping-other'),
              replace(c,map_to_odom_x=.251),replace(c,map_to_odom_yaw=.151),
              replace(c,map_to_odom_z=.251),replace(c,frame='odom')]:
   _,_,m=self.fixture();self.assertFalse(m.check(bad,g));self.assertTrue(m.groups);self.assertTrue(m.regions)
 def test_crop_then_expand_keeps_both_tunnels_and_side_wall(self):
  from continuous_passage_memory import cell_regions
  c,g,m=self.fixture()
  # Two separate admitted interiors; a side-wall cell overlaps an old region.
  second={y*c.width+x for y in range(3,6)for x in range(2,5)}
  m._add_regions(c,second)
  wall=10*c.width+7
  m.wall_regions.update(cell_regions(c,{wall}));m.wall_cells.add(wall);m.cells.discard(wall)
  self.assertTrue(m.check(c,g));expected=set(m.mask_cells)
  groups=copy.deepcopy(m.groups);regions=set(m.regions);walls=set(m.wall_regions)
  small=replace(c,width=10,height=10)
  self.assertTrue(m.check(small,[100]*100))
  self.assertTrue(m.mask_cells);self.assertLess(len(m.mask_cells),len(expected))
  self.assertEqual(m.groups,groups);self.assertEqual(m.regions,regions);self.assertEqual(m.wall_regions,walls)
  with tempfile.TemporaryDirectory()as folder:
   path=Path(folder)/'cropped.json';m.save(path)
   restored=ContinuousPassageMemory.restore(path,small,[100]*100)
   self.assertEqual(restored.state,'KNOWN_PASSAGE')
   self.assertTrue(restored.check(c,g));self.assertEqual(restored.mask_cells,expected)
   self.assertNotIn(wall,restored.mask_cells)
   self.assertTrue(second<=restored.mask_cells)
   self.assertEqual(restored.groups,groups);self.assertEqual(compact_regions(restored.regions),compact_regions(regions))
 def test_origin_shift_beyond_old_one_cell_limit_and_out_of_view_return(self):
  c,g,m=self.fixture();self.assertTrue(m.check(c,g));expected=set(m.mask_cells)
  regions=set(m.regions);groups=copy.deepcopy(m.groups)
  for offset in (.051,.073,2.):
   moved=replace(c,origin_y=c.origin_y+offset)
   self.assertTrue(m.check(moved,g));self.assertEqual(m.state,'KNOWN_PASSAGE')
   self.assertEqual(m.regions,regions);self.assertEqual(m.groups,groups)
   if offset==2.:self.assertFalse(m.mask_cells)
  self.assertTrue(m.check(c,g));self.assertEqual(m.mask_cells,expected)
 def test_old_geometry_suspension_recovers_through_same_association_checks(self):
  c,g,m=self.fixture();small=replace(c,width=18,height=19,origin_y=c.origin_y+.0724)
  m.suspend('MAP_RELOAD_OR_GEOMETRY_CHANGED')
  self.assertFalse(m.revalidate(replace(small,map_to_odom_x=.3),[100]*342))
  self.assertTrue(m.revalidate(small,[100]*342));self.assertEqual(m.context,small)
  self.assertEqual(m.reason,'HISTORY_ASSOCIATION_REVALIDATED')
 def test_saved_map_resize_and_map_value_changes_remain_rejected(self):
  for kind in ('resize','origin','values'):
   c,g,m=self.fixture();saved=replace(c,epoch='saved-map-frozen');m.context=saved
   self.assertTrue(m.check(saved,g))
   if kind=='resize':bad=replace(saved,width=19);grid=[100]*380
   elif kind=='origin':bad=replace(saved,origin_y=saved.origin_y+.001);grid=g
   else:
    bad=saved;grid=g.copy();grid[min(m.cells)]=0
   self.assertFalse(m.check(bad,grid));self.assertEqual(m.state,'SUSPENDED')
 def test_fault_retains_history_and_only_verified_context_recovers(self):
  c,g,m=self.fixture();regions=set(m.regions);groups=copy.deepcopy(m.groups)
  bad=replace(c,map_to_odom_x=1.)
  self.assertFalse(m.check(bad,g));self.assertFalse(m.revalidate(bad,g))
  self.assertEqual(m.regions,regions);self.assertEqual(m.groups,groups)
  self.assertTrue(m.revalidate(c,g));self.assertEqual(m.regions,regions);self.assertEqual(m.groups,groups)
  with tempfile.TemporaryDirectory()as folder:
   p=Path(folder)/'memory.json';m.save(p);restored=ContinuousPassageMemory.restore(p,c,g);self.assertEqual(compact_regions(restored.regions),compact_regions(regions));self.assertEqual(restored.cells,m.cells);self.assertEqual(restored.mask_cells,m.mask_cells)
 def test_storage_fault_does_not_auto_resume(self):
  c,g,m=self.fixture();m.suspend('RECORD_WRITE_FAILED');self.assertFalse(m.revalidate(c,g));self.assertEqual(m.state,'SUSPENDED')
 def test_tf_unavailable_can_resume_but_new_session_cannot(self):
  c,g,m=self.fixture();m.suspend('MAP_OR_TF_UNAVAILABLE');self.assertFalse(m.revalidate(replace(c,epoch='mapping-other'),g));self.assertTrue(m.revalidate(c,g))
 def test_adapter_does_not_replace_suspended_history_on_new_observation(self):
  from types import SimpleNamespace
  from unittest.mock import patch
  from geometry_msgs.msg import TransformStamped
  from live_memory_shadow import LiveMemoryShadow
  from test_structure_observation import cloud
  c,g,m=self.fixture();old=m;regions=set(m.regions)
  tf=TransformStamped();tf.transform.rotation.w=1.
  adapter=SimpleNamespace(pending_structure=cloud(),last_structure_stamp=0.,
   last_invalidation_time=0.,expected_frame='odom',map_session_id=c.epoch,
   memory=m,grid=SimpleNamespace(data=g),lookup=lambda stamp:tf)
  bad=replace(c,map_to_odom_x=1.)
  m.check(bad,g)
  with patch('live_memory_shadow.map_context',return_value=bad):
   LiveMemoryShadow.accept_structure(adapter,bad,10.1)
  self.assertIs(adapter.memory,old);self.assertEqual(m.regions,regions)
  self.assertEqual(m.state,'SUSPENDED');self.assertEqual(adapter.reason,'HISTORY_ASSOCIATION_UNAVAILABLE')
  adapter.pending_structure=cloud()
  with patch('live_memory_shadow.map_context',return_value=c):
   LiveMemoryShadow.accept_structure(adapter,c,10.1)
  self.assertIs(adapter.memory,old);self.assertEqual(m.regions,regions)
  self.assertEqual(m.state,'KNOWN_PASSAGE')
if __name__=='__main__':unittest.main(verbosity=2)
