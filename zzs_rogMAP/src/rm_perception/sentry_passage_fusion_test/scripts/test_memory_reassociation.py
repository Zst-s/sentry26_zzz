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
 def test_large_shift_resolution_and_reload_still_reject(self):
  c,g,_=self.fixture()
  for bad in [replace(c,origin_y=c.origin_y+.051),replace(c,resolution=.1),replace(c,epoch='mapping-other')]:
   _,_,m=self.fixture();self.assertFalse(m.check(bad,g));self.assertTrue(m.groups);self.assertTrue(m.regions)
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
