"""Persist the exact admitted area compactly; never bridge a real gap."""
import copy
import json
import tempfile
import unittest
from pathlib import Path
from continuous_passage_memory import compact_regions,ContinuousPassageMemory
from passage_record_io import packed_body,read_record
from passage_region import polygon_cells
import test_memory_reassociation as fixtures

class CompactionTests(unittest.TestCase):
 def test_overlap_touch_gap_and_different_rows(self):
  regions={(0.,0.,1.,1.),(.5,0.,2.,1.),(2.,0.,3.,1.),
           (3.+1e-12,0.,4.,1.),(0.,1.,1.,2.)}
  expected=[(0.,0.,3.,1.),(0.,1.,1.,2.),(3.+1e-12,0.,4.,1.)]
  self.assertEqual(compact_regions(regions),expected)
  self.assertEqual(compact_regions(expected),expected)
  self.assertEqual(compact_regions([]),[])
 def test_raster_union_identical_across_origin_changes(self):
  regions={(x*.05,y*.05,(x+1)*.05,(y+1)*.05)
           for x in range(3,12)for y in range(4,10)if not(x>7 and y>7)}
  def raster(rs,geo):
   out=set()
   for x0,y0,x1,y1 in rs:out.update(polygon_cells([[x0,y0],[x1,y0],[x1,y1],[x0,y1]],geo))
   return out
  for dy in [0.,.014969914395491,-.0209495984318145]:
   geo=(30,30,.05,-.1,-.1+dy)
   self.assertEqual(raster(regions,geo),raster(compact_regions(regions),geo))
 def test_large_history_saves_without_raising_capacity_or_changing_live_regions(self):
  c,g,m=fixtures.MemoryReassociationTests().fixture()
  m.regions.update((-.1+i*1e-9,-.1,-.05+i*1e-9,-.05)for i in range(95000))
  before=set(m.regions);legacy=m.body();legacy['correction_regions']=sorted(before)
  with self.assertRaisesRegex(ValueError,'RECORD_TOO_LARGE'):packed_body(legacy)
  self.assertLess(len(packed_body(m.body())),100000)
  with tempfile.TemporaryDirectory()as folder:
   path=Path(folder)/'memory.json';m.save(path);saved=read_record(path)
   restored=ContinuousPassageMemory.restore(path,c,g)
   self.assertEqual(restored.state,'KNOWN_PASSAGE')
   self.assertEqual(restored.groups,m.groups)
  self.assertEqual(m.regions,before)
 def test_record_size_guard_and_explicit_write_fault_remain(self):
  with self.assertRaisesRegex(ValueError,'RECORD_TOO_LARGE'):packed_body({'data':'x'*4000001})
  c,g,m=fixtures.MemoryReassociationTests().fixture()
  with tempfile.TemporaryDirectory()as folder:
   path=Path(folder)/'missing'/'record.json'
   with self.assertRaises(OSError):m.save(path)
   self.assertEqual(m.state,'SUSPENDED');self.assertTrue(m.diagnostics()['record_error'])

if __name__=='__main__':unittest.main(verbosity=2)
