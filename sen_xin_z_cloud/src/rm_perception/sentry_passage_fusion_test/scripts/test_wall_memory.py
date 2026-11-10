from pathlib import Path
import unittest,tempfile
from dataclasses import replace
import test_wall_bounded as fixtures
from continuous_passage_memory import ContinuousPassageMemory
from global_passage_memory import MapContext
from nav_msgs.msg import OccupancyGrid
class Integration(unittest.TestCase):
 def fixture(self):
  geo,grid,obs,free,planes=fixtures.Tests().fixture();w,h,r,ox,oy=geo
  # Memory persistence tests start from an actually associated full passage,
  # rather than asking a small remote roof patch to create lateral clearance.
  obs={y*w+x for y in range(12,18)for x in range(6,25)}
  for plane in planes:
   x=plane['origin'][0]
   plane['support_xy']=[[x,(y+.5)*r]for y in range(8,22)]
  ctx=MapContext('mapping-wall-test','map',w,h,r,ox,oy,0,0,0)
  observation=dict(frame='odom',measured_at=10.,resolution=r,columns=[((i%w+.5)*r,(i//w+.5)*r,.0,.35)for i in obs])
  m=ContinuousPassageMemory();m.add_structure(context=ctx,grid=grid,observation=observation,cells=tuple(obs),map_transform=(0,0,0,0,0,0,1),now=10.1)
  t=OccupancyGrid();t.header.frame_id='odom';t.header.stamp.sec=10;t.info.width=w;t.info.height=h;t.info.resolution=r;t.info.origin.orientation.w=1.;t.data=[0]*len(grid)
  return ctx,grid,planes,m,t
 def test_persistence_growth_and_missing_wall(self):
  ctx,g,p,m,t=self.fixture();target=14*80+7
  m.refine(ctx,g,t,(0,0,0,0,0,0,1),p);self.assertIn(target,m.planning_cells(ctx,g))
  t.header.stamp.sec+=1;t.data=[100]*len(g);m.refine(ctx,g,t,(0,0,0,0,0,0,1),[])
  self.assertIn(target,m.planning_cells(ctx,g)) # static memory persists; live layer independently blocks.
  with tempfile.TemporaryDirectory()as folder:
   path=Path(folder)/'memory.json';m.save(path);m=ContinuousPassageMemory.restore(path,ctx,g);self.assertIn(target,m.planning_cells(ctx,g))
   grown=replace(ctx,width=82,height=42);self.assertTrue(m.check(grown,[100]*(82*42)))
   self.assertIn(14*82+7,m.planning_cells(grown,[100]*(82*42)))
   self.assertFalse(m.check(replace(grown,epoch='mapping-other'),[100]*(82*42)))
 def test_mapping_admits_despite_live_obstacle_or_unknown(self):
  for value in [100,-1]:
   ctx,g,p,m,t=self.fixture();target=14*80+7;data=list(t.data);data[target]=value;t.data=data
   m.refine(ctx,g,t,(0,0,0,0,0,0,1),p);self.assertIn(target,m.planning_cells(ctx,g))
 def test_cached_region_keeps_mapping_interior(self):
  ctx,g,p,m,t=self.fixture();target=14*80+7;data=list(t.data);data[target]=100;t.data=data
  m.refine(ctx,g,t,(0,0,0,0,0,0,1),p);self.assertIn(target,m.mask_cells)
  data[target]=0;t.data=data;t.header.stamp.sec+=1
  m.refine(ctx,g,t,(0,0,0,0,0,0,1),p);self.assertIn(target,m.mask_cells)
 def test_cached_region_rechecks_static_map(self):
  ctx,g,p,m,t=self.fixture();target=14*80+7;g[target]=0
  m.refine(ctx,g,t,(0,0,0,0,0,0,1),p);self.assertNotIn(target,m.mask_cells)
  g[target]=100;t.header.stamp.sec+=1
  m.refine(ctx,g,t,(0,0,0,0,0,0,1),p);self.assertIn(target,m.mask_cells)
if __name__=='__main__':unittest.main(verbosity=2)
