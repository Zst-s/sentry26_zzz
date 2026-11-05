import unittest
from wall_bounded_region import wall_bounded_cells
class Tests(unittest.TestCase):
 def fixture(self):
  w,h,r=80,40,.05;g=[0]*(w*h);obs={y*w+x for y in range(12,18)for x in range(12,18)}
  for x in range(5,26):g[14*w+x]=100
  planes=[dict(tag=i,valid=True,support_z_min=0.,support_z_max=.4,wall_span_limits=[.26,.60],origin=[x*r,0],normal=[1.,0.],support_xy=[[x*r,.4],[x*r,1.1]])for i,x in enumerate([5,26])]
  return (w,h,r,0.,0.),g,obs,set(range(w*h)),planes
 def test_interior_extends_with_square_observation(self):
  geo,g,o,f,p=self.fixture();cells,d=wall_bounded_cells(geo,g,o,p)
  self.assertIn(14*80+7,cells);self.assertIn(14*80+24,cells);self.assertEqual(len(d),1)
 def test_no_hollow_no_clearing(self):
  a,g,o,f,p=self.fixture();self.assertFalse(wall_bounded_cells(a,g,set(),p)[0])
 def test_missing_wall_no_clearing(self):
  a,g,o,f,p=self.fixture();self.assertFalse(wall_bounded_cells(a,g,o,p[:1])[0])
 def test_mapping_ignores_unknown_and_live_obstacle(self):
  a,g,o,f,p=self.fixture();blocked=14*80+7;f.remove(blocked);cells,_=wall_bounded_cells(a,g,o,p);self.assertIn(blocked,cells);self.assertIn(14*80+8,cells)
 def test_sides_preserved_when_live_wall(self):
  a,g,o,f,p=self.fixture();walls={y*80+x for y in range(40)for x in [4,5,25,26]}
  for i in walls:g[i]=100
  f-=walls;self.assertFalse(wall_bounded_cells(a,g,o,p)[0]&{y*80+x for y in range(40)for x in [3,27]})
 def test_depth_does_not_expand_beyond_hollow(self):
  a,g,o,f,p=self.fixture();i=30*80+15;g[i]=100;self.assertNotIn(i,wall_bounded_cells(a,g,o,p)[0])
 def test_two_holes_keep_separation(self):
  a,g,o,f,p=self.fixture();o|={y*80+x for y in range(12,18)for x in range(52,58)}
  for x in range(45,66):g[14*80+x]=100
  for x in range(26,45):g[14*80+x]=100
  p+=[dict(tag=i+2,valid=True,support_z_min=0.,support_z_max=.4,wall_span_limits=[.26,.60],origin=[x*.05,0],normal=[1.,0.],support_xy=[[x*.05,.4],[x*.05,1.1]])for i,x in enumerate([45,66])]
  cells,d=wall_bounded_cells(a,g,o,p);self.assertIn(14*80+7,cells);self.assertIn(14*80+63,cells);self.assertNotIn(14*80+35,cells);self.assertEqual(len(d),2)
 def test_no_remote_wall_support(self):
  a,g,o,f,p=self.fixture();p[0]['support_xy']=[[.25,2.],[.25,2.5]];self.assertFalse(wall_bounded_cells(a,g,o,p)[0])
 def test_invalid_wall_fails(self):
  a,g,o,f,p=self.fixture();p[0]['normal']=[0.,0.]
  with self.assertRaises(ValueError):wall_bounded_cells(a,g,o,p)
if __name__=='__main__':unittest.main(verbosity=2)
