import unittest
import numpy as np

from wall_bounded_region import wall_bounded_cells

class SegmentTests(unittest.TestCase):
 def setup_scene(self):return (80,60,.05,-1.,-.5)
 def ids(self,g,x0,x1,y0,y1):
  w,h,r,ox,oy=g;return {y*w+x for y in range(h)for x in range(w)if x0<ox+(x+.5)*r<x1 and y0<oy+(y+.5)*r<y1}
 def plane(self,tag,x,y0,y1):return dict(valid=True,tag=tag,origin=[x,(y0+y1)/2],normal=[1.,0.],support_xy=[[x,y0],[x,y1]],support_z_min=0.,support_z_max=.4,wall_span_limits=[.26,.60])
 def test_shifted_wall_segments_do_not_delete_next_segments_wall(self):
  g=self.setup_scene();obs=self.ids(g,-.5,.5,0.,1.)|self.ids(g,-.5,.7,1.,2.)
  planes=[self.plane(0,-.5,0.,1.),self.plane(1,.5,0.,1.)]
  for tag,x in [(2,-.5),(3,.5)]:
   planes.append(dict(self.plane(tag,x,1.,2.),origin=[x+.1,1.5],normal=[1.,-.2],support_xy=[[x,1.],[x+.2,2.]]))
  cells,regions=wall_bounded_cells(g,[100]*(g[0]*g[1]),obs,planes,support_extent='observed_roof')
  # Whole cell at x=-.275, y=1.525 belongs to the second side-wall edge.
  bad=40*80+12
  self.assertNotIn(bad,cells)
  self.assertTrue(self.ids(g,0.,.2,.1,1.9)<=cells)
  self.assertGreaterEqual(len(regions),2)
 def test_voxel_end_support_clears_short_tunnel_last_row(self):
  g=self.setup_scene();obs=self.ids(g,-.5,.5,0.,1.05);planes=[self.plane(0,-.5,0.,1.045),self.plane(1,.5,0.,1.045)]
  cells,_=wall_bounded_cells(g,[100]*(g[0]*g[1]),obs,planes,support_extent='observed_roof')
  self.assertTrue(self.ids(g,-.4,.4,1.,1.05)<=cells)
 def test_disconnected_tunnels_do_not_clear_between_them(self):
  g=self.setup_scene();obs=self.ids(g,-.5,.5,0.,.7)|self.ids(g,1.,2.,1.,1.7);planes=[self.plane(0,-.5,0.,.7),self.plane(1,.5,0.,.7),self.plane(2,1.,1.,1.7),self.plane(3,2.,1.,1.7)]
  cells,_=wall_bounded_cells(g,[100]*(g[0]*g[1]),obs,planes,support_extent='observed_roof')
  self.assertFalse(cells&self.ids(g,.6,.9,0.,2.))
  self.assertTrue(self.ids(g,-.3,.3,.1,.6)<=cells);self.assertTrue(self.ids(g,1.2,1.8,1.1,1.6)<=cells)
if __name__=='__main__':unittest.main(verbosity=2)
