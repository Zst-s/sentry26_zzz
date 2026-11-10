import unittest,numpy as np

from roof_association import associated_wall_bounded_cells
from wall_bounded_region import wall_bounded_cells
from continuous_passage_memory import ContinuousPassageMemory

class AssociationTests(unittest.TestCase):
 def fixture(self):
  g=(100,50,.05,-3.5,-.5);w,h,r,ox,oy=g;ids={y*w+x for y in range(h)for x in range(w)if -.5<ox+(x+.5)*r<.5 and 0<oy+(y+.5)*r<1.}
  samples=np.array([[ox+(i%w+.5)*r,oy+(i//w+.5)*r,0.,ox+(i%w+.5)*r,oy+(i//w+.5)*r,.4]for i in ids])
  walls=[dict(tag=j,valid=True,origin=[x,.5],normal=[1.,0.],support_xy=[[x,.1],[x,.9]],support_z_min=0.,support_z_max=.4,wall_span_limits=[.26,.60])for j,x in enumerate((-3.,-.5,.5))]
  return g,ids,samples,walls
 def test_parallel_remote_wall_with_same_height_is_not_a_passage_side(self):
  g,ids,samples,w=self.fixture();cells,details=associated_wall_bounded_cells(g,[100]*5000,ids,[w[0],w[2]],samples,support_extent='observed_roof')
  self.assertFalse(cells);self.assertFalse(details)
 def test_adjacent_walls_of_the_classified_passage_are_accepted(self):
  g,ids,samples,w=self.fixture();cells,details=associated_wall_bounded_cells(g,[100]*5000,ids,w[1:],samples,support_extent='observed_roof')
  self.assertTrue(cells);self.assertTrue(details);self.assertTrue(cells<=ids)
 def test_walls_alone_cannot_create_a_tunnel_or_clear_cells(self):
  g,ids,samples,w=self.fixture();cells,details=associated_wall_bounded_cells(g,[100]*5000,set(),w,np.empty((0,6)),support_extent='observed_roof')
  self.assertFalse(cells);self.assertFalse(details)
  m=ContinuousPassageMemory();m.refine(None,[],None,None,w);self.assertFalse(m.regions)
 def test_wall_pair_cannot_expand_the_classified_roof_outer_boundary(self):
  g,ids,samples,w=self.fixture();# Direct geometry is also bounded, independently of admission.
  cells,_=wall_bounded_cells(g,[100]*5000,ids,[w[0],w[2]],support_extent='observed_roof')
  self.assertTrue(cells<=ids)
if __name__=='__main__':unittest.main(verbosity=2)
