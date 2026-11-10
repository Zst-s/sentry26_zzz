import unittest
import numpy as np
from nav_msgs.msg import OccupancyGrid
from mapping_terrain import clear_mapping_terrain
from continuous_wall_region import terrain_free_cells
class MappingTerrainTests(unittest.TestCase):
 def fixture(self):
  m=OccupancyGrid();m.header.frame_id='odom';m.header.stamp.sec=10;m.info.width=40;m.info.height=40;m.info.resolution=.05;m.info.origin.orientation.w=1.;m.data=[100]*1600
  return m,(40,40,float(m.info.resolution),0.,0.)
 def test_inside_obstacle_is_removed_and_outside_unchanged(self):
  m,g=self.fixture();cells={10*40+10,10*40+11};out=clear_mapping_terrain(m,g,(0,0,0,0,0,0,1),cells)
  self.assertTrue(cells<=terrain_free_cells(g,out,(0,0,0,0,0,0,1)))
  self.assertEqual(out.data[0],100);self.assertEqual(m.data[10*40+10],100)
  self.assertEqual(out.header,m.header)
 def test_unknown_inside_is_free(self):
  m,g=self.fixture();m.data=[-1]*1600;out=clear_mapping_terrain(m,g,(0,0,0,0,0,0,1),{410});self.assertEqual(out.data[410],0);self.assertEqual(out.data[0],-1)
 def test_rotated_overlap_matches_terrain_layer(self):
  m,g=self.fixture();yaw=.31;tr=(.05,-.03,0,0,0,np.sin(yaw/2),np.cos(yaw/2));cells={y*40+x for y in range(12,16)for x in range(12,16)}
  out=clear_mapping_terrain(m,g,tr,cells);self.assertTrue(cells<=terrain_free_cells(g,out,tr));self.assertEqual(out.data[0],100)
 def test_empty_region_and_invalid_inputs(self):
  m,g=self.fixture();self.assertEqual(clear_mapping_terrain(m,g,(0,0,0,0,0,0,1),set()).data,m.data)
  with self.assertRaises(ValueError):clear_mapping_terrain(m,g,(0,0,0,0,0,0,1),{1601})
  with self.assertRaises(ValueError):clear_mapping_terrain(m,g,(0,0,0,.3,0,0,.9539392014),{410})
if __name__=='__main__':unittest.main(verbosity=2)
