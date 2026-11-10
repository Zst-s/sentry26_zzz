import unittest
import numpy as np
from wall_observation import WallObservationHistory

class WallEvidenceTest(unittest.TestCase):
    def points(self):
        return np.array([[x,y,z]for x in [-.425,.425]for y in [-.075,-.025,.025,.075]for z in np.arange(.025,.426,.05)])
    def history(self):
        h=WallObservationHistory()
        for at in [1.,1.5]:h.add(self.points(),at,at,.05,(0,0,0,0,0,0,1))
        return h
    def test_two_distinct_frames(self):
        h=WallObservationHistory();h.add(self.points(),1.,1.,.05,(0,0,0,0,0,0,1));self.assertFalse(h.planes(1.))
        self.assertFalse(h.add(self.points(),1.,1.,.05,(0,0,0,0,0,0,1)))
        h.add(self.points(),1.5,1.5,.05,(0,0,0,0,0,0,1));self.assertEqual(len(h.planes(1.5)),2)
    def test_expiry(self):
        h=self.history();self.assertTrue(h.planes(1.5));self.assertFalse(h.planes(2.3));self.assertFalse(h.planes(10.))
    def test_clear(self):
        h=self.history();h.clear();self.assertFalse(h.planes(1.5));self.assertIsNone(h.last_stamp)
    def test_reject_bad_points_and_time(self):
        h=WallObservationHistory()
        with self.assertRaises(ValueError):h.add([[0,0,float('nan')]],1,1,.05,(0,0,0,0,0,0,1))
        with self.assertRaises(ValueError):h.add(self.points(),1,2,.05,(0,0,0,0,0,0,1))
        with self.assertRaises(ValueError):h.add(self.points(),1,1,0,(0,0,0,0,0,0,1))
    def test_resolution_change_resets(self):
        h=self.history();h.add(self.points(),2.,2.,.1,(0,0,0,0,0,0,1));self.assertEqual(len(h.frames),1);self.assertFalse(h.planes(2.))
    def test_cloud_layout(self):
        from sensor_msgs.msg import PointCloud2,PointField
        msg=PointCloud2();msg.header.frame_id='odom';msg.height=1;msg.width=1;msg.point_step=12;msg.row_step=16
        msg.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1)for i,n in enumerate('xyz')]
        msg.data=np.array([1,2,3],dtype='>f4').tobytes()+bytes(4);msg.is_bigendian=True
        np.testing.assert_equal(WallObservationHistory.cloud_points(msg,'odom'),[[1,2,3]])
        with self.assertRaises(ValueError):WallObservationHistory.cloud_points(msg,'map')
        msg.row_step=11
        with self.assertRaises(ValueError):WallObservationHistory.cloud_points(msg,'odom')

if __name__=='__main__':unittest.main()
