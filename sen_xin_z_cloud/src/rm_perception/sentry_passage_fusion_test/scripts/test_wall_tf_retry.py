"""Exact-time wall evidence: retry pending future TF, never refresh history stamps."""
import copy
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import Buffer
from live_memory_shadow import LiveMemoryShadow, map_context
from wall_observation import WallObservationHistory


def transform(at, x=0.):
    t=TransformStamped();t.header.frame_id='map';t.child_frame_id='odom'
    t.header.stamp=Time(seconds=at).to_msg();t.transform.rotation.w=1.
    t.transform.translation.x=x
    return t


def cloud(at):
    p=np.asarray([[x,y,z] for x in [-.425,.425]
                  for y in [-.075,-.025,.025,.075]
                  for z in np.arange(.025,.426,.05)],dtype='<f4')
    m=PointCloud2();m.header.frame_id='odom';m.header.stamp=Time(seconds=at).to_msg()
    m.height=1;m.width=len(p);m.point_step=12;m.row_step=m.width*12
    m.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1)
              for i,n in enumerate('xyz')];m.data=p.tobytes()
    return m


class WallTfRetryTests(unittest.TestCase):
    def fixture(self):
        grid=OccupancyGrid();grid.header.frame_id='map';grid.info.width=20;grid.info.height=20
        grid.info.resolution=.05;grid.info.origin.orientation.w=1.;grid.data=[100]*400
        history=WallObservationHistory()
        for at in [10.,10.5]:history.add(history.cloud_points(cloud(at),'odom'),at,at,.05,(0,0,0,0,0,0,1))
        buffer=Buffer(cache_time=Duration(seconds=10.))
        for at in [10.,10.5,10.9]:buffer.set_transform(transform(at),'isolated')
        node=SimpleNamespace(wall_enabled=True,require_health=False,expected_frame='odom',
            wall_history=history,pending_wall=cloud(11.),grid=grid,map_session_id='mapping-retry',
            memory=SimpleNamespace(parts=lambda:iter([dict(resolution=.05)])),
            wall_plane_count=0,wall_reason='',
            lookup=lambda at:buffer.lookup_transform('map','odom',Time(seconds=at)))
        context=map_context(grid,transform(10.9),node.map_session_id,True)
        return node,context,buffer

    def test_future_tf_preserves_history_and_eventually_adds_original_stamp(self):
        n,c,b=self.fixture();message=n.pending_wall;saved=copy.deepcopy(message)
        frames=list(n.wall_history.frames);last=n.wall_history.last_stamp
        planes=LiveMemoryShadow.current_wall_planes(n,c,11.1)
        self.assertEqual(len(planes),2);self.assertIs(n.pending_wall,message)
        self.assertEqual(n.wall_reason,'WAIT_WALL_EXACT_TF')
        self.assertEqual(n.wall_history.last_stamp,last);self.assertEqual(len(n.wall_history.frames),len(frames))
        LiveMemoryShadow.on_wall(n,cloud(11.5));self.assertIs(n.pending_wall,message)
        b.set_transform(transform(11.1),'isolated')
        self.assertEqual(len(LiveMemoryShadow.current_wall_planes(n,c,11.3)),2)
        self.assertIsNone(n.pending_wall);self.assertEqual(n.wall_history.last_stamp,11.)
        self.assertEqual(message,saved)
        LiveMemoryShadow.on_wall(n,cloud(11.5));self.assertIsNotNone(n.pending_wall)

    def test_retry_expiry_does_not_refresh_or_clear_valid_history(self):
        n,c,b=self.fixture()
        for at in [10.9]:
            n.wall_history.add(n.wall_history.cloud_points(cloud(at),'odom'),at,at,.05,(0,0,0,0,0,0,1))
        # Force an eligible observation at 11.4 while latest TF is 10.9.
        n.pending_wall=cloud(11.4)
        LiveMemoryShadow.current_wall_planes(n,c,11.5)
        self.assertIsNotNone(n.pending_wall)
        frames=len(n.wall_history.frames)
        self.assertFalse(LiveMemoryShadow.current_wall_planes(n,c,12.2))
        self.assertIsNone(n.pending_wall);self.assertEqual(n.wall_history.last_stamp,10.9)
        self.assertEqual(len(n.wall_history.frames),frames)
        self.assertFalse(n.wall_history.planes(12.2))

    def test_original_freshness_limit_unchanged_after_tf_arrives_late(self):
        n,c,b=self.fixture();b.set_transform(transform(11.1),'isolated')
        self.assertFalse(LiveMemoryShadow.current_wall_planes(n,c,11.8))
        self.assertIsNone(n.pending_wall);self.assertEqual(n.wall_history.last_stamp,10.5)

    def test_real_association_change_still_revokes(self):
        for exact in [False,True]:
            n,c,b=self.fixture()
            b.set_transform(transform(11.1 if exact else 10.99,1.),'isolated')
            LiveMemoryShadow.current_wall_planes(n,c,11.2)
            self.assertIsNone(n.pending_wall);self.assertFalse(n.wall_history.frames)
            self.assertEqual(n.wall_reason,'MAP_ODOM_MOVED_SINCE_WALL')

    def test_missing_tf_still_revokes(self):
        n,c,b=self.fixture();b.clear()
        self.assertFalse(LiveMemoryShadow.current_wall_planes(n,c,11.1))
        self.assertFalse(n.wall_history.frames);self.assertIsNone(n.pending_wall)

    def test_past_extrapolation_still_revokes(self):
        n,c,b=self.fixture();b.clear();b.set_transform(transform(11.2),'isolated')
        self.assertFalse(LiveMemoryShadow.current_wall_planes(n,c,11.3))
        self.assertFalse(n.wall_history.frames);self.assertIsNone(n.pending_wall)

    def test_invalid_frame_and_future_observation_still_revoke(self):
        for kind in ['frame','future']:
            n,c,b=self.fixture();b.set_transform(transform(11.1),'isolated')
            if kind=='frame':n.pending_wall.header.frame_id='elsewhere'
            else:n.pending_wall=cloud(12.)
            LiveMemoryShadow.current_wall_planes(n,c,11.2)
            self.assertFalse(n.wall_history.frames);self.assertIsNone(n.pending_wall)

    def test_throttled_frame_is_consumed_without_fitting(self):
        n,c,b=self.fixture();n.pending_wall=cloud(10.7)
        self.assertEqual(len(LiveMemoryShadow.current_wall_planes(n,c,10.8)),2)
        self.assertIsNone(n.pending_wall);self.assertEqual(n.wall_history.last_stamp,10.5)

    def test_source_association_and_fit_not_called_until_exact_tf(self):
        n,c,b=self.fixture()
        with patch.object(n.wall_history,'add',wraps=n.wall_history.add) as add:
            LiveMemoryShadow.current_wall_planes(n,c,11.1);add.assert_not_called()
            b.set_transform(transform(11.1),'isolated')
            LiveMemoryShadow.current_wall_planes(n,c,11.3);self.assertEqual(add.call_count,1)

if __name__=='__main__':unittest.main()
