#!/usr/bin/env python3
"""Real DDS delivery from the ROGMap visualizer's BEST_EFFORT QoS."""
import tempfile,time,unittest
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from live_memory_shadow import LiveMemoryShadow

class WallQosDelivery(unittest.TestCase):
    def test_best_effort_wall_cloud_reaches_runtime(self):
        with tempfile.TemporaryDirectory() as folder:
            rclpy.init(args=['--ros-args','-r','__ns:=/passage','-p',
                'map_session_id:=mapping-qos-regression','-p','record_path:='+folder+'/memory.json',
                '-p','wall_bounded_correction:=true'])
            node=LiveMemoryShadow(runtime=True)
            try:
                pub=node.create_publisher(PointCloud2,'/rog_map/raw_occupied',
                    QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
                self.assertEqual(node.wall_sub.qos_profile.reliability,ReliabilityPolicy.BEST_EFFORT)
                deadline=time.monotonic()+5.
                while time.monotonic()<deadline and node.pending_wall is None:
                    header=Header(frame_id='odom');header.stamp=node.get_clock().now().to_msg()
                    pub.publish(create_cloud_xyz32(header,[[1.234,2.345,.4]]))
                    rclpy.spin_once(node,timeout_sec=.02)
                self.assertIsNotNone(node.pending_wall)
                self.assertEqual(node.pending_wall.width,1)
                self.assertGreaterEqual(pub.get_subscription_count(),1)
            finally:
                node.destroy_node();rclpy.shutdown()

if __name__=='__main__':unittest.main(verbosity=2)
