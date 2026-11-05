#!/usr/bin/env python3
"""Simulation-only cloud frame adapter and expiring planar-drive transport."""
import math
import os
import time
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from navigation_assets import LIDAR_XYZ, LIDAR_RPY


class SimSupport(Node):
    def __init__(self):
        super().__init__('passage_navigation_support')
        if (os.environ.get('ROS_DOMAIN_ID') != '229' or
                os.environ.get('ROS_LOCALHOST_ONLY') != '1' or
                not self.get_parameter('use_sim_time').value):
            raise RuntimeError('Requires simulated clock, domain 229 and localhost only')
        self.command = Twist()
        self.command_time = self.odom_time = -math.inf
        self.output = self.create_publisher(Twist, '/passage_sim/drive_cmd_vel', 1)
        self.cloud = self.create_publisher(PointCloud2, '/passage_sim/body_points', 1)
        self.create_subscription(Twist, '/passage_sim/smoothed_cmd_vel', self.on_command, 1)
        self.create_subscription(Odometry, '/passage_sim/base_odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/passage_sim/points', self.on_cloud, qos_profile_sensor_data)
        self.create_timer(.02, self.tick)
        self.mount = TransformStamped()
        self.mount.header.frame_id = 'base_link'
        self.mount.child_frame_id = 'passage_lidar'
        t = self.mount.transform.translation
        t.x, t.y, t.z = LIDAR_XYZ
        r, p, y = (v/2 for v in LIDAR_RPY)
        q = self.mount.transform.rotation
        q.x = math.sin(r)*math.cos(p)*math.cos(y)-math.cos(r)*math.sin(p)*math.sin(y)
        q.y = math.cos(r)*math.sin(p)*math.cos(y)+math.sin(r)*math.cos(p)*math.sin(y)
        q.z = math.cos(r)*math.cos(p)*math.sin(y)-math.sin(r)*math.sin(p)*math.cos(y)
        q.w = math.cos(r)*math.cos(p)*math.cos(y)+math.sin(r)*math.sin(p)*math.sin(y)

    def on_command(self, msg):
        if not all(math.isfinite(v) for v in (msg.linear.x, msg.linear.y, msg.angular.z)):
            self.command_time = -math.inf
            return
        self.command = msg
        self.command_time = time.monotonic()

    def on_odom(self, msg):
        if msg.header.frame_id == 'world' and msg.child_frame_id == 'base_link':
            self.odom_time = time.monotonic()

    def on_cloud(self, msg):
        if msg.header.frame_id != 'passage_lidar':
            return
        self.mount.header.stamp = msg.header.stamp
        self.cloud.publish(do_transform_cloud(msg, self.mount))

    def tick(self):
        now = time.monotonic()
        healthy = now-self.command_time < .25 and now-self.odom_time < .5
        self.output.publish(self.command if healthy else Twist())


def main():
    rclpy.init()
    node = SimSupport()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.output.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
