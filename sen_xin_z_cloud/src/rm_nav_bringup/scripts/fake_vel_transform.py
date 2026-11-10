#!/usr/bin/env python3
"""固定 odom 朝向的虚拟导航系，以及从虚拟系到原速度系的转换。"""
import math
import copy
import numpy as np
import rclpy
from scipy.spatial.transform import Rotation
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException


def rotate_velocity(x, y, yaw):
    return x * math.cos(yaw) + y * math.sin(yaw), -x * math.sin(yaw) + y * math.cos(yaw)


def fake_odometry(message, fake_frame='base_link_fake', reference_frame='odom'):
    """把车体系实测速度转到与 odom 同朝向的虚拟车体系。"""
    if message.header.frame_id != reference_frame or message.child_frame_id != 'base_link':
        raise ValueError('Expected odom -> base_link odometry')
    q = message.pose.pose.orientation
    quaternion = np.array([q.x, q.y, q.z, q.w], dtype=float)
    if not np.isfinite(quaternion).all() or abs(np.linalg.norm(quaternion) - 1.0) > 0.01:
        raise ValueError('Invalid base_link odometry rotation')
    rotation = Rotation.from_quat(quaternion).as_matrix()
    result = copy.deepcopy(message)
    result.child_frame_id = fake_frame
    result.pose.pose.orientation.x = 0.0
    result.pose.pose.orientation.y = 0.0
    result.pose.pose.orientation.z = 0.0
    result.pose.pose.orientation.w = 1.0
    source = message.twist.twist
    linear = rotation @ np.array([source.linear.x, source.linear.y, source.linear.z])
    angular = rotation @ np.array([source.angular.x, source.angular.y, source.angular.z])
    target = result.twist.twist
    target.linear.x, target.linear.y, target.linear.z = map(float, linear)
    target.angular.x, target.angular.y, target.angular.z = map(float, angular)
    transform = np.zeros((6, 6))
    transform[:3, :3] = rotation
    transform[3:, 3:] = rotation
    covariance = np.asarray(message.twist.covariance, dtype=float).reshape(6, 6)
    result.twist.covariance = (transform @ covariance @ transform.T).ravel().tolist()
    return result


class FakeVelocityTransform(Node):
    def __init__(self):
        super().__init__('fake_vel_transform')
        defaults = dict(reference_frame='odom', robot_base_frame='base_link',
                        fake_robot_base_frame='base_link_fake', input_cmd_vel_topic='/cmd_vel_fake',
                        output_cmd_vel_topic='/cmd_vel', init_spin_speed=0.0,
                        cmd_spin_topic='/cmd_spin', input_odom_topic='/base_imu',
                        output_odom_topic='/base_imu_fake', timeout=0.5)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        if self.p['input_cmd_vel_topic'] == self.p['output_cmd_vel_topic']:
            raise ValueError('Input and output velocity topics must differ')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.pub = self.create_publisher(Twist, self.p['output_cmd_vel_topic'], 10)
        self.odom_pub = self.create_publisher(Odometry, self.p['output_odom_topic'], 10)
        self.create_subscription(Twist, self.p['input_cmd_vel_topic'], self.command, 10)
        self.create_subscription(Odometry, self.p['input_odom_topic'], self.odometry, 10)
        self.create_subscription(Float32, self.p['cmd_spin_topic'], self.spin, 10)
        self.last_command = None
        self.stopped = False
        self.spin_speed = self.p['init_spin_speed']
        self.create_timer(0.02, self.tick)

    def spin(self, msg):
        if math.isfinite(msg.data):
            self.spin_speed = msg.data

    def odometry(self, msg):
        try:
            self.odom_pub.publish(fake_odometry(
                msg, self.p['fake_robot_base_frame'], self.p['reference_frame']))
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=2.0)

    def lookup(self):
        t = self.buffer.lookup_transform(self.p['reference_frame'], self.p['robot_base_frame'], Time())
        age = (self.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds / 1e9
        if age > self.p['timeout'] or age < -0.1:
            raise ValueError(
                f"Robot TF is stale: age={age:.3f}s, limit={self.p['timeout']:.3f}s, "
                f"frame={self.p['reference_frame']}->{self.p['robot_base_frame']}"
            )
        q = t.transform.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        return t, yaw

    def command(self, msg):
        self.last_command = self.get_clock().now()
        self.stopped = False
        try:
            _, yaw = self.lookup()
            out = Twist()
            out.linear.x, out.linear.y = rotate_velocity(msg.linear.x, msg.linear.y, yaw)
            # 停止指令必须能停止，不能给全零指令继续叠加旋转。
            if any(abs(v) > 1e-8 for v in (msg.linear.x, msg.linear.y, msg.angular.z)):
                out.angular.z = msg.angular.z + self.spin_speed
            self.pub.publish(out)
        except (TransformException, ValueError):
            self.pub.publish(Twist())

    def tick(self):
        try:
            source, _ = self.lookup()
            t = TransformStamped()
            # 直接挂在 odom 下，避免和原 base_link 的父子链产生环。
            t.header = source.header
            t.child_frame_id = self.p['fake_robot_base_frame']
            t.transform.translation = source.transform.translation
            t.transform.rotation.w = 1.0
            self.broadcaster.sendTransform(t)
        except (TransformException, ValueError) as exe:
            self.get_logger().warning(
                str(exe),
                throttle_duration_sec=2.0
            )
        if self.last_command is not None and not self.stopped:
            age = (self.get_clock().now()-self.last_command).nanoseconds / 1e9
            if age > self.p['timeout']:
                self.pub.publish(Twist())
                self.stopped = True


def main():
    rclpy.init()
    node = FakeVelocityTransform()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
