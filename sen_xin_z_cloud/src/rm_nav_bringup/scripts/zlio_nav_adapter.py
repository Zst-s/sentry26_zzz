#!/usr/bin/env python3
"""把 ZLIO 的 IMU 原点里程计转换为车体里程计和水平车体 TF。"""

import copy

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def body_odometry(message, translation, rotation):
    """translation/rotation 表示 T_imu_base；速度补偿包含旋转造成的杆臂项。"""
    result = copy.deepcopy(message)
    result.child_frame_id = 'base_link'
    pose = message.pose.pose
    q = pose.orientation
    world_from_imu = Rotation.from_quat([q.x, q.y, q.z, q.w])
    lever_world = world_from_imu.apply(translation)
    position = np.array([pose.position.x, pose.position.y, pose.position.z]) + lever_world
    out_pose = result.pose.pose
    out_pose.position.x, out_pose.position.y, out_pose.position.z = map(float, position)
    quaternion = (world_from_imu * rotation).as_quat()
    (out_pose.orientation.x, out_pose.orientation.y,
     out_pose.orientation.z, out_pose.orientation.w) = map(float, quaternion)
    twist = message.twist.twist
    velocity = np.array([twist.linear.x, twist.linear.y, twist.linear.z])
    omega = np.array([twist.angular.x, twist.angular.y, twist.angular.z])
    base_from_imu = rotation.inv().as_matrix()
    linear = base_from_imu @ (velocity + np.cross(omega, translation))
    angular = base_from_imu @ omega
    out_twist = result.twist.twist
    out_twist.linear.x, out_twist.linear.y, out_twist.linear.z = map(float, linear)
    out_twist.angular.x, out_twist.angular.y, out_twist.angular.z = map(float, angular)
    pose_jacobian = np.eye(6)
    pose_jacobian[:3, 3:] = -skew(lever_world)
    twist_jacobian = np.zeros((6, 6))
    twist_jacobian[:3, :3] = base_from_imu
    twist_jacobian[3:, 3:] = base_from_imu
    twist_jacobian[:3, 3:] = -base_from_imu @ skew(translation)
    for field, jacobian in ((result.pose, pose_jacobian), (result.twist, twist_jacobian)):
        covariance = np.asarray(field.covariance).reshape(6, 6)
        field.covariance = (jacobian @ covariance @ jacobian.T).ravel().tolist()
    return result


class ZlioNavAdapter(Node):
    def __init__(self):
        super().__init__('zlio_nav_adapter')
        self.declare_parameter('imu_from_base_translation', [0., 0., 0.])
        self.declare_parameter('imu_from_base_quaternion', [0., 0., 0., 1.])
        self.translation = np.asarray(self.get_parameter('imu_from_base_translation').value)
        self.rotation = Rotation.from_quat(self.get_parameter('imu_from_base_quaternion').value)
        self.odom_pub = self.create_publisher(Odometry, '/base_imu', 20)
        self.footprint_tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/Odometry', self.odometry, 20)

    def odometry(self, message):
        if message.header.frame_id != 'odom' or message.child_frame_id != 'imu':
            self.get_logger().error('Expected ZLIO odom -> imu odometry', throttle_duration_sec=5.)
            return
        body = body_odometry(message, self.translation, self.rotation)
        self.odom_pub.publish(body)
        yaw = Rotation.from_quat([body.pose.pose.orientation.x,
                                  body.pose.pose.orientation.y,
                                  body.pose.pose.orientation.z,
                                  body.pose.pose.orientation.w]).as_euler('xyz')[2]
        tf = TransformStamped()
        tf.header = body.header
        tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x = body.pose.pose.position.x
        tf.transform.translation.y = body.pose.pose.position.y
        q = Rotation.from_euler('z', yaw).as_quat()
        tf.transform.rotation.x, tf.transform.rotation.y = float(q[0]), float(q[1])
        tf.transform.rotation.z, tf.transform.rotation.w = float(q[2]), float(q[3])
        self.footprint_tf.sendTransform(tf)

def main(args=None):
    rclpy.init(args=args)
    node = ZlioNavAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
