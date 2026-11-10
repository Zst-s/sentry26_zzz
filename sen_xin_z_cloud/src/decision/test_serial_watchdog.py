#!/usr/bin/env python3
"""Test trans_node2 through a PTY; never opens a physical serial device."""
import os
import pty
import select
import signal
import struct
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rm_decision_interfaces.msg import RobotStatus
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '231' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Require isolated ROS domain 231')
    master, slave = pty.openpty()
    path = os.ttyname(slave)
    if not path.startswith('/dev/pts/'):
        raise RuntimeError('PTY only')
    os.set_blocking(master, False)
    rclpy.init()
    node = rclpy.create_node('serial_watchdog_test')
    publisher = node.create_publisher(Twist, '/serial_watchdog_test/command', 1)
    ready = node.create_publisher(UInt8, '/serial_watchdog_test/ready', 1)
    passage = node.create_publisher(UInt8, '/serial_watchdog_test/passage',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    proc = subprocess.Popen([sys.argv[1], '--ros-args',
        '-p', 'serial_port:='+path, '-p', 'command_timeout:=0.25',
        '-p', 'serial_rx_log_every:=100000',
        '-r', '/robot_status:=/serial_watchdog_test/robot_status',
        '-r', '/passage/serial_tx_status:=/serial_watchdog_test/tx_status',
        '-r', '/cmd_vel:=/serial_watchdog_test/command',
        '-r', '/nav_ready:=/serial_watchdog_test/ready',
        '-r', '/if_guodong:=/serial_watchdog_test/passage',
        '-r', '/Odometry:=/serial_watchdog_test/odom',
        '-p', 'require_odometry:='+str(len(sys.argv)>2).lower()])
    require_odometry = len(sys.argv)>2 and sys.argv[2]=="--require-odometry"
    publish_odometry = not require_odometry
    odometry = node.create_publisher(Odometry, "/serial_watchdog_test/odom", 1)
    buffer = bytearray()
    frames = []
    passage_values = []
    received_status = []
    tx_status = []
    robot_sub = node.create_subscription(RobotStatus, '/serial_watchdog_test/robot_status',
        received_status.append, 10)
    diagnostic_sub = node.create_subscription(DiagnosticArray, '/serial_watchdog_test/tx_status',
        tx_status.append, qos_profile_sensor_data)
    # Emulated MCU feedback. Existing reverse protocol is a separate 30-byte frame.
    reply = bytearray(30)
    reply[0] = 0xA5
    reply[3:5] = bytes([0,100])
    reply[7] = 3
    reply[9] = 4


    def collect(duration, command=None, ready_value=None):
        start = time.monotonic()
        next_publish = start
        next_feedback = start
        while time.monotonic()-start < duration:
            if frames and time.monotonic() >= next_feedback:
                try:
                    assert os.write(master, reply) == len(reply)
                except BlockingIOError:
                    pass
                next_feedback = time.monotonic()+.01
            if command is not None and time.monotonic() >= next_publish:
                publisher.publish(command)
                if publish_odometry:
                    pose = Odometry()
                    pose.header.frame_id = 'odom'
                    pose.header.stamp = node.get_clock().now().to_msg()
                    pose.pose.pose.orientation.w = 1.
                    odometry.publish(pose)
                if ready_value is not None:
                    ready.publish(UInt8(data=ready_value))
                next_publish = time.monotonic()+.04
            if select.select([master], [], [], .01)[0]:
                buffer.extend(os.read(master, 4096))
            while len(buffer) >= 148:
                frame = bytes(buffer[:148])
                del buffer[:148]
                assert frame[0] == 0xA5 and frame[1] == 148 and frame[14] == 3 and frame[16] == 0 and frame[18:] == bytes(130), "packed frame layout or unused tail mismatch"
                frames.append((time.monotonic(), struct.unpack('<fff', frame[2:14])))
                passage_values.append(frame[17])
            rclpy.spin_once(node, timeout_sec=0)
            if proc.poll() is not None:
                raise AssertionError('serial process exited')

    try:
        collect(1.)
        assert frames and all(all(v == 0 for v in f[1]) for f in frames), 'startup nonzero'
        assert passage_values[-1] == 1, 'startup must request raised head (1)'
        command = Twist()
        command.linear.x, command.linear.y, command.angular.z = .12, -.08, .1
        if require_odometry:
            begin = len(frames)
            collect(.4, command)
            assert all(all(v==0 for v in f[1]) for f in frames[begin:]), 'command passed without odometry'
            publish_odometry = True
        begin = len(frames)
        passage.publish(UInt8(data=0))  # New protocol: fold.
        collect(.6, command)
        assert passage_values[-1] == 0, 'fold request must be byte 17 = 0'
        passage.publish(UInt8(data=1))  # New protocol: raise.
        collect(.3, command)
        assert passage_values[-1] == 1, 'raise request must be byte 17 = 1'
        assert sum(f[1][0] > .1 for f in frames[begin:]) >= 10, 'output blocked by empty RX'
        if require_odometry:
            publish_odometry = False
            stopped_at = time.monotonic()
            collect(.6, command)
            assert all(all(v==0 for v in values) for stamp,values in frames
                       if stamp>stopped_at+.35), 'fresh Nav2 commands passed after odometry stopped'
            publish_odometry = True
            collect(.3, command)
            assert frames[-1][1][0]>.1, 'fresh odometry and command did not recover'
        stopped_at = time.monotonic()
        collect(.7)
        assert all(all(v == 0 for v in values) for stamp, values in frames
                   if stamp > stopped_at+.35), 'stale velocity repeated beyond watchdog'
        collect(.3, command)
        assert frames[-1][1][0] > .1, 'fresh command did not recover'
        collect(.3, command, ready_value=0)
        assert all(v == 0 for v in frames[-1][1]), 'nav_ready=0 did not stop'
        command.linear.x = float('nan')
        collect(.3, command, ready_value=1)
        assert all(v == 0 for v in frames[-1][1]), 'nonfinite command was sent'
        assert received_status and received_status[-1].current_hp == 100, 'MCU feedback not consumed/decoded'
        assert tx_status, 'missing formal serial diagnostics'
        fields = {kv.key: kv.value for kv in tx_status[-1].status[0].values}
        assert fields['requested_bytes'] == '148' and fields['written_bytes'] == '148'
        assert fields['if_guodong_offset'] == '17' and fields['frame_header'] == '0xA5'
        assert int(fields['rx_bytes_total']) >= 300, 'feedback not drained continuously'
        print('PASS actual serial frames: startup zero, idle RX nonblocking, 250ms watchdog, ready veto, NaN rejection, default raised=1, fold=0/raise=1 byte17, continuous MCU feedback decoding and diagnostics')
    finally:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        os.close(master)
        os.close(slave)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
