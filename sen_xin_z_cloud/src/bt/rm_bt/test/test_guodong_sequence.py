#!/usr/bin/env python3
"""Exercise the real tree/client against a mock action server in an isolated DDS domain."""
import os
os.environ['ROS_DOMAIN_ID'] = '221'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import signal
import subprocess
import sys
import threading
import time
import unittest
from pathlib import Path

import rclpy
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import UInt8

EXECUTABLE = str(Path(sys.argv.pop(1)).resolve())


class PassageSequenceTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('mock_passage_navigation')
        self.requests = []
        self.limits = []
        self.goal_limits = []
        self.head = []
        self.cancelled = []
        self.release = [threading.Event() for _ in range(3)]
        self.abort_goal = 0
        self.reject = False
        self.accept_delay = 0
        self.feedback_period = None
        self.process = None
        self.output = None
        self.server = ActionServer(
            self.node, NavigateToPose, 'navigate_to_pose', self.execute,
            goal_callback=self.goal, cancel_callback=self.cancel,
            callback_group=ReentrantCallbackGroup())
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.subscription = self.node.create_subscription(
            UInt8, '/if_guodong', lambda m: self.head.append((m.data, time.monotonic())), qos)
        self.speed_subscription = self.node.create_subscription(
            SpeedLimit, '/speed_limit',
            lambda m: self.limits.append((m.speed_limit, m.percentage)), 10)
        self.executor = MultiThreadedExecutor(num_threads=6)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def goal(self, request):
        self.goal_limits.append(self.limits[-1] if self.limits else None)
        self.requests.append((request.pose.pose.position.x, request.pose.pose.position.y,
                              time.monotonic()))
        time.sleep(self.accept_delay)
        return GoalResponse.REJECT if self.reject else GoalResponse.ACCEPT

    def cancel(self, handle):
        self.cancelled.append(bytes(handle.goal_id.uuid))
        return CancelResponse.ACCEPT

    def execute(self, handle):
        index = len(self.requests) - 1
        deadline = time.monotonic() + 25
        while not self.release[index].is_set() and time.monotonic() < deadline:
            if handle.is_cancel_requested:
                handle.canceled()
                return NavigateToPose.Result()
            if self.feedback_period is not None:
                handle.publish_feedback(NavigateToPose.Feedback())
            time.sleep(self.feedback_period or .01)
        if handle.is_cancel_requested:
            handle.canceled()
        elif self.abort_goal == index + 1:
            handle.abort()
        else:
            handle.succeed()
        return NavigateToPose.Result()

    def start(self, *, stop=True, timeout=1500, missing=False, tick_ms=20):
        command = [EXECUTABLE, '--ros-args', '-p', 'tree_xml_file:=behavior_tree/guodong.xml',
                   '-p', 'enable_groot:=false', '-p', f'tick_period_milliseconds:={tick_ms}',
                   '-p', 'fold_wait_ms:=300', '-p', f'stop_on_completion:={str(stop).lower()}',
                   '-p', f'send_goal_timeout_ms:={timeout}']
        if missing:
            self.server.destroy()
            self.server = None
        import tempfile
        self.output = tempfile.TemporaryFile(mode='w+')
        self.process = subprocess.Popen(command, stdout=self.output, stderr=subprocess.STDOUT)

    def wait_for(self, predicate, timeout=8):
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            if predicate():
                return
            time.sleep(.02)
        self.fail('Timed out waiting for mock navigation event\n' + self.process_output())

    def process_output(self):
        self.output.seek(0)
        return self.output.read()

    def finish(self, expected):
        try:
            code = self.process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            self.fail('Decision failed to exit\n' + self.process_output())
        self.assertEqual(code, expected, self.process_output())
        time.sleep(.15)

    def test_arrival_order_wait_and_one_shot(self):
        self.start(stop=False)
        self.wait_for(lambda: len(self.requests) == 1)
        time.sleep(7)  # Crossing the old six-second threshold must not change stage.
        self.assertEqual(len(self.requests), 1)
        self.assertEqual(self.head, [])
        self.release[0].set()
        self.wait_for(lambda: len(self.requests) == 2)
        self.assertEqual(self.head, [])
        self.release[1].set()
        self.wait_for(lambda: len(self.requests) == 3)
        self.assertEqual([v for v, _ in self.head], [0])
        self.assertGreaterEqual(self.requests[2][2] - self.head[0][1], .27)
        self.release[2].set()
        self.wait_for(lambda: [v for v, _ in self.head] == [0, 1])
        time.sleep(.6)
        self.assertEqual([(x, y) for x, y, _ in self.requests],
                         [(-0.27, 1.48), (2.13, 2.01), (4.05, 2.06)])
        self.assertEqual([v for v, _ in self.head], [0, 1])
        self.assertEqual(self.goal_limits, [(0.5, False), (0.5, False), (0.0, False)])
        self.process.send_signal(signal.SIGINT)
        self.finish(0)

    def test_fast_feedback_does_not_starve_goal_response(self):
        self.feedback_period = .001
        self.start(timeout=600, tick_ms=50)
        self.wait_for(lambda: len(self.requests) == 1)
        time.sleep(1.2)  # A real response timeout still exists; acceptance must be processed first.
        self.assertNotIn('goal response timeout', self.process_output())
        self.assertEqual(self.process.poll(), None)
        for event in self.release:
            event.set()
        self.finish(0)
        self.assertEqual(len(self.requests), 3)
        self.assertEqual(self.cancelled, [])

    def test_success_exits_and_delivers_final_raise(self):
        for event in self.release:
            event.set()
        self.start()
        self.finish(0)
        self.assertEqual(len(self.requests), 3)
        self.assertEqual([v for v, _ in self.head], [0, 1])

    def test_first_goal_failure_never_folds(self):
        self.abort_goal = 1
        self.release[0].set()
        self.start()
        self.finish(1)
        self.assertEqual(len(self.requests), 1)
        self.assertEqual(self.head, [])

    def test_crossing_failure_does_not_raise(self):
        self.abort_goal = 3
        for event in self.release:
            event.set()
        self.start()
        self.finish(1)
        self.assertEqual(len(self.requests), 3)
        self.assertEqual([v for v, _ in self.head], [0])

    def test_rejected_goal_never_folds(self):
        self.reject = True
        self.start()
        self.finish(1)
        self.assertEqual(len(self.requests), 1)
        self.assertEqual(self.head, [])

    def test_missing_speed_subscriber_does_not_start_goal(self):
        self.node.destroy_subscription(self.speed_subscription)
        self.start()
        self.finish(1)
        self.assertEqual(self.requests, [])
        self.assertEqual(self.head, [])

    def test_missing_server_never_folds(self):
        self.start(missing=True)
        self.finish(1)
        self.assertEqual(self.requests, [])
        self.assertEqual(self.head, [])

    def test_interrupt_cancels_active_goal(self):
        self.start()
        self.wait_for(lambda: len(self.requests) == 1)
        time.sleep(.3)
        self.process.send_signal(signal.SIGINT)
        self.wait_for(lambda: len(self.cancelled) == 1)
        self.finish(0)
        self.assertEqual(self.head, [])

    def test_late_acceptance_after_timeout_is_cancelled(self):
        self.accept_delay = .7
        self.start(timeout=350)
        self.wait_for(lambda: len(self.cancelled) == 1)
        self.finish(1)
        self.assertEqual(self.head, [])

    def tearDown(self):
        if self.process is not None and self.process.poll() is None:
            self.process.send_signal(signal.SIGTERM)
        for event in self.release:
            event.set()
        if self.process is not None:
            try:
                self.process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.executor.shutdown(timeout_sec=3)
        self.thread.join(timeout=3)
        if self.server is not None:
            self.server.destroy()
        self.node.destroy_node()
        rclpy.shutdown()
        if self.output is not None:
            self.output.seek(0)
            output = self.output.read()
            # Keep failures diagnosable without flooding normal test output.
            result = self._outcome.result
            if any(test is self for test, _ in result.failures + result.errors):
                print(output)
            self.output.close()


if __name__ == '__main__':
    unittest.main(verbosity=2)
