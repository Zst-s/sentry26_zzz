#!/usr/bin/env python3
"""Send a cruise event only after the cruise node is discovered, then confirm it."""

import argparse
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("event", choices=("find", "lose"))
    args = parser.parse_args()
    topic = "/find_already" if args.event == "find" else "/lose_already"
    expected = args.event == "find"
    rclpy.init()
    node = rclpy.create_node("send_cruise_event")
    command_pub = node.create_publisher(Bool, topic, 10)
    state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
    received = []
    state_sub = node.create_subscription(
        Bool, "/cruise/found_state", lambda msg: received.append(msg.data), state_qos)
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            subscribers = node.get_subscriptions_info_by_topic(topic)
            if any(s.node_name == "cruise_controller_node" for s in subscribers) and \
                    node.count_publishers("/cruise/found_state") > 0:
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        else:
            print(f"巡航节点未订阅 {topic} 或未发布确认话题；事件未发送", file=sys.stderr)
            return 1

        # Drain the latched startup state before sending; only a new callback is an ack.
        settle_until = time.monotonic() + 0.3
        while time.monotonic() < settle_until:
            rclpy.spin_once(node, timeout_sec=0.05)
        received.clear()

        deadline = time.monotonic() + 3.0
        next_send = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                command_pub.publish(Bool(data=True))
                next_send = now + 0.1
            rclpy.spin_once(node, timeout_sec=0.05)
            if expected in received:
                print(f"巡航节点已确认 {args.event} 事件")
                return 0
        print(f"已发送 {topic}，但巡航节点未确认收到", file=sys.stderr)
        return 1
    finally:
        node.destroy_subscription(state_sub)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
