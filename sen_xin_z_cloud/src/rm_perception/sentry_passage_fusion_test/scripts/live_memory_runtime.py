#!/usr/bin/env python3
"""Run the real structure-memory mask producer without the legacy motion chain."""
import rclpy
from rclpy.executors import ExternalShutdownException
from live_memory_shadow import LiveMemoryShadow


def main():
    rclpy.init()
    node = LiveMemoryShadow(runtime=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
