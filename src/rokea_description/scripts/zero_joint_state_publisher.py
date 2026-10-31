#!/usr/bin/env python3
"""Publish zero joint states for all non-fixed joints in robot_description."""

import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ZeroJointStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__("zero_joint_state_publisher")
        self.declare_parameter("robot_description", "")
        desc = self.get_parameter("robot_description").get_parameter_value().string_value
        if not desc:
            raise RuntimeError("robot_description parameter is empty")
        root = ET.fromstring(desc)
        self._names = [
            j.get("name")
            for j in root.findall("joint")
            if j.get("type") in ("revolute", "continuous", "prismatic")
        ]
        self._pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_timer(0.05, self._tick)
        self.get_logger().info(f"Publishing zeros for {len(self._names)} joints")

    def _tick(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._names
        msg.position = [0.0] * len(self._names)
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ZeroJointStatePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
