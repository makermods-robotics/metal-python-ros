#!/usr/bin/env python3
"""Bridge: metal_arm_msgs/ArmJointState  ->  sensor_msgs/JointState.

The driver publishes the real arm's state as a custom ArmJointState (no joint
names). RViz / robot_state_publisher need sensor_msgs/JointState on /joint_states
with names matching the DESCRIPTION urdf (joint1..joint6, lowercase). This node
maps the SDK joint order (J1..J6, base->tip) to those names so the RViz model
tracks the real arm.
"""
import rclpy
from rclpy.node import Node

from metal_arm_msgs.msg import ArmJointState
from sensor_msgs.msg import JointState

# description-package urdf joint names, base->tip. Must match that urdf.
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


class ArmStateBridge(Node):
    def __init__(self):
        super().__init__("arm_state_bridge")
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.sub = self.create_subscription(
            ArmJointState, "/metal/arm_joint_state", self.cb, 10)
        self.get_logger().info(
            "bridging /metal/arm_joint_state -> /joint_states (%s)" % JOINT_NAMES)

    def cb(self, msg):
        n = min(len(JOINT_NAMES), len(msg.joint_position))
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINT_NAMES[:n]
        js.position = [float(x) for x in msg.joint_position[:n]]
        if len(msg.joint_velocity) >= n:
            js.velocity = [float(x) for x in msg.joint_velocity[:n]]
        if len(msg.joint_effort) >= n:
            js.effort = [float(x) for x in msg.joint_effort[:n]]
        self.pub.publish(js)


def main():
    rclpy.init()
    node = ArmStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
