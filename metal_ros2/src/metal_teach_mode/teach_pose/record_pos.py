#!/usr/bin/env python3
import json
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from metal_msg.msg import ArmJointState


class RecordPoseNode(Node):
    def __init__(self):
        super().__init__("record_pose")
        self.declare_parameter("joint_state_topic", "/master_arm_right/joint_states")
        self.declare_parameter("output_jsonl", "data/recorded_pos.jsonl")

        topic = self.get_parameter("joint_state_topic").value
        self._output = self.get_parameter("output_jsonl").value

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._joint_state = None
        self.create_subscription(ArmJointState, topic, self._joint_state_callback, qos)

    def _joint_state_callback(self, msg: ArmJointState):
        self._joint_state = msg

    def spin_short(self):
        rclpy.spin_once(self, timeout_sec=0.05)


def main(args=None):
    rclpy.init(args=args)
    node = RecordPoseNode()
    os.makedirs("data", exist_ok=True)

    count = 1
    with open(node._output, "a", encoding="utf-8") as f:
        while rclpy.ok():
            key = input(
                "Input key [Enter] record pose, key [q] to stop recording: "
            )
            if key == "q":
                break
            node.spin_short()
            if node._joint_state is not None:
                js = node._joint_state
                data = {
                    "position": list(js.joint_position),
                    "velocity": list(js.joint_velocity),
                    "effort": list(js.joint_effort),
                    "end_pose": list(js.end_pose),
                }
                f.write(json.dumps(data) + "\n")
                f.flush()
                node.get_logger().info(
                    f"record {count}th pose, position: {data['position']} , end_pose: {data['end_pose']}"
                )
                count += 1
            else:
                node.get_logger().warn("No joint state received yet")

    node.get_logger().info("record finish")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
