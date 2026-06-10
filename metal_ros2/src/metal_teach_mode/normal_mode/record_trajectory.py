#!/usr/bin/env python3
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from metal_msg.msg import ArmJointState


class RecordTrajectoryNode(Node):
    def __init__(self):
        super().__init__("record_trajectory")
        self.declare_parameter("joint_state_topic", "/master_arm_right/joint_states")
        self.declare_parameter("output_jsonl", "data/arm_state_200hz.jsonl")
        self.declare_parameter("record_rate_hz", 200.0)

        topic = self.get_parameter("joint_state_topic").value
        self._output = self.get_parameter("output_jsonl").value
        self._period = 1.0 / float(self.get_parameter("record_rate_hz").value)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._joint_state = None
        self.create_subscription(ArmJointState, topic, self._state_callback, qos)

    def _state_callback(self, msg: ArmJointState):
        self._joint_state = msg

    def wait_first_state(self):
        while self._joint_state is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    @property
    def joint_state(self):
        return self._joint_state

    def spin_period(self):
        rclpy.spin_once(self, timeout_sec=0.0)


def main(args=None):
    rclpy.init(args=args)
    node = RecordTrajectoryNode()
    os.makedirs("data", exist_ok=True)

    input("press key [Enter] to start record trajectory.")
    node.wait_first_state()
    if node.joint_state is None:
        node.get_logger().error("No joint state received")
        node.destroy_node()
        rclpy.shutdown()
        return

    with open(node._output, "a", encoding="utf-8") as f:
        count = 1
        while rclpy.ok():
            js = node.joint_state
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
            node.spin_period()
            try:
                time.sleep(node._period)
            except KeyboardInterrupt:
                break

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
