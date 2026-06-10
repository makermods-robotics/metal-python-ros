#!/usr/bin/env python3
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from metal_msg.msg import ArmEndPoseControl, ArmJointState


def _infer_gripper_from_position(node: Node, data_lines):
    """Return use_gripper (bool). None = invalid length."""
    first = data_lines[0]
    pos = first.get("position")
    if pos is None:
        node.get_logger().warn(
            'No "position" in first line; gripper_stroke not driven (arm-only).'
        )
        return False
    n = len(pos)
    if n == 6:
        node.get_logger().info(
            'First line "position" has 6 values; end_pose only, gripper_stroke ignored.'
        )
        return False
    if n == 7:
        node.get_logger().info(
            'First line "position" has 7 values; using position[6] as gripper_stroke each pose.'
        )
        return True
    node.get_logger().error(
        'First line "position" length is %d; only 6 (arm) or 7 (arm+gripper) are supported.'
        % n
    )
    return None


def _fill_end_pose_msg(msg: ArmEndPoseControl, row: dict, use_gripper: bool, node: Node, line_idx: int):
    msg.end_pose = row["end_pose"]
    msg.joint_velocity = 5
    if not use_gripper:
        msg.gripper_stroke = 0.0
        msg.gripper_velocity = 6
        return
    pos = row.get("position")
    if pos is None or len(pos) < 7:
        node.get_logger().error(
            "Line %d: expected 7 position values for gripper, got %s"
            % (line_idx, "missing" if pos is None else len(pos))
        )
        raise ValueError("invalid position for gripper")
    msg.gripper_stroke = float(pos[6])
    msg.gripper_velocity = 6


class EndPosePlayPosNode(Node):
    def __init__(self):
        super().__init__("end_pose_play_pos")
        self.declare_parameter("joint_state_topic", "/puppet_arm_right/joint_states")
        self.declare_parameter(
            "end_pose_control_topic", "/master_arm_right/end_pose_control"
        )
        self.declare_parameter("poses_jsonl", "data/recorded_pos.jsonl")
        self.declare_parameter("sleep_after_reach_sec", 2.0)
        self.declare_parameter("pose_tolerance", 0.3)

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._joint_state = None
        self.create_subscription(
            ArmJointState,
            self.get_parameter("joint_state_topic").value,
            self._joint_state_callback,
            qos,
        )
        self._pub = self.create_publisher(
            ArmEndPoseControl,
            self.get_parameter("end_pose_control_topic").value,
            qos,
        )

    def _joint_state_callback(self, msg: ArmJointState):
        self._joint_state = msg

    def wait_joint_state(self):
        while self._joint_state is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    node = EndPosePlayPosNode()

    jsonl_file = node.get_parameter("poses_jsonl").value
    sleep_time = float(node.get_parameter("sleep_after_reach_sec").value)
    tol = float(node.get_parameter("pose_tolerance").value)

    with open(jsonl_file, "r", encoding="utf-8") as f:
        data_lines = [json.loads(line) for line in f]

    if not data_lines:
        node.get_logger().error("%s file no data." % jsonl_file)
        node.destroy_node()
        rclpy.shutdown()
        return

    use_gripper = _infer_gripper_from_position(node, data_lines)
    if use_gripper is None:
        node.destroy_node()
        rclpy.shutdown()
        return

    time.sleep(3)
    node.wait_joint_state()
    if node._joint_state is None:
        node.get_logger().error("No joint state received")
        node.destroy_node()
        rclpy.shutdown()
        return

    msg = ArmEndPoseControl()
    msg.header.stamp = node.get_clock().now().to_msg()
    data_len = len(data_lines)
    node.get_logger().info("total pos number : %d" % data_len)
    input("input key [Enter] to start play position.")

    for i in range(data_len):
        row = data_lines[i]
        target_pose = row["end_pose"]
        try:
            _fill_end_pose_msg(msg, row, use_gripper, node, i)
        except ValueError:
            break

        js = node._joint_state
        node.get_logger().info(
            "play %dth pose, current end pose: %s, target end pose: %s"
            % (i + 1, list(js.end_pose), target_pose)
        )
        msg.header.stamp = node.get_clock().now().to_msg()
        node._pub.publish(msg)

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            js = node._joint_state
            if js is None:
                continue
            current_pose = list(js.end_pose)
            if len(current_pose) < 6:
                continue
            if all(
                abs(current_pose[j] - target_pose[j]) < tol for j in range(6)
            ):
                break

        time.sleep(sleep_time)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
