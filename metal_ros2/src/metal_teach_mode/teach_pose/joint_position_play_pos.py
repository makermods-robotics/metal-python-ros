#!/usr/bin/env python3
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from metal_msg.msg import ArmJointPositionControl, ArmJointState


def _infer_gripper_from_position(node: Node, data_lines):
    """Return use_gripper (bool) or None if invalid."""
    pos = data_lines[0].get("position")
    if pos is None or len(pos) == 0:
        node.get_logger().error('First line has no "position" or it is empty.')
        return None
    n = len(pos)
    if n == 6:
        node.get_logger().info(
            'First line "position" has 6 values; arm joints only, gripper_stroke=0.'
        )
        return False
    if n == 7:
        node.get_logger().info(
            'First line "position" has 7 values; position[0:6] -> joint_position, '
            "position[6] -> gripper_stroke."
        )
        return True
    node.get_logger().error(
        'First line "position" length is %d; only 6 (arm) or 7 (arm+gripper) are supported.'
        % n
    )
    return None


def _fill_joint_position_msg(
    msg: ArmJointPositionControl, row: dict, use_gripper: bool, node: Node, line_idx: int
):
    pos = row.get("position")
    need = 7 if use_gripper else 6
    if pos is None or len(pos) < need:
        node.get_logger().error(
            "Line %d: need at least %d position values, got %s"
            % (line_idx, need, "missing" if pos is None else len(pos))
        )
        raise ValueError("invalid position")
    msg.joint_position = list(pos[:6])
    msg.joint_velocity = 6
    if use_gripper:
        msg.gripper_stroke = float(pos[6])
        msg.gripper_velocity = 6
    else:
        msg.gripper_stroke = 0.0
        msg.gripper_velocity = 6


class JointPositionPlayPosNode(Node):
    def __init__(self):
        super().__init__("joint_position_play_pos")
        self.declare_parameter("joint_state_topic", "/puppet_arm_right/joint_states")
        self.declare_parameter(
            "joint_position_control_topic", "/master_arm_right/joint_states"
        )
        self.declare_parameter("poses_jsonl", "data/recorded_pos.jsonl")
        self.declare_parameter("sleep_after_reach_sec", 2.0)
        self.declare_parameter("position_tolerance", 0.01)

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
            ArmJointPositionControl,
            self.get_parameter("joint_position_control_topic").value,
            qos,
        )

    def _joint_state_callback(self, msg: ArmJointState):
        self._joint_state = msg

    def wait_joint_state(self):
        while self._joint_state is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    node = JointPositionPlayPosNode()

    jsonl_file = node.get_parameter("poses_jsonl").value
    sleep_time = float(node.get_parameter("sleep_after_reach_sec").value)
    tol = float(node.get_parameter("position_tolerance").value)

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

    msg = ArmJointPositionControl()
    msg.header.stamp = node.get_clock().now().to_msg()
    data_len = len(data_lines)
    node.get_logger().info("total pos number : %d" % data_len)
    input("input key [Enter] to start play position.")

    for i in range(data_len):
        row = data_lines[i]
        try:
            _fill_joint_position_msg(msg, row, use_gripper, node, i)
        except ValueError:
            break

        target_arm = list(row["position"][:6])
        js = node._joint_state
        node.get_logger().info(
            "play %dth position, current position: %s, target arm joints: %s"
            % (i + 1, list(js.joint_position), target_arm)
        )
        msg.header.stamp = node.get_clock().now().to_msg()
        node._pub.publish(msg)

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            js = node._joint_state
            if js is None:
                continue
            current_pos = list(js.joint_position)
            if len(current_pos) < 6:
                continue
            if all(
                abs(current_pos[j] - target_arm[j]) < tol for j in range(6)
            ):
                node.get_logger().info("reach target position")
                break

        time.sleep(sleep_time)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
