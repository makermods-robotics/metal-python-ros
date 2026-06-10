#!/usr/bin/env python3
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from metal_msg.msg import ArmJointPositionControl


def _infer_gripper_from_position(node: Node, data_lines):
    """Return use_gripper (bool) or None if invalid. position: 6=arm only, 7=arm+gripper."""
    pos = data_lines[0].get("position")
    if pos is None or len(pos) == 0:
        node.get_logger().error('First line has no "position" or it is empty.')
        return None
    n = len(pos)
    if n == 6:
        node.get_logger().info(
            'First line "position" has 6 values; joint_position only, gripper_stroke=0.'
        )
        return False
    if n == 7:
        node.get_logger().info(
            'First line "position" has 7 values; position[0:6] -> joint_position, '
            "position[6] -> gripper_stroke each frame."
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
    msg.joint_velocity = 3
    if use_gripper:
        msg.gripper_stroke = float(pos[6])
        msg.gripper_velocity = 6
    else:
        msg.gripper_stroke = 0.0
        msg.gripper_velocity = 6


def playback_trajectory(node: Node, jsonl_file: str, control_pub, rate_hz: float):
    with open(jsonl_file, "r", encoding="utf-8") as f:
        data_lines = [json.loads(line) for line in f]

    if not data_lines:
        node.get_logger().error("No data found in the file.")
        return

    use_gripper = _infer_gripper_from_position(node, data_lines)
    if use_gripper is None:
        return

    data_len = len(data_lines)
    node.get_logger().info("trajectory size : %d" % data_len)
    idx = 1
    msg = ArmJointPositionControl()
    msg.header.stamp = node.get_clock().now().to_msg()
    try:
        _fill_joint_position_msg(msg, data_lines[0], use_gripper, node, 0)
    except ValueError:
        return
    input("Press key [Enter] to start play trajectory.")

    time.sleep(3)

    control_pub.publish(msg)
    node.get_logger().info(
        "Publish start position, sleeping for 3 seconds go to start position."
    )
    time.sleep(3)
    node.get_logger().info("Playback started.")

    period = 1.0 / rate_hz
    while rclpy.ok() and idx < data_len:
        msg.header.stamp = node.get_clock().now().to_msg()
        try:
            _fill_joint_position_msg(msg, data_lines[idx], use_gripper, node, idx)
        except ValueError:
            break
        control_pub.publish(msg)
        idx += 1
        node.get_logger().info("index: %d" % idx)
        time.sleep(period)


def main(args=None):
    rclpy.init(args=args)
    node = Node("play_trajectory_joint_position")
    node.declare_parameter(
        "trajectory_jsonl", "data/arm_state_200hz.jsonl"
    )
    node.declare_parameter(
        "joint_position_control_topic", "/master_arm_right/joint_states"
    )
    node.declare_parameter("playback_rate_hz", 200.0)

    jsonl_file = node.get_parameter("trajectory_jsonl").value
    topic = node.get_parameter("joint_position_control_topic").value
    rate_hz = float(node.get_parameter("playback_rate_hz").value)

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    pub = node.create_publisher(ArmJointPositionControl, topic, qos)

    time.sleep(0.5)
    node.get_logger().info("Preparing to play back trajectory from %s..." % jsonl_file)
    playback_trajectory(node, jsonl_file, pub, rate_hz)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
