#!/usr/bin/env python3
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from metal_msg.msg import ArmEndPoseControl


def _infer_gripper_from_position(node: Node, data_lines):
    """Return (use_gripper_stroke: bool, joint_count: int). joint_count is 6 or 7."""
    first = data_lines[0]
    pos = first.get("position")
    if pos is None:
        node.get_logger().warn(
            'No "position" in first line; gripper_stroke not driven (arm-only).'
        )
        return False, 0
    n = len(pos)
    if n == 6:
        node.get_logger().info(
            'First line "position" has 6 values; end_pose only, gripper_stroke ignored.'
        )
        return False, 6
    if n == 7:
        node.get_logger().info(
            'First line "position" has 7 values; using position[6] as gripper_stroke each frame.'
        )
        return True, 7
    node.get_logger().error(
        'First line "position" length is %d; only 6 (arm) or 7 (arm+gripper) are supported.'
        % n
    )
    return None, n


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


def playback_trajectory(node: Node, jsonl_file: str, control_pub, rate_hz: float):
    with open(jsonl_file, "r", encoding="utf-8") as f:
        data_lines = [json.loads(line) for line in f]

    if not data_lines:
        node.get_logger().error("No data found in the file.")
        return

    use_gripper, _ = _infer_gripper_from_position(node, data_lines)
    if use_gripper is None:
        return

    data_len = len(data_lines)
    node.get_logger().info("trajectory size : %d" % data_len)
    time.sleep(3)
    input("Press key [Enter] to start play trajectory.")

    idx = 1
    msg = ArmEndPoseControl()
    msg.header.stamp = node.get_clock().now().to_msg()
    try:
        _fill_end_pose_msg(msg, data_lines[0], use_gripper, node, 0)
    except ValueError:
        return
    control_pub.publish(msg)
    node.get_logger().info(
        "Publish start position, sleeping for 3 seconds go to start position."
    )
    time.sleep(3)

    period = 1.0 / rate_hz
    while rclpy.ok() and idx < data_len:
        msg.header.stamp = node.get_clock().now().to_msg()
        try:
            _fill_end_pose_msg(msg, data_lines[idx], use_gripper, node, idx)
        except ValueError:
            break
        control_pub.publish(msg)
        idx += 1
        node.get_logger().info("index: %d" % idx)
        time.sleep(period)


def main(args=None):
    rclpy.init(args=args)
    node = Node("play_trajectory_end_pose_high_rate")
    node.declare_parameter(
        "trajectory_jsonl", "data/arm_state_400hz.jsonl"
    )
    node.declare_parameter(
        "end_pose_control_topic", "/metal/arm_end_pose_control"
    )
    node.declare_parameter("playback_rate_hz", 400.0)

    jsonl_file = node.get_parameter("trajectory_jsonl").value
    topic = node.get_parameter("end_pose_control_topic").value
    rate_hz = float(node.get_parameter("playback_rate_hz").value)

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    pub = node.create_publisher(ArmEndPoseControl, topic, qos)

    time.sleep(0.5)
    node.get_logger().info("Preparing to play trajectory from %s..." % jsonl_file)
    playback_trajectory(node, jsonl_file, pub, rate_hz)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
