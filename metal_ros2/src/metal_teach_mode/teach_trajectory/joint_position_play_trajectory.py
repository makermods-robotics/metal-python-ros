#!/usr/bin/env python3
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from metal_msg.msg import ArmJointState


def playback_trajectory(node: Node, jsonl_file: str, control_pub, rate_hz: float):
    with open(jsonl_file, "r", encoding="utf-8") as f:
        data_lines = [json.loads(line) for line in f]

    if not data_lines:
        node.get_logger().error("No data found in the file.")
        return

    first_positions = data_lines[0].get("position")
    if not first_positions:
        node.get_logger().error('First line has no "position" field or it is empty.')
        return
    joint_count = len(first_positions)
    node.get_logger().info(
        'First waypoint "position" has %d element(s); playback uses position[:%d] each frame.'
        % (joint_count, joint_count)
    )

    time.sleep(3)
    data_len = len(data_lines)
    node.get_logger().info("data size : %d" % data_len)
    input("Press key [Enter] to start play trajectory.")

    idx = 1
    msg = ArmJointState()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.joint_position = list(data_lines[0]["position"][:joint_count])
    control_pub.publish(msg)
    node.get_logger().info(
        "Publish start position, sleeping for 3 seconds go to start position."
    )
    time.sleep(3)

    period = 1.0 / rate_hz
    while rclpy.ok() and idx < data_len:
        row = data_lines[idx]["position"]
        if len(row) < joint_count:
            node.get_logger().error(
                "Line %d: position length %d < expected %d, skip/stop."
                % (idx, len(row), joint_count)
            )
            break
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.joint_position = list(row[:joint_count])
        control_pub.publish(msg)
        idx += 1
        node.get_logger().info("index: %d" % idx)
        time.sleep(period)


def main(args=None):
    rclpy.init(args=args)
    node = Node("play_trajectory_teach_joint_stream")
    node.declare_parameter(
        "trajectory_jsonl", "data/arm_state_400hz.jsonl"
    )
    node.declare_parameter(
        "joint_command_topic", "/master_arm_right/joint_states"
    )
    node.declare_parameter("playback_rate_hz", 400.0)

    jsonl_file = node.get_parameter("trajectory_jsonl").value
    topic = node.get_parameter("joint_command_topic").value
    rate_hz = float(node.get_parameter("playback_rate_hz").value)

    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    pub = node.create_publisher(ArmJointState, topic, qos)

    time.sleep(0.5)
    node.get_logger().info("Preparing to play back trajectory from %s..." % jsonl_file)
    playback_trajectory(node, jsonl_file, pub, rate_hz)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
