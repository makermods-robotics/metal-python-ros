#!/bin/sh

ros2 topic pub -1 /master_arm_right/joint_states metal_arm_msgs/msg/ArmJointPositionControl "header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
joint_position: [0, 0, 0, 0, 0, 0]
joint_velocity: 3
gripper_stroke: 0
gripper_velocity: 3"