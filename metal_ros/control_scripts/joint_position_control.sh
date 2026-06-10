#!/bin/sh

rostopic pub /master_arm_right/joint_states metal_msg/ArmJointPositionControl "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
joint_position: [0.6, -0.6, 0.6, 0.5, 0.4, 0]
joint_velocity: 3
gripper_stroke: 50
gripper_velocity: 3"
