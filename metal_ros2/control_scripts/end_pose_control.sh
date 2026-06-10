#!/bin/sh

ros2 topic pub -1 /master_arm_right/end_pose_control metal_msg/ArmEndPoseControl "header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
end_pose: [0.0535, -0.0476, 0.3963, 0.1990, -0.4414, -1.0352]
joint_velocity: 3
gripper_stroke: 50
gripper_velocity: 3"