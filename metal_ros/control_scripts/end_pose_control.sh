#!/bin/sh

rostopic pub /master_arm_right/end_pose_control metal_msg/ArmEndPoseControl "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
end_pose: [0.0535, -0.0476, 0.3963, -0.3829, -1.0915, 2.5349]
joint_velocity: 3
gripper_stroke: 50
gripper_velocity: 3"