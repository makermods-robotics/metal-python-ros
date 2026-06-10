# -*- coding: utf-8 -*-
"""
  ControlMode.NRT_JOINT_POSITION
  Set the Metal arm to non-real-time control mode for joint-position and end-pose commands. This is the recommended mode for model inference.
  This mode accepts high-rate real-time commands up to 400 Hz and does not perform trajectory interpolation. Use it for follower tracking


example:
    python3 single_arm_control.py
"""

from metal_sdk import MetalSDKInterface, ControlMode
import os
import time

# Current script directory
HERE = os.path.dirname(os.path.abspath(__file__))

can_id = "can1"
# Enable or disable motors
auto_enable = True
# 0: nothing, 1: gripper, 2: teaching pendant, 3: gripper and teaching pendant
arm_end_type = 3

if arm_end_type == 0:
    urdf_path = os.path.join(HERE, "urdf", "metal_no_gripper.urdf")
elif arm_end_type == 1:
  urdf_path = os.path.join(HERE, "urdf", "metal_with_gripper.urdf")
elif arm_end_type == 2:
  urdf_path = os.path.join(HERE, "urdf", "metal_with_gripper.urdf")
elif arm_end_type == 3:
  urdf_path = os.path.join(HERE, "urdf", "metal_with_gripper.urdf")
else:
    print(f"arm_end_type {arm_end_type} not supported")
    raise RuntimeError("Unsupported arm_end_type")

if __name__ == "__main__":
    # Initialize the Metal SDK
    single_control_arm = MetalSDKInterface(
        can_id=can_id,
        urdf_path=urdf_path,
        arm_end_type=arm_end_type,
        enable_arm=auto_enable,
    )

    # Initialize the Metal SDK
    if not single_control_arm.Init():
        print("Init Metal SDK interface failed")
        raise RuntimeError("Metal SDK Init failed")

    # Set gravity-compensation mode, usually for a leader arm
    single_control_arm.SetArmControlMode(ControlMode.NRT_JOINT_POSITION)

    # The SDK control loop runs on this PC, so feedback and command handling stop when this process exits

    # Joint-position control
    joint_position_control_flag = False
    if joint_position_control_flag:
      time.sleep(3)
      # joint_position_control = [0.6, -0.6, 0.6, 0.5, 0.4, 0]
      joint_position_control = [0, 0, 0, 0, 0, 0]
      joint_velocity_control = 3  # Joint speed ratio (1-10), 1 is slowest, 10 is fastest, default is 5
      single_control_arm.SetArmJointPosition(joint_position_control, joint_velocity_control)  # Control J1-J6

      gripper_stroke = 10  # Gripper stroke (0-80 mm)
      gripper_velocity = 3 # Gripper speed ratio (1-10), 1 is slowest, 10 is fastest, default is 5
      single_control_arm.SetGripperStroke(gripper_stroke, gripper_velocity)  # Control gripper

    # End-pose control
    end_pose_control_flag = True
    if end_pose_control_flag:
      time.sleep(3)
      arm_end_pose_control = [0.05, -0.04, 0.4, 0.2, -0.5, -1]
      joint_velocity_control = 3  # Joint speed ratio (1-10), 1 is slowest, 10 is fastest, default is 5
      ik_result = single_control_arm.SetArmEndPose(arm_end_pose_control, joint_velocity_control)  # End-pose arm command
      print("ik result: ", ik_result)

      gripper_stroke = 10  # Gripper stroke (0-80 mm)
      gripper_velocity = 3 # Gripper speed ratio (1-10), 1 is slowest, 10 is fastest, default is 5
      single_control_arm.SetGripperStroke(gripper_stroke, gripper_velocity)  # Control gripper

    # Read joint feedback
    while True:
        # End-effector pose
        arm_end_pose = single_control_arm.GetArmEndPose()
        # Joint position
        joint_position = single_control_arm.GetJointPosition()
        # Joint velocity
        joint_velocity = single_control_arm.GetJointVelocity()
        # Joint effort
        joint_effort = single_control_arm.GetJointEffort()

        # print("arm end pose: ", arm_end_pose)
        # print("arm joint position: ", joint_position)
        # print("arm joint velocity: ", joint_velocity)
        # print("arm joint effort: ", joint_effort)
        # Wait 100 ms
        time.sleep(0.1)
