# -*- coding: utf-8 -*-
"""
  Two-arm control demo.
  ControlMode.NRT_JOINT_POSITION
  Set the Metal arm to non-real-time control mode for joint-position and end-pose commands. This is the recommended mode for model inference.

example:
    python3 two_arm_control.py
"""

from metal_sdk import MetalSDKInterface, ControlMode
import os
import time

# Current script directory
HERE = os.path.dirname(os.path.abspath(__file__))

# Enable or disable motors
auto_enable = True
# 0: nothing, 1: gripper, 2: teaching pendant, 3: gripper and teaching pendant
arm_end_type = 0

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
    right_arm = MetalSDKInterface(
        can_id="can0",
        urdf_path=urdf_path,
        arm_end_type=arm_end_type,
        enable_arm=auto_enable,
    )

    left_arm = MetalSDKInterface(
        can_id="can1",
        urdf_path=urdf_path,
        arm_end_type=arm_end_type,
        enable_arm=auto_enable,
    )

    # Initialize the Metal SDK
    if not right_arm.Init():
        print("Init right arm SDK Interface failed")
        raise RuntimeError("Right arm SDK Init failed")

    if not left_arm.Init():
        print("Init left arm SDK Interface failed")
        raise RuntimeError("Left arm SDK Init failed")

    # Set non-real-time control mode for joint-position and end-pose control
    right_arm.SetArmControlMode(ControlMode.NRT_JOINT_POSITION)
    left_arm.SetArmControlMode(ControlMode.NRT_JOINT_POSITION)

    # The SDK control loop runs on this PC, so feedback and command handling stop when this process exits.

    # Joint-position control
    time.sleep(3)
    joint_position_control_flag = False
    if joint_position_control_flag:
      # control right arm
      joint_position_control = [0.6, -0.6, 0.6, 0.5, 0.4, 0]
      joint_velocity_control = 3  # Joint speed ratio (1-10), 1 is slowest, 10 is fastest, default is 5
      right_arm.SetArmJointPosition(joint_position_control, joint_velocity_control)  # Control J1-J6

      gripper_stroke = 10  # Gripper stroke (0-80 mm)
      gripper_velocity = 3 # Gripper speed ratio (1-10), 1 is slowest, 10 is fastest, default is 5
      right_arm.SetGripperStroke(gripper_stroke, gripper_velocity)  # Control gripper

      # control left arm
      left_arm.SetArmJointPosition(joint_position_control, joint_velocity_control)
      left_arm.SetGripperStroke(gripper_stroke, gripper_velocity)  # Control gripper

    # End-pose control
    time.sleep(3)
    end_pose_control_flag = False
    if end_pose_control_flag:
      arm_end_pose_control = [0.0535, -0.0476, 0.3963, -0.3829, -1.0915, 2.5349]
      right_arm.SetArmEndPose(arm_end_pose_control)  # End-pose arm command

      gripper_stroke = 10  # Gripper stroke (0-80 mm)
      gripper_velocity = 3 # Gripper speed ratio (1-10), 1 is slowest, 10 is fastest, default is 5
      right_arm.SetGripperStroke(gripper_stroke, gripper_velocity)  # Control gripper

    # Read joint feedback
    while True:
        # right_arm
        # End-effector pose
        right_arm_end_pose = right_arm.GetArmEndPose()
        # Joint position
        right_arm_joint_position = right_arm.GetJointPosition()
        # Joint velocity
        right_arm_joint_velocity = right_arm.GetJointVelocity()
        # Joint effort
        right_arm_joint_effort = right_arm.GetJointEffort()

        print("right arm end pose: ", right_arm_end_pose)
        print("right arm joint position: ", right_arm_joint_position)
        print("right arm joint velocity: ", right_arm_joint_velocity)
        print("right arm joint effort: ", right_arm_joint_effort)

        # left_arm
        # End-effector pose
        left_arm_end_pose = left_arm.GetArmEndPose()
        # Joint position
        left_arm_joint_position = left_arm.GetJointPosition()
        # Joint velocity
        left_arm_joint_velocity = left_arm.GetJointVelocity()
        # Joint effort
        left_arm_joint_effort = left_arm.GetJointEffort()

        print("left arm end pose: ", left_arm_end_pose)
        print("left arm joint position: ", left_arm_joint_position)
        print("left arm joint velocity: ", left_arm_joint_velocity)
        print("left arm joint effort: ", left_arm_joint_effort)

        # Wait 10 ms
        time.sleep(0.01)
