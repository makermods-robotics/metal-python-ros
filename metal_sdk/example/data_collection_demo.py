# -*- coding: utf-8 -*-
"""
  ControlMode.GRAVITY_COMPENSATION
  Set the Metal arm to gravity-compensation mode for hand-guided teaching or data capture.

example:
    python3 one_master.py
"""

from metal_sdk import MetalSDKInterface, ControlMode
import os
import time

# Current script directory
HERE = os.path.dirname(os.path.abspath(__file__))

can_id = "can0"
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
    master_arm = MetalSDKInterface(
        can_id=can_id,
        urdf_path=urdf_path,
        arm_end_type=arm_end_type,
        enable_arm=auto_enable,
    )

    # Initialize the Metal SDK
    if not master_arm.Init():
        print("Init Metal SDK interface failed")
        raise RuntimeError("Metal SDK Init failed")

    # The SDK control loop runs on this PC, so gravity compensation stops when this process exits
    # Read joint feedback
    collect_count = 0
    while collect_count < 3:
      # Set gravity-compensation mode, usually for a leader arm
      master_arm.SetArmControlMode(ControlMode.GRAVITY_COMPENSATION)
      count = 0
      while count < 20:
          # End-effector pose
          arm_end_pose = master_arm.GetArmEndPose()
          # Joint position
          joint_position = master_arm.GetJointPosition()
          # Joint velocity
          joint_velocity = master_arm.GetJointVelocity()
          # Joint effort
          joint_effort = master_arm.GetJointEffort()

          # print("arm end pose: ", arm_end_pose)
          # print("arm joint position: ", joint_position)
          # print("arm joint velocity: ", joint_velocity)
          # print("arm joint effort: ", joint_effort)
          # Wait 100 ms
          print(f"collect {collect_count}th data, count: {count}")
          time.sleep(0.1)
          count += 1

      # Set control mode to NRT_JOINT_POSITION
      master_arm.SetArmControlMode(ControlMode.NRT_JOINT_POSITION)
      time.sleep(1)

      # Joint-position control
      joint_position_control = [0, 0, 0, 0, 0, 0]
      joint_velocity_control = 3  # Joint speed ratio (1-10), 1 is slowest, 10 is fastest, default is 5
      master_arm.SetArmJointPosition(joint_position_control, joint_velocity_control)  # Control J1-J6

      gripper_stroke = 0  # Gripper stroke (0-80 mm)
      gripper_velocity = 3 # Gripper speed ratio (1-10), 1 is slowest, 10 is fastest, default is 5
      master_arm.SetGripperStroke(gripper_stroke, gripper_velocity)  # Control gripper

      # Wait 3 seconds to return to zero
      time.sleep(3)
      collect_count += 1