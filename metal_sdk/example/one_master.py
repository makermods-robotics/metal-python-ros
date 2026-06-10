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

    # Set gravity-compensation mode, usually for a leader arm
    master_arm.SetArmControlMode(ControlMode.GRAVITY_COMPENSATION)

    # The SDK control loop runs on this PC, so gravity compensation stops when this process exits
    # Read joint feedback
    while True:
        # End-effector pose
        arm_end_pose = master_arm.GetArmEndPose()
        # Joint position
        joint_position = master_arm.GetJointPosition()
        # Joint velocity
        joint_velocity = master_arm.GetJointVelocity()
        # Joint effort
        joint_effort = master_arm.GetJointEffort()

        print("arm end pose: ", arm_end_pose)
        print("arm joint position: ", joint_position)
        print("arm joint velocity: ", joint_velocity)
        print("arm joint effort: ", joint_effort)
        # Wait 10 ms
        time.sleep(0.01)
