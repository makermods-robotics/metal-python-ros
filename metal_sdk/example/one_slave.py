# -*- coding: utf-8 -*-
"""
  ControlMode.RT_JOINT_POSITION
  Set the Metal arm to follower tracking mode. This mode is very fast.
  This mode accepts high-rate real-time commands up to 400 Hz and does not perform trajectory interpolation. Use it for leader-follower control or when your application already plans trajectories.

example:
    python3 one_slave.py
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
    slave_arm = MetalSDKInterface(
        can_id=can_id,
        urdf_path=urdf_path,
        arm_end_type=arm_end_type,
        enable_arm=auto_enable,
    )

    # Initialize the Metal SDK
    if not slave_arm.Init():
        print("Init Metal SDK interface failed")
        raise RuntimeError("Metal SDK Init failed")

    # Set real-time joint-position mode, usually for follower-arm tracking
    slave_arm.SetArmControlMode(ControlMode.RT_JOINT_POSITION)

    # The SDK control loop runs on this PC, so feedback and command handling stop when this process exits
    while True:
        # Read joint feedback
        # End-effector pose
        arm_end_pose = slave_arm.GetArmEndPose()
        # Joint position
        joint_position = slave_arm.GetJointPosition()
        # Joint velocity
        joint_velocity = slave_arm.GetJointVelocity()
        # Joint effort
        joint_effort = slave_arm.GetJointEffort()

        print("arm end pose: ", arm_end_pose)
        print("arm joint position: ", joint_position)
        print("arm joint velocity: ", joint_velocity)
        print("arm joint effort: ", joint_effort)

        # This mode is very fast. Use it mainly for leader-follower control with planned leader joint positions.
        # Send joint-position command
        # joint_position_control = [0.6, -0.6, 0.6, 0.5, 0.4, 0]  # First 6 values control J1-J6; final value controls gripper stroke (0-80 mm)
        # slave_arm.SetFollowerArmJointPosition(joint_position_control)

        # Send end-pose command
        # arm_end_pose_control = [0.0535, -0.0476, 0.3963, -0.3829, -1.0915, 2.5349]
        # slave_arm.SetArmEndPose(arm_end_pose_control)
        # slave_arm.SetGripperStroke(gripper_stroke, gripper_velocity)  # Set gripper stroke and speed

        # Wait 10 ms
        time.sleep(0.01)
