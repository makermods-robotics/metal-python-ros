#!/usr/bin/env bash
# READ-ONLY: render the URDF model in RViz with slider joint control.
# Validates description package + meshes. Needs ros-humble-rviz2 + xacro + jsp-gui.
source "$(dirname "$0")/_env.sh"
ros2 launch metal_description display_metal_no_gripper.launch.py
