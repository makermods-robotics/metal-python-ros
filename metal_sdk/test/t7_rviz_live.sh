#!/usr/bin/env bash
# READ-ONLY live visualization: move the real arm by hand, RViz model follows.
# Starts driver(motors OFF) + ArmJointState->JointState bridge + RSP + RViz.
source "$(dirname "$0")/_env.sh"
ros2 launch "$(dirname "$0")/rviz_live.launch.py"
