#!/usr/bin/env bash
# READ-ONLY: run the driver node (leader_arm, auto_enable=false -> motors OFF),
# list topics, echo one joint_state + one status, then stop. Arm does not move.
source "$(dirname "$0")/_env.sh"
ros2 run metal_arm_driver metal_arm_driver --ros-args \
  -p arm_can_id:=can0 -p arm_control_type:=leader_arm \
  -p arm_end_type:=0 -p auto_enable:=false > /tmp/metal_node.log 2>&1 &
NODE=$!
sleep 4
echo "--- metal topics ---"; ros2 topic list | grep metal || true
echo "--- joint_state ---"; ros2 topic echo --once /metal/arm_joint_state || true
echo "--- status ---";      ros2 topic echo --once /metal/arm_status || true
kill $NODE 2>/dev/null || true
echo "--- node log tail ---"; tail -6 /tmp/metal_node.log
