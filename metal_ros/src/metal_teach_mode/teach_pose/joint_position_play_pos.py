#!/usr/bin/env python3

import rospy
import json
import time
from metal_msg.msg import ArmJointPositionControl, ArmJointState

joint_state = None
def joint_state_callback(msg):
    global joint_state
    joint_state = msg

if __name__ == '__main__':
    rospy.init_node('joint_position_play_pos', anonymous=True)

    # parameters
    jsonl_file = "/path/to/metal_ros/data/recorded_pos.jsonl"
    sleep_time = 2

    rospy.Subscriber("/metal/arm_joint_state",
                     ArmJointState, joint_state_callback, queue_size=1)
    joint_position_control_pub = rospy.Publisher('/metal/arm_joint_position_control', ArmJointPositionControl, queue_size=1)

    rospy.loginfo(f"Preparing to play pos from {jsonl_file}...")

    with open(jsonl_file, 'r') as f:
      data_lines = [json.loads(line) for line in f]

    if not data_lines:
      print(f"{jsonl_file} file no data.")
      exit()

    time.sleep(3) # Wait briefly so the first point can be sent reliably.
    if joint_state is None:
      print("No receive joint state data")
      exit()

    msg = ArmJointPositionControl()
    msg.header.stamp = rospy.Time.now()
    data_len  = len(data_lines)
    print("total pos number : ", data_len)
    input("input key [Enter] to start play position.")

    for i in range(data_len):
      target_pos = data_lines[i]['position'][0:6]
      print(f"play {i + 1}th position, current position: {joint_state.joint_position}, target position: {target_pos}")
      msg.arm_joint_position = target_pos
      # Wait briefly so the first point can be sent reliably.
      joint_position_control_pub.publish(msg)

      while True:
        current_pos = joint_state.joint_position
        if all(abs(current_pos[i] - target_pos[i]) < 0.01 for i in range(6)):
          print("reach target position")
          break

        if rospy.is_shutdown() == True:
          exit()

      time.sleep(sleep_time)