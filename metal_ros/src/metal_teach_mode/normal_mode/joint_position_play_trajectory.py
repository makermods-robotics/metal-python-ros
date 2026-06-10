#!/usr/bin/env python3

import rospy
import json
from metal_msg.msg import ArmJointPositionControl

def playback_trajectory(jsonl_file, control_pub):
    with open(jsonl_file, 'r') as f:
        data_lines = [json.loads(line) for line in f]

    if not data_lines:
        rospy.logerr("No data found in the file.")
        return

    data_len  = len(data_lines)
    print("trajectory size : ", data_len)
    idx = 1  # First point has already been sent.
    msg = ArmJointPositionControl()
    msg.header.stamp = rospy.Time.now()
    msg.joint_position = data_lines[0]['position'][0:6]
    msg.joint_velocity = 3
    input("Press key [Enter] to start play trajectory.")

    rospy.sleep(3) # Wait briefly so the first point can be sent reliably.

    control_pub.publish(msg)
    rospy.loginfo("Publish start position, sleeping for 3 seconds go to start position.")
    rospy.sleep(3)  # Give the arm 3 seconds to reach the target.
    print("Playback started.")

    rate = rospy.Rate(25)
    while not rospy.is_shutdown() and idx < data_len:
        msg.header.stamp = rospy.Time.now()
        msg.joint_position = data_lines[idx]['position'][0:6]
        # msg.gripper_stroke = data_lines[idx]['position'][6]

        control_pub.publish(msg)
        idx += 1
        print("index: ", idx)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('play_trajectory', anonymous=True)

    jsonl_file = "/path/to/metal_ros/data/arm_state_25hz.jsonl"

    pub = rospy.Publisher('/master_arm_right/joint_states', ArmJointPositionControl, queue_size=1)

    rospy.loginfo(f"Preparing to play back trajectory from {jsonl_file}...")
    playback_trajectory(jsonl_file, pub)
