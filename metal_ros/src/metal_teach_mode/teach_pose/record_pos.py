#!/usr/bin/env python3
import rospy, json, os
from metal_msg.msg import ArmJointState

joint_state = None
def joint_state_callback(msg):
    global joint_state
    joint_state = msg

if __name__ == "__main__":
    rospy.init_node('record_pose', anonymous=True)
    os.makedirs("data", exist_ok=True)
    rospy.Subscriber("/master_arm_right/joint_states",
                     ArmJointState, joint_state_callback, queue_size=1)

    count = 1

    with open("data/recorded_pos.jsonl", 'a') as f:
      while input("Input key [Enter] record pose, key [q] to stop recording") != "q":
          if joint_state is not None:
            data = {
                'position': list(joint_state.joint_position),
                'velocity': list(joint_state.joint_velocity),
                'effort': list(joint_state.joint_effort),
                "end_pose": list(joint_state.end_pose),
            }
            f.write(json.dumps(data) + '\n')
            print(f"record {count}th pose, position: {data['position']} , end_pose: {data['end_pose']}")
            count += 1
          else:
            print("No receive joint state data")

    print("record finish")
