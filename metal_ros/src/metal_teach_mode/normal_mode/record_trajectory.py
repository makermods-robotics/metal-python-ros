#!/usr/bin/env python3
import rospy, json, os
from metal_msg.msg import ArmJointState

joint_state = None
def state_callback(msg):
    global joint_state
    joint_state = msg

if __name__ == "__main__":
    rospy.init_node('record_trajectory', anonymous=True)
    os.makedirs("data", exist_ok=True)
    rospy.Subscriber("/master_arm_right/joint_states",
                     ArmJointState, state_callback, queue_size=1)

    input("press key [Enter] to start record trajectory.")
    if joint_state is None:
      print("No receive joint state data")
      exit()

    rate = rospy.Rate(10)
    with open("data/arm_state_10hz.jsonl", 'a') as f:
        count = 1
        while not rospy.is_shutdown():
            data = {
                'position': list(joint_state.joint_position),
                'velocity': list(joint_state.joint_velocity),
                'effort': list(joint_state.joint_effort),
                "end_pose": list(joint_state.end_pose),
            }
            f.write(json.dumps(data) + '\n')
            print(f"record {count}th pose, position: {data['position']} , end_pose: {data['end_pose']}")
            count += 1
            rate.sleep()
