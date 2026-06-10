#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import math

# Joint list
joint_names = [
    "joint1", "joint2", "joint3", "joint4",
    "joint5", "joint6", "joint7", "joint8"
]

# Publisher map
publishers = {}

# Cache previous joint positions
last_positions = {}

def joint_state_callback(msg):
    # Convert JointState to a dictionary
    joint_positions = {name: pos for name, pos in zip(msg.name, msg.position)}

    for joint_name in joint_names:
        position = 0.0

        if joint_name == "joint8":
            # joint8 = joint7; change to -joint7 if needed
            if "joint7" in joint_positions:
                position = -joint_positions["joint7"]
            else:
                position = 0.0
        else:
            if joint_name in joint_positions:
                position = joint_positions[joint_name]
            else:
                continue  # No data for this joint

        # Publish only when the joint position changes
        last_pos = last_positions.get(joint_name, None)
        if last_pos is None or abs(position - last_pos) > 1e-5:
            cmd = Float64()
            cmd.data = position
            publishers[joint_name].publish(cmd)
            rospy.loginfo(f"Publishing {joint_name} position: {position:.6f}")
            last_positions[joint_name] = position


def main():
    rospy.init_node("joint_states_ctrl")

    # Initialize publishers
    for name in joint_names:
        topic = f"/gazebo/{name}_position_controller/command"
        publishers[name] = rospy.Publisher(topic, Float64, queue_size=10)

    # Subscribe to joint_states
    rospy.Subscriber("/joint_states", JointState, joint_state_callback)

    rospy.loginfo("joint_states_ctrl node started")
    rospy.spin()


if __name__ == "__main__":
    main()
