std_msgs/Header header

# x y z roll pitch yaw
float64[6] end_pose

# range: [1, 10], 1 is slow, 10 is fast
uint8 joint_velocity

# individual gripper control
float64 gripper_stroke

# range: [1, 10], 1 is slow, 10 is fast
uint8 gripper_velocity