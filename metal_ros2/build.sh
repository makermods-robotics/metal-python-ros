#!/bin/bash

# Metal arm gazebo
# catkin_make install --pkg metal_gazebo -j${thread_num}

# Metal arm ros msgs
colcon build --packages-select metal_msg

# metal description
colcon build --packages-select metal_description

# Metal arm ros2 driver
colcon build --packages-select metal_controller

# Metal arm no gripper ros2 moveit
colcon build --packages-select metal_no_gripper_moveit

# Metal arm with gripper ros2 moveit
colcon build --packages-select metal_with_gripper_moveit

# Metal arm metal_moveit_ctrl
# colcon build --packages-select metal_moveit_ctrl

# Metal arm metal_gazebo
colcon build --packages-select metal_gazebo