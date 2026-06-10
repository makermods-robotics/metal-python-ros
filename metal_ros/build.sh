#!/bin/bash

thread_num=$(($(nproc) - 1))

# Metal arm gazebo
# catkin_make install --pkg metal_gazebo -j${thread_num}

# Metal arm ros msgs
catkin_make install --only-pkg-with-deps metal_msg -j${thread_num}

# Metal arm gazebo
catkin_make install --only-pkg-with-deps  metal_gazebo -j${thread_num}

# Metal arm metal_moveit_ctrl
catkin_make install --only-pkg-with-deps  metal_moveit_ctrl -j${thread_num}

# Metal arm metal_no_gripper_moveit
catkin_make install --only-pkg-with-deps metal_no_gripper_moveit -j${thread_num}

# Metal arm metal_with_gripper_moveit
catkin_make install --only-pkg-with-deps  metal_with_gripper_moveit -j${thread_num}


# Metal arm ros1 driver
catkin_make install --only-pkg-with-deps metal_controller -j${thread_num}