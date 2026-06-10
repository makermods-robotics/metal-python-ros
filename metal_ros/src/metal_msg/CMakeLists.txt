cmake_minimum_required(VERSION 3.0.2)
project(metal_msg)

# set for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  ArmEndPoseControl.msg
  ArmJointPositionControl.msg
  ArmJointState.msg
  ArmStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()