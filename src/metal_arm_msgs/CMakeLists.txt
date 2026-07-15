cmake_minimum_required(VERSION 3.8)
project(metal_arm_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmEndPoseControl.msg"
  "msg/ArmJointPositionControl.msg"
  "msg/ArmJointState.msg"
  "msg/ArmStatus.msg"
  DEPENDENCIES
  std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
