cmake_minimum_required(VERSION 3.0.2)
project(metal_moveit_ctrl)

## Find catkin macros and libraries
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(catkin REQUIRED COMPONENTS
  moveit_commander
  roscpp
  moveit_ros_planning_interface
  sensor_msgs
  message_generation
)

# Add service files
add_service_files(
  FILES
  JointMoveitCtrl.srv  # Custom service file
)

# Generate messages and services
generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs message_runtime
)

# Include headers
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/joint_moveit_ctrl_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)