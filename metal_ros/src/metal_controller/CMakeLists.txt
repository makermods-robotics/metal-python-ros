cmake_minimum_required(VERSION 3.0.2)
project(metal_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  metal_msg
)

catkin_package(
  CATKIN_DEPENDS roscpp roslib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)