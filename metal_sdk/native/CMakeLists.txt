cmake_minimum_required(VERSION 3.16)
project(metal_sdk)

# set for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set for gdb debug
set(CMAKE_CXX_FLAGS "--std=c++17 -O0 -Og -ggdb")
add_compile_options(-std=c++17)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)

# 包含目录
include_directories(${PROJECT_SOURCE_DIR} 
                    ${EIGEN3_INCLUDE_DIRS}
                    ${orocos_kdl_INCLUDE_DIRS}
                    ${urdfdom_headers_INCLUDE_DIRS}
                    ${urdf_INCLUDE_DIRS}
)

# track ik
# 查找静态库
find_library(TRAC_IK_STATIC_LIB
    NAMES trac_ik_lib
    PATHS ${PROJECT_SOURCE_DIR}/third_lib/trac_ik_lib/lib
)

# 源文件列表
set(METAL_SDK_SRCS
    config/motor_config.cpp
    motor_interface_base/motor_reader_base.cpp
    motor_interface_base/motor_writer_base.cpp
    motor_readers/dm_motor_reader.cpp
    motor_writers/dm_motor_writer.cpp
    kinodynamic/kdl_solver.cpp
    kinodynamic/payload_estimator/payload_estimator.cpp
    can_manager.cpp
    metal_sdk_interface.cpp
    trajectory/polynomial_interpolation.cpp
)

add_library(metal_sdk SHARED ${METAL_SDK_SRCS})
target_link_libraries(metal_sdk PUBLIC
    ${TRAC_IK_STATIC_LIB}  # 静态链接 trac_ik
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs__rosidl_generator_c
    kdl_parser::kdl_parser
    # orocos_kdl
    ${urdfdom_LIBRARIES}
    # urdf
    nlopt          # 显式链接 NLopt 动态库
)

target_include_directories(metal_sdk PUBLIC
  ${PROJECT_SOURCE_DIR}/third_lib/trac_ik_lib/include
)

# 根据架构设置安装目录和库名称
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
    set(ARCH_DIR "lib/arm64")
    set(LIB_NAME "metal_sdk_arm64")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "AMD64")
    set(ARCH_DIR "lib/x64")
    set(LIB_NAME "metal_sdk_x64")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

set_target_properties(metal_sdk PROPERTIES OUTPUT_NAME ${LIB_NAME})

# install include directories
install(FILES metal_sdk_interface.h
        DESTINATION include/${PROJECT_NAME}/
)

# 安装库文件到对应的架构目录，并重命名库文件
install(TARGETS metal_sdk
    # LIBRARY DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${ARCH_DIR}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION ${ARCH_DIR}
)

# 导出依赖
ament_export_include_directories(include)
ament_export_libraries(metal_sdk)
ament_export_dependencies(
    rclcpp
    sensor_msgs
    kdl_parser
    orocos_kdl
)

# 最终包配置
ament_package()