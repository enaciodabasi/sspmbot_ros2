cmake_minimum_required(VERSION 3.8)
project(sspmbot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(amr_kinematics REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(
  sspmbot_amr_controller
  src/control_node.cpp
  src/controller.cpp
  src/hw_communicator.cpp
)
ament_target_dependencies(
  sspmbot_amr_controller
  rclcpp
  nav_msgs
  tf2_msgs
  sensor_msgs
  std_msgs
  realtime_tools
  tf2
  geometry_msgs
  tf2_geometry_msgs
  amr_kinematics
)
target_include_directories(
  sspmbot_amr_controller
  PUBLIC
  include
  ${amr_kinematics}_INCLUDE_DIRS
)

install(TARGETS sspmbot_amr_controller DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY
  include
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
