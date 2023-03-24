cmake_minimum_required(VERSION 3.8)
project(sspmbot_hardware)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(amr_kinematics REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


add_library(
  ${PROJECT_NAME}
  SHARED
  src/sspmbot_system.cpp
  src/sspmbot_hardware_communicator.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  include
  ${amr_kinematics}_INCLUDE_DIRS
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  nav_msgs
  geometry_msgs
  tf2
  tf2_msgs
  tf2_geometry_msgs
  realtime_tools
  amr_kinematics
)

target_compile_definitions(
  ${PROJECT_NAME}
  PRIVATE
  "SSPMBOT_HARDWARE_BUILDING_DLL"
)

pluginlib_export_plugin_description_file(hardware_interface sspmbot_hardware.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
ament_package()