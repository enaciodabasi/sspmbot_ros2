cmake_minimum_required(VERSION 3.8)
project(amr_kinematics)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include/amr_kinematics)

set(HEADER_FILES 
  include/amr_kinematics/mecanum_kinematics.hpp
)

set(KINEMATICS_LIB_INCLUDE_DEPENDS
  rclcpp
  nav_msgs
  tf2
  tf2_geometry_msgs
  geometry_msgs
)

add_library(
  amr_kinematics_library
  SHARED
  src/mecanum_kinematics.cpp
  src/odom.cpp
)
ament_target_dependencies(amr_kinematics_library ${KINEMATICS_LIB_INCLUDE_DEPENDS})

ament_export_targets(amr_kinematics_library HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/amr_kinematics
  DESTINATION include
)

install(
  TARGETS amr_kinematics_library
  EXPORT amr_kinematics_library
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
