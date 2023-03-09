cmake_minimum_required(VERSION 3.8)
project(amr_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

include_directories(include/amr_kinematics)

set(HEADER_FILES 
  include/amr_kinematics/mecanum_kinematics.hpp
)

add_library(
  amr_kinematics_library
  SHARED
  src/mecanum_kinematics.cpp
  ${HEADER_FILES}
)


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
