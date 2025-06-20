cmake_minimum_required(VERSION 3.8)
project(unity_scene_updater)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ur16e_unity_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(primitives_updater src/primitives_updater.cpp)
ament_target_dependencies(
  primitives_updater
  rclcpp
  ur16e_unity_interfaces
  geometry_msgs
  shape_msgs
  moveit_ros_planning_interface
)

install(TARGETS
  primitives_updater
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
