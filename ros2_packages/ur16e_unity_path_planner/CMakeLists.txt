cmake_minimum_required(VERSION 3.8)
project(ur16e_unity_path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(ur16e_unity_interfaces REQUIRED)

add_executable(ur16e_path_mover src/ur16e_path_mover.cpp)
ament_target_dependencies(ur16e_path_mover
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  std_srvs
  ur16e_unity_interfaces
)

install(TARGETS ur16e_path_mover
  DESTINATION lib/${PROJECT_NAME})

ament_package()
