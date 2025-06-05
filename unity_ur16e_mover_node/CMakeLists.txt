cmake_minimum_required(VERSION 3.8)
project(unity_ur16e_mover_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp               REQUIRED)
find_package(geometry_msgs        REQUIRED)
find_package(moveit_msgs          REQUIRED)
find_package(moveit_core          REQUIRED)
find_package(moveit_ros_planning  REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(ur16e_unity_interfaces REQUIRED)

add_executable(unity_ur16e_mover src/unity_ur16e_mover.cpp)
ament_target_dependencies(
  unity_ur16e_mover
  rclcpp
  geometry_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  ur16e_unity_interfaces
)

install(TARGETS
	unity_ur16e_mover
	DESTINATION lib/${PROJECT_NAME}
	)

ament_package()
