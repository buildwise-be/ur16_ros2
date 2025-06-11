cmake_minimum_required(VERSION 3.5)
project(unity_scene_updater)

# allow C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ===== Declare and generate the custom service =====
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddPrimitive.srv"
)

# ===== Build the planning_scene_updater node =====
add_executable(planning_scene_updater src/planning_scene_updater.cpp)

ament_target_dependencies(planning_scene_updater
  rclcpp
  geometry_msgs
  shape_msgs
  moveit_msgs
  moveit_ros_planning_interface
)

# Link the generated service typesupport
rosidl_target_interfaces(planning_scene_updater
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(TARGETS planning_scene_updater
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

