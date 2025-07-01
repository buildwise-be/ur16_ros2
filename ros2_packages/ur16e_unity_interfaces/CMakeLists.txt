cmake_minimum_required(VERSION 3.8)
project(ur16e_unity_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
	"srv/UR16eMoverService.srv"
	"srv/UpdatePlanningScene.srv"
	"srv/AddPrimitive.srv"
	"srv/UR16ePathPlan.srv"
	DEPENDENCIES geometry_msgs moveit_msgs shape_msgs builtin_interfaces
       	)

ament_export_dependencies(rosidl_default_runtime geometry_msgs moveit_msgs)

ament_package()
