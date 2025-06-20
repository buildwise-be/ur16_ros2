# UR16e - Unity ROS2 packages
:dizzy: ROS2 packages to help manipulate the UR16e. :dizzy:

## Packages included
The following packages are included:
- bw_ur_moveit_config
- unity_scene_updater
- ur16e_mover
- ur16e_unity_interfaces

## bw_ur_moveit_config package
This package is directly taken from the jazzy branch of Universal Robots ur_moveit_config package (see [here](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/jazzy/ur_moveit_config)). It is slightly modified to launch some additional custom nodes from the mover and scene_updater packages.

## unity_scene_updater package
This package is meant to add obstacles to the ros scene from Unity GameObjects.

## ur16e_mover package
This package provides a ROS2 Node that creates a UR16eMoverService Service, to which a Unity app can publish to. This Service type is defined in the ur16e_unity_interfaces package.

## ur16e_unity_interfaces package
This package defines interfaces for Unity-ROS2 communication.
It defines the following services:
- UR16eMoverService
- AddPrimitive
- UpdatePlanningScene

### UR16eMoverService

\# Request  
float64[6] joints  
geometry_msgs/Pose target_pose  
\---  
\# Response  
moveit_msgs/RobotTrajectory[] trajectories  

### AddPrimitive
\# Request
string    type                   \# "BOX" or "PLANE"
geometry_msgs/Pose    pose
geometry_msgs/Vector3 dimensions           # For BOX: [x,y,z]; for PLANE: [width, height, _] (thickness ignored)
bool      remove                 # true to remove this object instead of adding
string	id	# the GUID of the unity gameObject, created on the fly by unity when doing the call
\---
\# Response
bool      success
string    message
string 	id	# the GUID of the unity gameObject
bool remove

### UpdatePlanningScene
\# Request
string id
shape_msgs/Mesh mesh
geometry_msgs/Pose pose
string frame_id
\---
\# Response
bool success
string message
