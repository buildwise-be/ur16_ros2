# ur16_ros2
:dizzy: ROS2 packages to help manipulate the UR16e. :dizzy:

## Packages included
The following packages are included:
- mir250_ur16e_moveit_config
- mir250_ur16e_description
- ur16e_mover
- ur16e_unity_interfaces

## mir250_ur16e_moveit_config package
This package is directly taken from the jazzy branch of Universal Robots ur_moveit_config package (see [here](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/jazzy/ur_moveit_config)). It allows to launch Moveit2 with the custom URDF file provided by the mir250_ur16e_description package from this repo.

## mir250_ur16e_description package
This package is built on top of the urdf (xacro) files of the ur_robot_driver package of Universal Robots (see [here](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/main/ur_robot_driver)). It adds the MIR250 robot under it, as well as some obstacles.

## ur16e_mover package
This package provides a ROS2 Node that creates a UR16eMoverService Service, to which a Unity app can publish to. This Service type is defined in the ur16e_unity_interfaces package.

## ur16e_unity_interfaces package
This package defines interfaces for Unity-ROS2 communication.
It defines the following services:
- UR16eMoverService

### UR16eMoverService

\# Request  
float64[6] joints  
geometry_msgs/Pose target_pose  
\---  
\# Response  
moveit_msgs/RobotTrajectory[] trajectories  
