# ur16_ros2
:dizzy: ROS2 packages to help manipulate the UR16e. :dizzy:

## Packages included
The following packages are included:
- mir250_ur16e_moveit_config
- mir250_ur16e_description

## mir250_ur16e_moveit_config package
This package is directly taken from the jazzy branch of Universal Robots ur_moveit_config package (see [here](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/jazzy/ur_moveit_config)). It allows to launch Moveit2 with the custom URDF file provided by the mir250_ur16e_description package from this repo.

## mir250_ur16e_description package
This package is built on top of the urdf (xacro) files of the ur_robot_driver package of Universal Robots (see [here](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/main/ur_robot_driver)). It adds the MIR250 robot under it, as well as some obstacles.
