# file: launch/ur_full_stack.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"),
                "launch",
                "ur_control.launch.py"
            ])
        ]),
        launch_arguments={
            "ur_type": "ur16e",
            "robot_ip": "192.168.125.101",
            "reverse_ip": "192.168.125.1",
            "launch_rviz": "false"
        }.items()
    )

    unity_tcp_server = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_tcp_endpoint", "default_server_endpoint",
            "--ros-args", "-p", "ROS_IP:=0.0.0.0"
        ],
        output="screen"
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("bw_ur_moveit_config"),
                "launch",
                "ur_moveit.launch.py"
            ])
        ]),
        launch_arguments={
            "ur_type": "ur16e",
            "launch_rviz": "true"
        }.items()
    )

    return LaunchDescription([
        ur_control_launch,
        unity_tcp_server,
        moveit_launch
    ])
