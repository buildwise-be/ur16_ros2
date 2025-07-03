# Copyright (c) 2024 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# … [license text truncated for brevity] …

import os
import yaml

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:
        return None


def declare_arguments():
    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?"),
            DeclareLaunchArgument(
                "ur_type",
                description="Typo/series of used UR robot.",
                choices=[
                    "ur3",
                    "ur3e",
                    "ur5",
                    "ur5e",
                    "ur7e",
                    "ur10",
                    "ur10e",
                    "ur12e",
                    "ur16e",
                    "ur15",
                    "ur20",
                    "ur30",
                ],
            ),
            DeclareLaunchArgument(
                "warehouse_sqlite_path",
                default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
                description="Path where the warehouse database should be stored",
            ),
            DeclareLaunchArgument("launch_servo", default_value="false", description="Launch Servo?"),
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Using or not time from simulation"
            ),
            DeclareLaunchArgument(
                "publish_robot_description_semantic",
                default_value="true",
                description="MoveGroup publishes robot description semantic",
            ),
        ]
    )


def generate_launch_description():
    launch_rviz = LaunchConfiguration("launch_rviz")
    ur_type = LaunchConfiguration("ur_type")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    launch_servo = LaunchConfiguration("launch_servo")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_robot_description_semantic = LaunchConfiguration("publish_robot_description_semantic")

    # Build MoveIt configuration (this already populates URDF, SRDF, kinematics, etc.)
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="bw_ur_moveit_config")
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": ur_type})
        .to_moveit_configs()
    )

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    ld = LaunchDescription()
    ld.add_entity(declare_arguments())

    # Wait for the robot_description parameter (published by the driver)
    wait_robot_description = Node(
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        output="screen",
    )
    ld.add_action(wait_robot_description)

    # 1) MoveIt move_group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),           # includes robot_description, robot_description_semantic, kinematics, etc.
            warehouse_ros_config,
            {
                "use_sim_time": use_sim_time,
                "publish_robot_description_semantic": publish_robot_description_semantic,
            },
        ],
    )

    # 2) MoveIt Servo (optional)
    servo_yaml = load_yaml("bw_ur_moveit_config", "config/ur_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(launch_servo),
        executable="servo_node",
        parameters=[
            moveit_config.to_dict(),
            servo_params,
        ],
        output="screen",
    )

    # 3) RViz (optional)
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("bw_ur_moveit_config"), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )

    # unity_ur16e_mover service node
    service_node = Node(
        package="unity_ur16e_mover_node",       # your C++ node’s package
        executable="unity_ur16e_mover",         # the name of the executable
        output="screen",
        parameters=[
            moveit_config.robot_description,         # exactly the same URDF parameter
            moveit_config.robot_description_semantic,# same SRDF parameter
            moveit_config.robot_description_kinematics,  # same kinematics.yaml
            {"use_sim_time": use_sim_time},          # if you want the service node to use sim time
        ],
    )

    # primitive scene updater node
    primitives_updater_node = Node(
        package="unity_scene_updater",
        executable="primitives_updater",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )

    # ur16e_path_mover service node
    path_mover_node = Node(
        package="ur16e_unity_path_planner",       # your C++ node’s package
        executable="ur16e_path_mover",         # the name of the executable
        output="screen",
        parameters=[
            moveit_config.robot_description,         # exactly the same URDF parameter
            moveit_config.robot_description_semantic,# same SRDF parameter
            moveit_config.robot_description_kinematics,  # same kinematics.yaml
            {"use_sim_time": use_sim_time},          # if you want the service node to use sim time
        ],
    )

    #
    # We want to start MoveIt, RViz, Servo and the service node
    # only after “wait_for_robot_description” finishes.
    #
    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_robot_description,
                on_exit=[
                    move_group_node,
                    rviz_node,
                    servo_node,
                    service_node,
                    primitives_updater_node,
                    path_mover_node,
                ],
            )
        ),
    )

    return ld
