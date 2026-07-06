#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ------------------------------------------------------------------
    # Launch Arguments
    # ------------------------------------------------------------------

    robot_model = DeclareLaunchArgument(
        "robot_model",
        default_value="crx10ia_l",
        description="FANUC robot model",
    )

    # ------------------------------------------------------------------
    # Include FANUC MoveIt Launch
    # ------------------------------------------------------------------

    fanuc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("fanuc_moveit_config"),
                "launch",
                "fanuc_moveit.launch.py",
            )
        ),
        launch_arguments={
            "robot_model": LaunchConfiguration("robot_model"),
            "use_mock": "true",
        }.items()
    )

    # ------------------------------------------------------------------
    # Human Tracker
    # ------------------------------------------------------------------

    human_tracker = Node(
        package="human_tracker",
        executable="human_pose",
        name="human_tracker",
        output="screen",
    )

    # ------------------------------------------------------------------
    # Human Avoidance
    # ------------------------------------------------------------------

    human_avoidance = Node(
        package="human_avoidance",
        executable="avoidance_node",
        name="human_avoidance",
        output="screen",
    )

    camera_tf = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    arguments=[
        "0", "0", "0.5",
        "-1.5708", "0", "-1.5708",
        "world",
        "camera_link",
    ],
    output="screen",
)

    # ------------------------------------------------------------------
    # Launch Description
    # ------------------------------------------------------------------

    return LaunchDescription(
        [
            robot_model,
            fanuc_launch,
            human_tracker,
            human_avoidance,
            camera_tf,
        ]
    )