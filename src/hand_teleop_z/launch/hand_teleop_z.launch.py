#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument


def launch_setup(context, *args, **kwargs):
    """Launch setup function"""
    
    # Include FANUC MoveIt launch with simulation parameters
    fanuc_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_moveit_config"),
                    "launch",
                    "fanuc_moveit.launch.py",
                ]
            ),
        ),
        launch_arguments={
            "robot_model": "crx10ia_l",  # CRX-10iA/L
            "robot_ip": "0.0.0.0",  # Dummy value; not used in mock mode
            "use_mock": "true",  # Simulation mode
            "motion_control": "1",
        }.items(),
    )
    
    # Start the hand teleop node
    hand_teleop_node = Node(
        package="hand_teleop_z",
        executable="hand_teleop_z_node",
        name="hand_teleop_z",
        output="screen",
        prefix="xterm -e",  # Launch in separate terminal for keyboard input
    )
    
    return [fanuc_moveit_launch, hand_teleop_node]


def generate_launch_description():
    """Generate launch description"""
    
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
