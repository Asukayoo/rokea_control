#!/usr/bin/env python3
"""Display the Rokea dual-arm torso in RViz."""

import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_robot_description(context, *args, **kwargs):
    model = LaunchConfiguration("model").perform(context)
    if model.endswith(".xacro"):
        import xacro

        robot_description = xacro.process_file(model).toxml()
    else:
        with open(model, "r", encoding="utf-8") as f:
            robot_description = f.read()

    rvizconfig = LaunchConfiguration("rvizconfig")
    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
    ]

    # Prefer GUI, then plain joint_state_publisher, else zero publisher script.
    if shutil.which("joint_state_publisher_gui") or _pkg_exists(
        "joint_state_publisher_gui"
    ):
        nodes.append(
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
            )
        )
    elif _pkg_exists("joint_state_publisher"):
        nodes.append(
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
            )
        )
    else:
        nodes.append(
            Node(
                package="rokea_description",
                executable="zero_joint_state_publisher.py",
                name="zero_joint_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            )
        )

    nodes.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rvizconfig],
        )
    )
    return nodes


def _pkg_exists(name: str) -> bool:
    try:
        from ament_index_python.packages import PackageNotFoundError, get_package_prefix

        get_package_prefix(name)
        return True
    except Exception:
        return False


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("rokea_description")
    default_model = os.path.join(pkg, "urdf", "rokea.urdf")
    default_rviz = os.path.join(pkg, "rviz", "rokea.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=default_model,
                description="Absolute path to robot URDF or xacro",
            ),
            DeclareLaunchArgument(
                "rvizconfig",
                default_value=default_rviz,
                description="Absolute path to RViz config",
            ),
            OpaqueFunction(function=_load_robot_description),
        ]
    )
