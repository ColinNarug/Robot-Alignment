#!/usr/bin/env python3
"""
Launch args:
  use_camera:=true|false   -> starts/stops d435i_camera
  use_robot:=true|false    -> starts/stops wrench and ur_e_series
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_camera = LaunchConfiguration("use_camera")
    use_robot = LaunchConfiguration("use_robot")

    d435i_camera = Node(
        package="uralignment_cpp",
        executable="d435i_camera",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(use_camera),
    )
    
    ur_e_series = Node(
        package="uralignment_cpp",
        executable="ur_e_series",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(use_robot),
    )

    apriltags = Node(
        package="uralignment_cpp",
        executable="apriltags",
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_camera", default_value="true"),
        DeclareLaunchArgument("use_robot", default_value="true"),

        d435i_camera,
        TimerAction(period=10.0, actions=[ur_e_series]),
        TimerAction(period=5.0, actions=[apriltags]),
    ])
