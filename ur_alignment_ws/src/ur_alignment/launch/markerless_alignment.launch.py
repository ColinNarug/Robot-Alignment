#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    realsense_node = Node(
        package="ur_alignment",
        executable="realsense_data_publisher.py",
        output="screen",
        emulate_tty=True,
    )

    markerless_node = Node(
        package="ur_alignment",
        executable="markerless_pose_estimator.py",
        output="screen",
        emulate_tty=True,
    )

    ui_node = Node(
        package="ur_alignment",
        executable="user_interface.py",
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        realsense_node,
        TimerAction(period=10.0, actions=[markerless_node]),
        TimerAction(period=15.0, actions=[ui_node]),
    ])
