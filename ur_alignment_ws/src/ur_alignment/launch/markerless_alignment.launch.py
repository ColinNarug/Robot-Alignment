#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),

        # Camera Node (Python executable name includes .py)
        Node(
            package='ur_alignment',
            executable='realsense_data_publisher.py',
            name='realsense_data_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # Object Node (markerless pose estimator, executable includes .py)
        Node(
            package='ur_alignment',
            executable='markerless_pose_estimator.py',
            name='markerless_pose_estimator',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # Displays / UI Node (executable includes .py)
        Node(
            package='ur_alignment',
            executable='user_interface.py',
            name='user_interface',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
