#!/usr/bin/env python3
"""Patrick MoveIt2 demo — full-stack convenience launch.

Brings up robot + Gazebo + ros2_control + MoveIt + RViz in one command.
Useful for local development / sanity checks.

Usage:
    ros2 launch patrick_moveit_config demo.launch.py

For the two-shell workflow (recommended for real hardware debugging) see
patrick_moveit.launch.py.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    patrick_main_launch = os.path.join(
        get_package_share_directory('patrick_main'),
        'launch', 'patrick_bringup.launch.py')

    patrick_moveit_launch = os.path.join(
        get_package_share_directory('patrick_moveit_config'),
        'launch', 'patrick_moveit.launch.py')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(patrick_main_launch),
        launch_arguments={
            'simulation': 'true',
            'rviz': 'false',            # MoveIt launch brings its own RViz
            'vision': 'false',
        }.items(),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(patrick_moveit_launch),
        launch_arguments={
            'use_sim_time': 'true',
            'rviz': 'true',
            'simulation': 'true',
        }.items(),
    )

    # Small delay so Gazebo's /clock is publishing before RViz subscribes.
    # Without it RViz briefly uses wall-time and then a /clock jump triggers
    # a buffer reset, which some MoveIt rviz plugins don't handle cleanly.
    delayed_moveit = TimerAction(period=8.0, actions=[moveit])

    return LaunchDescription([bringup, delayed_moveit])
