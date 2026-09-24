#!/usr/bin/env python3
"""Run MIMOSA and replay a ROS 2 bag using simulated time."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("mimosa")
    core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "mimosa.launch.py")),
        launch_arguments={
            "profile": LaunchConfiguration("profile"),
            "viz": LaunchConfiguration("viz"),
            "use_sim_time": "true",
            "config_override": LaunchConfiguration("config_override"),
        }.items(),
    )
    play = ExecuteProcess(
        cmd=["ros2", "bag", "play", LaunchConfiguration("bag_name"), "--clock",
             "--start-offset", LaunchConfiguration("start_offset")],
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="hornbill"),
        DeclareLaunchArgument("bag_name"),
        DeclareLaunchArgument("start_offset", default_value="0"),
        DeclareLaunchArgument("viz", default_value="false"),
        DeclareLaunchArgument("config_override", default_value=""),
        core, TimerAction(period=2.0, actions=[play]),
    ])
