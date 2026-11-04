#!/usr/bin/env python3
"""Nav2 全向导航栈（MPPI + OmniMotionModel），配合 config/nav2_omni.yaml。

前置：仿真已起（launch/depot_sim.launch.py），且已有 map（slam 跑过或加载静态图）。

  ros2 launch launch/nav2_omni.launch.py
  ros2 launch launch/nav2_omni.launch.py params_file:=config/nav2_omni.yaml
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description() -> LaunchDescription:
    bringup = get_package_share_directory("nav2_bringup")
    params = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "navigation_launch.py")),
        launch_arguments={
            "params_file": params,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "use_composition": "False",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=os.path.join(ROOT, "config", "nav2_omni.yaml")),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            nav,
        ]
    )
