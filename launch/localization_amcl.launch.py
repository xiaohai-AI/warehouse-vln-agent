#!/usr/bin/env python3
"""绝对定位：静态世界图 + AMCL（全向）+ /scan 抽环。

替代 `scripts/static_map_pub.py` 的"map->odom 固定常量"模式：
那个模式没有绝对位姿来源，轮式里程计每轮漂 1.5–2 cm，M2 五轮下来累积 8–12 cm。

  ros2 launch launch/localization_amcl.launch.py                      # 默认出生点 (-31, 0, 0)
  ros2 launch launch/localization_amcl.launch.py x0:=-20.4 y0:=-2.15 yaw0:=0.0

提供：/map（transient local）、AMCL 的 map->odom、/amcl_pose
依赖：/scan（本 launch 自己起 scan_relay 从 /lidar3d 抽水平环 -> lidar3d_link）
"""
from __future__ import annotations

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description() -> LaunchDescription:
    map_yaml = LaunchConfiguration("map")
    params = LaunchConfiguration("params")
    use_sim_time = LaunchConfiguration("use_sim_time")
    x0 = LaunchConfiguration("x0")
    y0 = LaunchConfiguration("y0")
    yaw0 = LaunchConfiguration("yaw0")

    # 多线激光在 LaserScan 里是拍平的一维 -> relay 切成单圈，frame 改成 TF 认得的 lidar3d_link
    relay = ExecuteProcess(
        cmd=[sys.executable, os.path.join(ROOT, "scripts", "scan_relay.py"),
             "--type", "ring",
             "--in-topic", "/lidar3d",
             "--out-topic", "/scan",
             "--frame-id", "lidar3d_link"],
        output="screen",
    )

    map_server = Node(
        package="nav2_map_server", executable="map_server", name="map_server",
        output="screen",
        parameters=[params, {"yaml_filename": map_yaml, "use_sim_time": use_sim_time}],
    )

    # initial_pose 用参数覆盖，这样出生点可以由命令行给
    amcl = Node(
        package="nav2_amcl", executable="amcl", name="amcl",
        output="screen",
        parameters=[params, {
            "use_sim_time": use_sim_time,
            "initial_pose": {"x": x0, "y": y0, "z": 0.0, "yaw": yaw0},
        }],
    )

    lifecycle = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_localization", output="screen",
        parameters=[{"use_sim_time": use_sim_time, "autostart": True,
                     "bond_timeout": 0.0,
                     "node_names": ["map_server", "amcl"]}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=os.path.join(ROOT, "maps", "depot_static.yaml")),
        DeclareLaunchArgument("params", default_value=os.path.join(ROOT, "config", "amcl_omni.yaml")),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("x0", default_value="-31.0", description="出生点 x（世界系）"),
        DeclareLaunchArgument("y0", default_value="0.0"),
        DeclareLaunchArgument("yaw0", default_value="0.0"),
        relay,
        map_server,
        amcl,
        lifecycle,
    ])
