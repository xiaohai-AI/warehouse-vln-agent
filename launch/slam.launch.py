#!/usr/bin/env python3
"""S1 纯激光基线：/lidar3d(PointCloud2) -> /scan(2D 切片) -> slam_toolbox。

  ros2 launch launch/slam.launch.py                # 建图
  ros2 launch launch/slam.launch.py use_sim_time:=false

切片高度：取 z∈[0.1, 0.6] 的一层，覆盖货架底层与地面杂物；
这个高度带是"只用激光能看到的语义上限"，也是 P5 里 T1 结构发现的输入。
"""
from __future__ import annotations

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params = LaunchConfiguration("slam_params")
    toolbox = get_package_share_directory("slam_toolbox")

    # SPEC §P4 S1 的原设计：从点云切 z∈[0.1,0.6] 的一层做 2D 激光。
    #
    # 为什么中间要多一个中继：gz 的点云 header.frame_id 是作用域名字
    # "depot_bot/base_footprint/lidar3d"，TF 树里没有这个 frame（我们的是
    # base_footprint -> lidar3d_link，雷达装在 0.55 m）。不重写 frame_id 的话
    # p2l 会一直等 TF，表现为
    #   "Message Filter dropping message: frame 'depot_bot/base_footprint/lidar3d' ... queue is full"
    # 一帧都出不来。
    # 点云桥已通（ros2 topic info /lidar3d/points -> 发布者 1；probe 6 s 收 18 条），
    # 所以走 SPEC §P4 S1 的原设计：点云只改写 frame_id，切片交给 pointcloud_to_laserscan。
    # 比之前用 scan_relay --type ring 从多线 LaserScan 里"抽一圈"干净得多（有高度信息、
    # 不会因为挑错圈而丢柱子/挡板）。
    relay = ExecuteProcess(
        cmd=[sys.executable, os.path.join(ROOT, "scripts", "scan_relay.py"),
             "--type", "cloud",
             "--in-topic", "/lidar3d/points",
             "--out-topic", "/lidar3d/points_tf",
             "--frame-id", "lidar3d_link"],
        output="screen",
    )

    p2l = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "target_frame": "base_footprint",
                "transform_tolerance": 0.1,
                "min_height": 0.10,
                "max_height": 0.60,
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.0087,   # 0.5°
                "scan_time": 0.1,
                "range_min": 0.30,
                "range_max": 25.0,
                "use_inf": True,
                "inf_epsilon": 1.0,
            }
        ],
        # 注意话题类型：ROS 侧 /lidar3d 是 LaserScan（gz 单平面输出），
        # PointCloud2 在 /lidar3d/points —— 这里必须给点云话题，否则 p2l 一条也收不到。
        # 用中继之后的话题（frame_id 已改写成 lidar3d_link）。
        remappings=[("cloud_in", "/lidar3d/points_tf"), ("scan", "/scan")],
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(toolbox, "launch", "online_async_launch.py")),
        launch_arguments={"use_sim_time": use_sim_time, "slam_params_file": slam_params}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("slam_params", default_value=os.path.join(ROOT, "config", "slam_toolbox.yaml")),
            relay,
            # 点云桥恢复后按预留接法启用：/lidar3d/points -> relay(改写 frame) ->
            # /lidar3d/points_tf -> p2l 切 z∈[0.1,0.6] -> /scan -> slam_toolbox
            p2l,
            slam,
        ]
    )
