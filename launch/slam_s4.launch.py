#!/usr/bin/env python3
"""S4 配置：**地面带光流里程计**（floorflow + IMU）-> slam_toolbox。

与 S3 的结构一样，只把"里程计来源"从 rtabmap 双目换成地面带光流：

    /stereo_left（左下角地面带）+ /imu  ->  floorflow_odom  ->  odom->base_footprint
    /lidar3d/points -> relay(改 frame) -> p2l 切 z∈[0.1,0.6] -> /scan -> slam_toolbox -> map

为什么换这条链（DECISIONS D13）：
  * rtabmap 用全图特征 + 立体视差，画面主体是周期同构的货架 → 沿轴只还原 1.4%（实测）；
  * 地面带 2D 光流不用视差、只用地面的行位移 → 沿轴还原 93.4%（`make s4-floorflow` 实测）。
  两者看的是**同一路图像**，唯一差别是"输入哪些像素、怎么算"。

所以：
  * 仿真必须 `odom_tf:=false`（odom->base_footprint 由 floorflow_odom 独占）；
  * 必须起相机桥（`make cam-bridge`）—— 相机 always_on=false，没人订阅就不渲染；
  * 不需要 camera_info、不需要右目（本配置只用左目地面带）。
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
VENV_PY = os.path.join(ROOT, ".venv", "bin", "python")


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params = LaunchConfiguration("slam_params")
    h_cam = LaunchConfiguration("h_cam")
    h_lat = LaunchConfiguration("h_lat")
    band = LaunchConfiguration("band")
    toolbox = get_package_share_directory("slam_toolbox")

    # 图像/IMU 的 frame_id 改写成 TF 树里的名字（gz 给的是作用域名）
    image_relay = ExecuteProcess(
        cmd=[sys.executable, os.path.join(ROOT, "scripts", "image_frame_relay.py"),
             "--in-left", "/stereo_left", "--out-left", "/stereo_left_tf",
             "--frame-left", "stereo_left_link",
             "--in-right", "/stereo_right", "--out-right", "/stereo_right_tf",
             "--frame-right", "stereo_right_link",
             "--in-imu", "/imu", "--out-imu", "/imu_tf", "--frame-imu", "imu_link"],
        output="screen",
    )

    # 地面带光流里程计：**独占** odom->base_footprint
    floorflow = ExecuteProcess(
        cmd=[VENV_PY if os.path.exists(VENV_PY) else sys.executable,
             os.path.join(ROOT, "scripts", "floorflow_odom.py"),
             "--h", h_cam, "--h-lat", h_lat, "--band", band, "--publish-tf", "1"],
        output="screen",
    )

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
                "angle_increment": 0.0087,
                "scan_time": 0.1,
                "range_min": 0.30,
                "range_max": 25.0,
                "use_inf": True,
                "inf_epsilon": 1.0,
            }
        ],
        remappings=[("cloud_in", "/lidar3d/points_tf"), ("scan", "/scan")],
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(toolbox, "launch", "online_async_launch.py")),
        launch_arguments={"use_sim_time": use_sim_time, "slam_params_file": slam_params}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("slam_params",
                                  default_value=os.path.join(ROOT, "config", "slam_toolbox.yaml")),
            # 标定值：nominal 0.426 m（SDF 0.35 + 轮轴 0.076），
            # 实测 20.01 m 走成 18.68 m（93.4%）→ h_eff = 0.426 × 20.01/18.68 = 0.456 m
            DeclareLaunchArgument("h_cam", default_value="0.456"),
            DeclareLaunchArgument("h_lat", default_value="0.425"),
            DeclareLaunchArgument("band", default_value="0.40"),
            image_relay,
            floorflow,
            relay,
            p2l,
            slam,
        ]
    )
