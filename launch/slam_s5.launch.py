#!/usr/bin/env python3
"""S5 配置：**地面光流 + 轮速 + IMU 的 EKF 融合** -> slam_toolbox。

与 S4 的差别只有一处：odom->base_footprint 不再由 floorflow_odom 独占，
而是由 robot_localization 的 EKF 融合两路来源：

    /odom（轮速 250 Hz，主力）  ┐
    /odometry/floorflow（视觉 10 Hz，校正）├─ EKF ─> odom->base_footprint
    /imu（只取偏航角速度）       ┘
    /lidar3d/points -> relay -> p2l -> /scan -> slam_toolbox -> map

为什么这样做（DECISIONS D13 / S4 的收尾）：S4 的分段还原率已经 98–102%（里程计本身没问题），
离 SPEC 的 0.6×S1=0.214 m 还差 34%，全部来自 map->odom 的修正与漂移。
轮速短时最稳但会随打滑漂，视觉不打滑但会断流 —— 两者互补是工业 AMR 的标准做法。

关键约定：
  * 仿真 `odom_tf:=false`（mecanum_odom 不发 TF）；
  * floorflow_odom 必须 `--publish-tf 0`（TF 归 EKF）。
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
    ekf_params = LaunchConfiguration("ekf_params")
    h_cam = LaunchConfiguration("h_cam")
    h_lat = LaunchConfiguration("h_lat")
    band = LaunchConfiguration("band")
    toolbox = get_package_share_directory("slam_toolbox")

    image_relay = ExecuteProcess(
        cmd=[sys.executable, os.path.join(ROOT, "scripts", "image_frame_relay.py"),
             "--in-left", "/stereo_left", "--out-left", "/stereo_left_tf",
             "--frame-left", "stereo_left_link",
             "--in-right", "/stereo_right", "--out-right", "/stereo_right_tf",
             "--frame-right", "stereo_right_link",
             "--in-imu", "/imu", "--out-imu", "/imu_tf", "--frame-imu", "imu_link"],
        output="screen",
    )

    # 地面光流：**不发 TF**（EKF 独占 odom->base_footprint）
    # FLOORFLOW_NO_LATERAL=1 可关掉侧移通道（隔离"横向漏量"的 A/B 开关，见 DECISIONS D15）
    ff_extra = ["--no-lateral"] if os.environ.get("FLOORFLOW_NO_LATERAL") == "1" else []
    floorflow = ExecuteProcess(
        cmd=[VENV_PY if os.path.exists(VENV_PY) else sys.executable,
             os.path.join(ROOT, "scripts", "floorflow_odom.py"),
             "--h", h_cam, "--h-lat", h_lat, "--band", band, "--publish-tf", "0"] + ff_extra,
        output="screen",
    )

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params, {"use_sim_time": use_sim_time}],
        remappings=[("odometry/filtered", "/odometry/filtered")],
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
            DeclareLaunchArgument("ekf_params",
                                  default_value=os.path.join(ROOT, "config", "ekf_s5.yaml")),
            DeclareLaunchArgument("h_cam", default_value="0.456"),
            DeclareLaunchArgument("h_lat", default_value="0.425"),
            DeclareLaunchArgument("band", default_value="0.40"),
            image_relay,
            floorflow,
            ekf,
            relay,
            p2l,
            slam,
        ]
    )
