#!/usr/bin/env python3
"""S3 配置：双目视觉里程计（rtabmap stereo_odometry）-> slam_toolbox。

与 S2（launch/slam_s2.launch.py）的结构完全一样，只把"里程计来源"从
robot_localization EKF 换成 rtabmap 的双目里程计：

    /stereo_left + /stereo_right + camera_info  ->  stereo_odometry  ->  odom->base_footprint
    /lidar3d/points -> relay(改 frame) -> p2l 切 z∈[0.1,0.6] -> /scan -> slam_toolbox -> map

所以：
  * 仿真必须 `odom_tf:=false`（否则 mecanum_odom 和视觉里程计同时发 odom->base_footprint）；
  * 必须同时起相机桥（`make cam-bridge`）—— 相机在 xacro 里 always_on=false，
    没人订阅就不渲染，视觉里程计就没有图；这也是 GPU 开销的来源；
  * camera_info 由 `scripts/stereo_info.py` 发（仿真只桥了 Image）。

判据（SPEC §6 SLAM-1）：S3 的 ATE 相对 S1 降低 ≥40%，或回环残差 <0.3 m；
不达标就在报告里写清"视觉在哪里失败"，不许删掉 S3。
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
    cam_w = LaunchConfiguration("cam_width")
    cam_h = LaunchConfiguration("cam_height")
    slam_params = LaunchConfiguration("slam_params")
    odom_params = LaunchConfiguration("odom_params")
    toolbox = get_package_share_directory("slam_toolbox")

    # 图像 frame 改写：gz 的 header.frame_id 是作用域名（depot_bot/base_footprint/stereo_left），
    # TF 树里没有 -> rtabmap 的 getTransform 全失败、一条 odom 不出。见 image_frame_relay.py。
    image_relay = ExecuteProcess(
        cmd=[sys.executable, os.path.join(ROOT, "scripts", "image_frame_relay.py"),
             "--in-left", "/stereo_left", "--out-left", "/stereo_left_tf",
             "--frame-left", "stereo_left_link",
             "--in-right", "/stereo_right", "--out-right", "/stereo_right_tf",
             "--frame-right", "stereo_right_link",
             "--in-imu", "/imu", "--out-imu", "/imu_tf", "--frame-imu", "imu_link"],
        output="screen",
    )

    # camera_info：仿真只桥了 Image；帧戳必须跟（改写后的）图像一致，否则 approx_sync 永远配不上
    stereo_info = ExecuteProcess(
        cmd=[sys.executable, os.path.join(ROOT, "scripts", "stereo_info.py"),
             "--image-topic", "/stereo_left_tf",
             "--width", cam_w, "--height", cam_h],
        output="screen",
    )

    # 双目里程计。话题名按 rtabmap_odom 的约定 remap。
    # frame_id=base_footprint：它内部用 TF 把相机位姿转到车体，再发 odom->base_footprint。
    # 注意不能用 odom_frame_id=map —— map->odom 由 slam_toolbox 负责，抢了会打架。
    stereo_odom = Node(
        package="rtabmap_odom",
        executable="stereo_odometry",
        name="stereo_odometry",
        output="screen",
        parameters=[odom_params, {"use_sim_time": use_sim_time}],
        remappings=[
            ("left/image_rect", "/stereo_left_tf"),
            ("right/image_rect", "/stereo_right_tf"),
            ("left/camera_info", "/stereo_left/camera_info"),
            ("right/camera_info", "/stereo_right/camera_info"),
            ("imu", "/imu_tf"),
            # **不要**发成 /odom：mecanum_odom 以 ~250 Hz 一直在发 /odom（odom_tf:=false 只是
            # 不发 TF，照样发消息）。两个发布者混在一个话题上，冒烟检查会读到轮式里程计，
            # 数值完全对不上（实测轮式 36.9 m vs 真值 2.0 m，因为车当时顶着货架空转）。
            ("odom", "/odometry/visual"),
            ("odom_info", "/odometry/visual_info"),
        ],
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
            # 必须与 make robot 的 cam_width/cam_height 一致，否则内参错、深度全错
            DeclareLaunchArgument("cam_width", default_value="320"),
            DeclareLaunchArgument("cam_height", default_value="240"),
            DeclareLaunchArgument("slam_params",
                                  default_value=os.path.join(ROOT, "config", "slam_toolbox.yaml")),
            DeclareLaunchArgument("odom_params",
                                  default_value=os.path.join(ROOT, "config", "rtabmap_stereo.yaml")),
            image_relay,
            stereo_info,
            stereo_odom,
            relay,
            p2l,
            slam,
        ]
    )
