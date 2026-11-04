#!/usr/bin/env python3
"""depot-vln 仿真总启动：世界 + 机器人 + 桥接（一条命令）。

  ros2 launch launch/depot_sim.launch.py
  ros2 launch launch/depot_sim.launch.py gui:=true            # 额外起 GUI 客户端
  ros2 launch launch/depot_sim.launch.py bridge_images:=true  # 桥相机图像（P3 用，带宽大）
  ros2 launch launch/depot_sim.launch.py x:=2.0 y:=0.0        # 指定出生点

设计说明
  * 世界只读 world/warehouse.sdf（生成 + 自检过的那一份），不在 launch 里重新生成
  * 相机图像默认不桥：3 路 640×480@15 Hz 会明显压低 RTF（SPEC §11 风险表）
  * 全部 use_sim_time；TF 只有一个源（scripts/odom_tf.py 从 /odom 发）
"""
from __future__ import annotations

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description() -> LaunchDescription:
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    world = os.path.join(ROOT, "world", "warehouse.sdf")
    sdf = os.path.join(ROOT, "robot", "depot_bot.sdf")
    urdf = os.path.join(ROOT, "robot", "depot_bot.urdf")

    gui = LaunchConfiguration("gui")
    software_render = LaunchConfiguration("software_render")
    egl_vendor = LaunchConfiguration("egl_vendor")
    bridge_images = LaunchConfiguration("bridge_images")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # 只留一个 gz 进程：
    #   gui:=true  -> "-r -v 3 <world>"     一个进程同时起 server + GUI
    #   gui:=false -> "-s -r -v 3 <world>"  纯 headless
    # 之前错误地"server 一个 include + 再额外起一个 -g 客户端"，会出现两个 Gazebo 实例、
    # 两个 GUI 抢 EGL，结果话题只建了一半（/lidar3d /imu /joint_states 全没有）。
    server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-r -v 3 ", world], "on_exit_shutdown": "true"}.items(),
        condition=IfCondition(gui),
    )
    server_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-s -r -v 3 ", world], "on_exit_shutdown": "true"}.items(),
        condition=UnlessCondition(gui),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": open(urdf, encoding="utf-8").read(), "use_sim_time": use_sim_time}],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-name", "depot_bot", "-file", sdf, "-x", x, "-y", y, "-z", "0.05"],
    )

    bridge_core = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        # ros_gz_bridge 的 ROS 侧话题名 = 参数里写的那串本身（默认 /model/depot_bot/xxx）。
        # Nav2 / 我们的脚本都认 /cmd_vel、/odom、/joint_states，所以在这里统一 remap，
        # 而不是让每个下游去迁就 gz 的命名。
        # 注意 gz 侧的真名：cmd_vel/odom 在 /model/depot_bot/ 下（插件自己拼的），
        # 而 joint_states 是 JointStatePublisher 里 <topic> 指定的 /joint_states
        # （之前按 /model/depot_bot/joint_states 桥，拿到的是空话题）。
        remappings=[
            ("/model/depot_bot/cmd_vel", "/cmd_vel"),
            # 曾经这里有一条 ("/lidar3d/points", "/lidar3d") 的 remap，起因是当时点云桥不通、
            # 想把点云塞到 /lidar3d 上凑合。它的副作用是致命的：
            #   点云的 ROS 侧被重映射成 /lidar3d(PointCloud2)，和 LaserScan 桥的 /lidar3d 抢同一个
            #   ROS 话题 -> 两种类型冲突 -> 后建的桥直接 "Failed to create a bridge"，
            #   而且 ROS 图里 /lidar3d 会同时列出两种类型（这就是当初排查时看到的怪现象）。
            # 现在两条雷达话题各归各位，不再 remap：
            #   /lidar3d        -> sensor_msgs/LaserScan
            #   /lidar3d/points -> sensor_msgs/PointCloud2
        ],
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/model/depot_bot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            # 注意：**不再**桥 odom。Fortress 6.18 的 MecanumDrive 不在这个话题上发数据
            # （publisher 注册了但永远静默，官方例程也一样），/odom 由 scripts/mecanum_odom.py
            # 从 /joint_states 的四个轮速积分出来。桥一个空的 /odom 只会掩盖问题。
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            # 传感器类必须用 ignition.msgs.* 命名！
            # 装的是 ros_gz_bridge 0.244.26（按 Gazebo Harmonic / gz.msgs.* 构建），
            # 而仿真器是 Fortress / ignition-transport11：
            #   gz.msgs.PointCloudPacked -> 桥"创建成功"但一条不发（静默）
            #   gz.msgs.LaserScan        -> 桥直接报 Failed to create a bridge
            #   ignition.msgs.*          -> 两条都通（实测 10 Hz 仿真频率）
            # Clock/IMU/JointState 两种命名都行，所以以前没暴露。
            "/lidar3d/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            "/lidar3d@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        ],
    )

    # 话题名由 xacro 里 <topic> 显式指定（stereo_left / stereo_right / rgbd），
    # 不依赖 link 结构 —— 因为 `ign sdf -p` 会把 fixed joint 合并进 base_footprint，
    # 之前按 link 名拼路径的写法拿到的是空话题（踩过）。
    # 传感器类消息**必须**写 ignition.msgs.*：本机 ros_gz_bridge 0.244.26 是按 Harmonic
    # (gz.msgs) 构建的，而仿真器是 Fortress(ignition-transport11)。写 gz.msgs.Image /
    # gz.msgs.CameraInfo 会"桥创建成功但一条不发"（静默失败）—— 之前一直拿不到图就是这个原因。
    bridge_image_topics = [
        "/stereo_left@sensor_msgs/msg/Image[ignition.msgs.Image",
        "/stereo_right@sensor_msgs/msg/Image[ignition.msgs.Image",
        "/rgbd/image@sensor_msgs/msg/Image[ignition.msgs.Image",
        "/rgbd/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
        "/rgbd/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
        "/rgbd/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
    ]
    bridge_images_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(bridge_images),
        arguments=bridge_image_topics,
    )

    # 你这台机器的 gz 一直报 `libEGL warning: egl: failed to create dri2 screen`：
    # server 侧的渲染上下文建不起来 -> gpu_lidar / 相机拿不到数据（表面现象是"传感器全哑"）。
    # software_render:=true 用 llvmpipe 兜底，慢但一定能出图；确认数据链路通了再想性能。
    software_render_env = SetEnvironmentVariable(
        name="LIBGL_ALWAYS_SOFTWARE", value="1", condition=IfCondition(software_render)
    )

    # gz 的 Sensors 系统在 server 进程里用 EGL 建渲染上下文。你这台机器（RTX 4080 + X11 :1）
    # 上 glvnd 默认挑到 Mesa，Mesa 建 dri2 screen 失败：
    #   libEGL warning: egl: failed to create dri2 screen
    # 后果不是"画质差"，而是**整个仿真循环被卡住**（server_console.log 里
    # "Waiting for init" 一直等到 Ctrl-C 才打印 "Rendering Thread initialized"）。
    # 把 EGL 厂商强制成 NVIDIA 通常直接解决。
    egl_env_nvidia = SetEnvironmentVariable(
        name="__EGL_VENDOR_LIBRARY_FILENAMES",
        value="/usr/share/glvnd/egl_vendor.d/10_nvidia.json",
        condition=LaunchConfigurationEquals("egl_vendor", "nvidia"),
    )
    egl_env_mesa = SetEnvironmentVariable(
        name="__EGL_VENDOR_LIBRARY_FILENAMES",
        value="/usr/share/glvnd/egl_vendor.d/50_mesa.json",
        condition=LaunchConfigurationEquals("egl_vendor", "mesa"),
    )

    # 里程计 + TF 的唯一来源：从 /joint_states 的四个轮速积分（见脚本头部说明）。
    # odom_tf.py 保留但不再使用 —— 两个节点同时发 odom->base_footprint 会互相打架。
    # S2（EKF 融合）时用 odom_tf:=false：TF odom->base_footprint 交给 robot_localization，
    # 否则两条链同时发同一个变换，slam 会看到位姿来回跳。
    odom_tf = LaunchConfiguration("odom_tf")
    odom = ExecuteProcess(
        cmd=[sys.executable, os.path.join(ROOT, "scripts", "mecanum_odom.py"), "--verbose", "--tf"],
        output="screen",
        condition=IfCondition(odom_tf),
    )
    odom_no_tf = ExecuteProcess(
        cmd=[sys.executable, os.path.join(ROOT, "scripts", "mecanum_odom.py"), "--verbose", "--no-tf"],
        output="screen",
        condition=UnlessCondition(odom_tf),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true", description="起 GUI（false = 纯 headless，RTF 最高）"),
            DeclareLaunchArgument("software_render", default_value="false",
                                  description="LIBGL_ALWAYS_SOFTWARE=1（EGL 建不起来时的兜底）"),
            DeclareLaunchArgument("egl_vendor", default_value="nvidia",
                                  description="强制 EGL 厂商: nvidia / mesa / none"),
            DeclareLaunchArgument("bridge_images", default_value="false", description="桥相机图像到 ROS"),
            DeclareLaunchArgument("odom_tf", default_value="true",
                                  description="mecanum_odom 是否发 TF odom->base_footprint（S2 用 false，交给 EKF）"),
            DeclareLaunchArgument("x", default_value="-31.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            egl_env_nvidia,
            egl_env_mesa,
            software_render_env,
            server,
            server_headless,
            robot_state_publisher,
            TimerAction(period=3.0, actions=[spawn]),
            TimerAction(period=4.0, actions=[bridge_core, odom, odom_no_tf]),
            TimerAction(period=5.0, actions=[bridge_images_node]),
        ]
    )
