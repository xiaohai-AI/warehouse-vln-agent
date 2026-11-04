#!/usr/bin/env bash
# 相机桥：把 gz 的三路相机桥到 ROS。**单独起，不重启仿真**。
#
# 为什么单独起：相机在 xacro 里是 `always_on=false` —— gz 只在**有人订阅**时才渲染。
# 所以这个桥在跑 = 在渲染，Ctrl-C 掉 = 立刻停渲染（零开销）。这是 GPU 安全的关键开关。
#
# 类型必须 ignition.msgs.*（本机 ros_gz_bridge 0.244.26 是按 Harmonic/gz.msgs 构建的，
# 写 gz.msgs.Image 会"桥创建成功但一条不发"，静默失败）。
# 注意：不能 `set -u` —— ROS 的 setup.bash 内部会引用未定义变量（AMENT_TRACE_SETUP_FILES），
# 开了 nounset 会在 source 阶段就退出（报"未绑定的变量"）。
set -o pipefail
ROS=${ROS:-/opt/ros/humble/setup.bash}
# shellcheck disable=SC1090
source "$ROS"

exec ros2 run ros_gz_bridge parameter_bridge \
  "/stereo_left@sensor_msgs/msg/Image[ignition.msgs.Image" \
  "/stereo_right@sensor_msgs/msg/Image[ignition.msgs.Image" \
  "/rgbd/image@sensor_msgs/msg/Image[ignition.msgs.Image" \
  "/rgbd/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image" \
  "/rgbd/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo" \
  "/rgbd/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked"
