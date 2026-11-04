#!/bin/bash
# S5 冒烟用的附加节点组合：地面光流（不发 TF） + EKF 融合（独占 odom->base_footprint）
#
# **为什么要有这个 wrapper**：`make s5-smoke` 早期把两条命令直接塞进 EXTRA_NODES
# （`.venv/... floorflow_odom.py & ros2 run ...`），结果日志里只有
#     `bash: 行 1: ros2: 未找到命令`
# 原因不是没装 robot_localization（装了），而是 shell 优先级：
#     `source $ROS && A & B`  ==  `(source $ROS && A) & B`
# —— `&&` 比 `&` 紧，B 在 source **之外**的前台跑，ROS 环境根本没源上。
# 用脚本文件就不存在这个问题：父 shell 已 `source $ROS`，子进程直接继承环境。
set -o pipefail

cd "$(dirname "$0")/.." || exit 1

# 视觉里程计：--publish-tf 0，TF 交给 EKF（两个发布者会打架）
.venv/bin/python scripts/floorflow_odom.py --publish-tf 0 &
FF=$!

cleanup() { kill -9 "$FF" 2>/dev/null; }
trap cleanup EXIT TERM INT

# EKF：前台阻塞，直到被 smoke 脚本按 PID 杀掉
ros2 run robot_localization ekf_node --ros-args \
  --params-file config/ekf_s5.yaml \
  -p use_sim_time:=true
