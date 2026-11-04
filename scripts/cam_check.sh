#!/usr/bin/env bash
# 相机链路体检：gz 侧有没有该传感器、ROS 侧桥有没有转发、三路各收多少包。
# 用法：先 make sim ...，再另开一个终端 make cam-bridge，然后 make cam-check
set -uo pipefail
cd "$(dirname "$0")/.."
ROS=${ROS:-/opt/ros/humble/setup.bash}
PY=${PY:-.venv/bin/python}

echo "[cam-check] gz 侧话题（应看到 stereo_left / stereo_right / rgbd 系列）："
ign topic -l 2>/dev/null | grep -iE "stereo|rgbd" | sort || echo "  （一个都没有 —— 仿真没起，或 xacro 没生成相机）"

for spec in "/stereo_left:sensor_msgs/msg/Image" \
            "/stereo_right:sensor_msgs/msg/Image" \
            "/rgbd/image:sensor_msgs/msg/Image" \
            "/rgbd/depth_image:sensor_msgs/msg/Image"; do
  topic=${spec%%:*}; type=${spec##*:}
  echo
  echo "===== $topic ====="
  bash -lc "source $ROS && timeout 40 $PY scripts/clock_probe.py --topic $topic --type $type" 2>&1 \
    | grep -vE "^\[WARN\].*incompatible QoS" | tail -8
done

echo
echo "[cam-check] 判定：每路 best_effort 或 reliable 应 > 0；全 0 说明桥没转发"
echo "[cam-check] 若这里是 0 但 gz 话题存在 -> 桥的类型写错（必须 ignition.msgs.*）"
