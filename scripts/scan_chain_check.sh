#!/usr/bin/env bash
# 一条命令验证扫描链：/lidar3d/points -> relay(改写 frame) -> p2l 切片 -> /scan
#
# 背景：点云桥（/lidar3d/points）之前"桥建成功但 0 发布者"，S1 只能用 scan_relay 从多线
# LaserScan 里抽单圈顶上（沿长直巷道还丢特征）。点云桥修好后恢复 SPEC §P4 S1 的原设计，
# 这个脚本用来一次性确认整条链有没有断点，并给出断在哪一环。
set -uo pipefail
cd "$(dirname "$0")/.." || exit 2

ROS=${ROS:-/opt/ros/humble/setup.bash}
PY=${PY:-.venv/bin/python}
mkdir -p results

cleanup() {
  local p
  for p in slam_toolbox scan_relay pointcloud_to_laserscan; do
    pkill -f "[${p:0:1}]${p:1}" 2>/dev/null
  done
  sleep 1
  for p in slam_toolbox scan_relay pointcloud_to_laserscan; do
    pkill -9 -f "[${p:0:1}]${p:1}" 2>/dev/null
  done
  echo "[scan-check] 已收尾（SLAM / relay / p2l）"
}
trap cleanup EXIT

if ! pgrep -f "[w]arehouse.sdf" > /dev/null; then
  echo "[scan-check] 仿真没在跑：先 make sim SIM_ARGS='gui:=false'（或 make m2-static 会自带）"
  exit 2
fi

echo "[scan-check] 起 slam.launch.py（日志 results/slam_chain.log）…"
bash -lc "source $ROS && ros2 launch launch/slam.launch.py" > results/slam_chain.log 2>&1 &
sleep 12

fail=0
for spec in "/lidar3d/points:sensor_msgs/msg/PointCloud2" \
            "/lidar3d/points_tf:sensor_msgs/msg/PointCloud2" \
            "/scan:sensor_msgs/msg/LaserScan"; do
  topic=${spec%%:*}
  type=${spec##*:}
  echo
  echo "===== $topic ====="
  out=$(bash -lc "source $ROS && timeout 40 $PY scripts/clock_probe.py --topic $topic --type $type" 2>&1)
  echo "$out"
  best=$(echo "$out" | awk '/best_effort/ {print $2}' | head -1)
  if [ -z "${best:-}" ] || [ "${best:-0}" -le 0 ]; then
    echo "[scan-check] FAIL $topic 没有数据"
    fail=1
  fi
done

if grep -q "Message Filter dropping" results/slam_chain.log; then
  echo "[scan-check] 注意：slam 日志里有 TF 丢帧（Message Filter dropping），说明 frame_id 没对齐："
  grep -m3 "Message Filter dropping" results/slam_chain.log
  fail=1
fi

if [ "$fail" = "0" ]; then
  echo
  echo "RESULT: PASS  —— 点云链通，/scan 由 pointcloud_to_laserscan 产出"
else
  echo
  echo "RESULT: FAIL  —— 看上面哪一环没有数据（relay / p2l / TF）"
fi
exit "$fail"
