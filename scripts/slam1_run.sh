#!/usr/bin/env bash
# SLAM-1（P4 · S1 纯激光）一条命令跑完判据：
#   起 slam -> 等 /scan -> 沿固定路线走 ~199 m（记真值 + TF 估计轨迹）-> 存图 -> 算 ATE/地图一致性
#
# 前置：仿真已在跑（gui:=false）。这条判据耗时较长（约 20–25 分钟），过程中会每 10 s 打印进度。
# 参数：SPEED=0.35（巡航速度，打滑随速度超线性，别贸然调高）
set -uo pipefail
cd "$(dirname "$0")/.." || exit 2

ROS=${ROS:-/opt/ros/humble/setup.bash}
PY=${PY:-.venv/bin/python}
SPEED=${SPEED:-0.35}
# CFG=s1 纯激光基线；CFG=s2 轮速+IMU 经 EKF 融合（EKF 独占 odom->base_footprint）
CFG=${CFG:-s1}
if [ "$CFG" = "s2" ]; then
  SIM_ARGS_CFG="gui:=false odom_tf:=false"
  SLAM_LAUNCH="launch/slam_s2.launch.py"
  TRAJ="results/slam1_s2_traj.csv"
  JSON="results/slam1_s2.json"
  MAP_NAME="maps/depot_slam1_s2"
  LABEL="S2 轮速+IMU（robot_localization EKF -> slam_toolbox）"
elif [ "$CFG" = "s3" ]; then
  # S3：双目视觉里程计（rtabmap）独占 odom->base_footprint，所以同样 odom_tf:=false。
  # 需要 640×480@10 的双目 + 相机桥（渲染）+ camera_info，见 launch/slam_s3.launch.py。
  SIM_ARGS_CFG="gui:=false odom_tf:=false"
  SLAM_LAUNCH="launch/slam_s3.launch.py"
  TRAJ="results/slam1_s3_traj.csv"
  JSON="results/slam1_s3.json"
  MAP_NAME="maps/depot_slam1_s3"
  LABEL="S3 双目惯性里程计（rtabmap stereo_odometry -> slam_toolbox）"
elif [ "$CFG" = "s4" ]; then
  # S4：地面带 2D 光流 + IMU 独占 odom->base_footprint（DECISIONS D13）。
  # 与 S3 用同一路图像，只换"取哪些像素、怎么算"。
  SIM_ARGS_CFG="gui:=false odom_tf:=false"
  SLAM_LAUNCH="launch/slam_s4.launch.py"
  TRAJ="results/slam1_s4_traj.csv"
  JSON="results/slam1_s4.json"
  MAP_NAME="maps/depot_slam1_s4"
  LABEL="S4 地面光流里程计（floorflow + IMU -> slam_toolbox）"
elif [ "$CFG" = "s5" ]; then
  # S5：地面光流 + 轮速 + IMU 的 EKF 融合（工业 AMR 的标准架构，DECISIONS D13）
  SIM_ARGS_CFG="gui:=false odom_tf:=false"
  SLAM_LAUNCH="launch/slam_s5.launch.py"
  TRAJ="results/slam1_s5_traj.csv"
  JSON="results/slam1_s5.json"
  MAP_NAME="maps/depot_slam1_s5"
  LABEL="S5 地面光流+轮速 EKF（floorflow + wheel + IMU -> slam_toolbox）"
else
  SIM_ARGS_CFG="gui:=false"
  SLAM_LAUNCH="launch/slam.launch.py"
  TRAJ="results/slam1_traj.csv"
  JSON="results/slam1.json"
  MAP_NAME="maps/depot_slam1"
  LABEL="S1 纯激光（3D 点云 -> p2l 切片 -> slam_toolbox）"
fi
mkdir -p results maps

cleanup() {
  local p
  # S3 还要收：视觉里程计 / camera_info / 相机桥（桥不收就一直渲染，GPU 风险）
  local procs="floorflow_odom ekf_node slam_toolbox scan_relay pointcloud_to_laserscan stereo_odometry stereo_info parameter_bridge"
  for p in $procs; do
    pkill -f "[${p:0:1}]${p:1}" 2>/dev/null
  done
  sleep 1
  for p in $procs; do
    pkill -9 -f "[${p:0:1}]${p:1}" 2>/dev/null
  done
  echo "[slam1] 已收尾（SLAM / relay / p2l / 视觉里程计 / 相机桥）"
}
trap cleanup EXIT

# 判据可复跑的前提：每次从**干净仿真**开始。
# 踩过：旧 sim 连续跑几小时后 RTF 从 0.5 掉到 0.14，224 m 的路线走不完（撞安全阀），
# 而且起点不同会让路线长度在 176–260 m 之间浮动，两次结果不可比。
# 所以这里一律先拆再起：出生点固定 (-31,0) -> 路线固定 203 m。
echo "[slam1] 重建机器人模型（xacro -> sdf + 轮摩擦注入）…"
# 注意：slam1_run.sh 原来一律用默认 CAM_ARGS（320×240@5、cam_pitch=0），
# 会把 S3 需要的 640×480@10 静默改回去 —— 所以 S3 必须显式传。
if [ "$CFG" = "s3" ] || [ "$CFG" = "s4" ] || [ "$CFG" = "s5" ]; then
  # 320×240：S1 双目深度 320 档已 PASS；640×480 双相机把 RTF 压到 ≈0.2，203 m 要跑 1 小时
  make robot CAM_ARGS='cam_width:=320 cam_height:=240 cam_rate:=10 enable_rgbd:=false cam_always_on:=true' \
    > /dev/null 2>&1 \
    || { echo "[slam1] make robot（S3 相机参数）失败"; exit 2; }
else
  make robot > /dev/null 2>&1 || { echo "[slam1] make robot 失败"; exit 2; }
fi

echo "[slam1] 重启仿真（保证 RTF 与起点一致）…"
make down > /dev/null 2>&1 || true
nohup make sim SIM_ARGS="$SIM_ARGS_CFG" > results/sim_run.log 2>&1 &
for _ in $(seq 1 60); do
  pubs=$(bash -lc "source $ROS && ros2 topic info /odom 2>/dev/null" | awk '/Publisher count/ {print $3}' | head -1)
  if [ -n "${pubs:-}" ] && [ "${pubs:-0}" -ge 1 ]; then break; fi
  sleep 3
done
if ! pgrep -f "[w]arehouse.sdf" > /dev/null; then
  echo "[slam1] 仿真起不来，日志末尾："
  tail -15 results/sim_run.log
  exit 2
fi
echo "[slam1] 仿真就绪（/odom 有 ${pubs:-?} 个发布者）"

# 逐级等"有数据"，而不是等"进程起来了"。
# 踩过：/odom 只要关节状态就绪就有，比雷达早很多；新起仿真时传感器系统还在预热，
# 于是 slam 起来了但 /scan 一条没有，脚本误判"链路没通"。
wait_topic() {
  local topic=$1 type=$2 tries=${3:-3} i out n
  for i in $(seq 1 "$tries"); do
    out=$(bash -lc "source $ROS && timeout 40 $PY scripts/clock_probe.py --topic $topic --type $type" 2>&1)
    n=$(echo "$out" | awk '/best_effort/ {print $2}' | head -1)
    if [ -n "${n:-}" ] && [ "${n:-0}" -gt 0 ]; then
      echo "[slam1] $topic 有数据（6 s 收 $n 条）"
      return 0
    fi
    echo "[slam1] $topic 暂无数据（第 $i/$tries 次），等 15 s 再试"
    sleep 15
  done
  echo "[slam1] $topic 始终没数据，最后输出："
  echo "$out" | tail -8
  return 1
}

wait_topic /lidar3d/points sensor_msgs/msg/PointCloud2 4 || exit 3

# S3 额外前置：相机桥 + 双目图像 + IMU。
# 相机在 xacro 里 always_on=false —— 没人订阅就不渲染，所以**桥必须在 launch 之前起**，
# 否则 stereo_odometry 一上来收不到图，rtabmap 会直接退出（"Did not receive data"）。
if [ "$CFG" = "s3" ] || [ "$CFG" = "s4" ] || [ "$CFG" = "s5" ]; then
  echo "[slam1] $CFG：起相机桥（渲染开关）…"
  nohup bash scripts/cam_bridge.sh > results/s3_cam_bridge.log 2>&1 &
  wait_topic /stereo_left sensor_msgs/msg/Image 4 || { echo "[slam1] 双目无数据"; exit 3; }
  wait_topic /stereo_right sensor_msgs/msg/Image 2 || { echo "[slam1] 右目无数据"; exit 3; }
  wait_topic /imu sensor_msgs/msg/Imu 2 || echo "[slam1] 注意：/imu 无数据，航向会漂（$CFG）"
fi

echo "[slam1] 起 slam（日志 results/slam1_run.log）…"
# 参数文件可换：SLAM_PARAMS=config/slam_toolbox_xxx.yaml make slam1-s5
# （同一份轨迹跑 A/B，免得"改了文件忘了哪次是哪组"）
SLAM_PARAMS=${SLAM_PARAMS:-config/slam_toolbox.yaml}
case "$SLAM_PARAMS" in /*) ;; *) SLAM_PARAMS="$PWD/$SLAM_PARAMS" ;; esac
[ -f "$SLAM_PARAMS" ] || { echo "[slam1] slam 参数文件不存在：$SLAM_PARAMS"; exit 4; }
echo "[slam1] slam 参数：$SLAM_PARAMS"
bash -lc "source $ROS && ros2 launch $SLAM_LAUNCH slam_params:=$SLAM_PARAMS" > results/slam1_run.log 2>&1 &
sleep 12
wait_topic /scan sensor_msgs/msg/LaserScan 4 || exit 3

# S4/S5：odom->base_footprint 的唯一发布者分别是 floorflow_odom / EKF，
# 它们没出数就起跑会白跑 25 分钟（slam_toolbox 拿不到 TF）。
if [ "$CFG" = "s4" ]; then
  wait_topic /odometry/floorflow nav_msgs/msg/Odometry 6 \
    || { echo "[slam1] S4 视觉里程计没出数"; exit 3; }
elif [ "$CFG" = "s5" ]; then
  wait_topic /odometry/filtered nav_msgs/msg/Odometry 8 \
    || { echo "[slam1] S5：EKF 融合输出没出数（看 results/slam1_run.log 的 ekf_filter_node）"; exit 3; }
  wait_topic /odometry/floorflow nav_msgs/msg/Odometry 6 \
    || { echo "[slam1] S5：视觉那一路没进 EKF"; exit 3; }
  echo "[slam1] /odometry/filtered 有数据（EKF 融合已就绪）"
fi
echo "[slam1] 开始走固定路线（速度 ${SPEED} m/s）"

start_ts=$(date +%s)
bash -lc "source $ROS && $PY scripts/slam_route.py --speed $SPEED --out $TRAJ"
route_rc=$?
elapsed=$(( $(date +%s) - start_ts ))
echo "[slam1] 路线结束（rc=$route_rc），耗时 ${elapsed} s / 仿真 $(grep -c . "$TRAJ" 2>/dev/null) 帧"

# 路线没跑完就**不许出数**：否则会拿"上一次的 CSV + 这一次只有几秒的图"混出一组假指标
# （踩过：路线 5 s 就崩，指标却打印了 ATE 2.236 m —— 那其实是上一轮的 CSV）。
# 但"跑完了、收尾时崩"要区分开：判据看的是轨迹本身，不是进程退出码。
if [ "$route_rc" != "0" ]; then
  done_m=$(tail -1 "$TRAJ" 2>/dev/null | awk -F, '{print $9}')
  if [ -n "${done_m:-}" ] && awk "BEGIN{exit !(${done_m:-0} >= 190)}"; then
    echo "[slam1] 注意：路线已走完（累计 ${done_m} m），但进程退出码 $route_rc（收尾时崩）—— 继续算指标"
  else
    echo "[slam1] 路线未完成（rc=$route_rc，累计 ${done_m:-0} m），本次不存图、不算指标。日志："
    tail -5 "$TRAJ" 2>/dev/null
    exit "$route_rc"
  fi
fi

echo "[slam1] 存图 -> $MAP_NAME"
bash -lc "source $ROS && ros2 run nav2_map_server map_saver_cli -f $MAP_NAME \
          --ros-args -p use_sim_time:=true" 2>&1 | tail -6

# 先算指标再收尾（指标只读文件，但保留 slam 便于失败时现场复现）
bash -lc "source $ROS && $PY scripts/slam_metrics.py --traj $TRAJ \
          --map $MAP_NAME --out $JSON --label \"$LABEL\""
metrics_rc=$?

$PY - <<EOF
import json
from pathlib import Path
p = Path("$JSON")
if not p.exists():
    print("[slam1] RESULT: FAIL —— 没有 slam1.json")
    raise SystemExit(1)
d = json.loads(p.read_text())
print()
print(f"== SLAM-1（{d['config']}）==")
print(f"  路线       {d['path_length_m']:.1f} m，{d['frames']} 帧（估计有效 {d['frames_with_estimate']}）")
print(f"  ATE RMSE   {d['ate_rmse_m']:.3f} m（阈值 {d['thresholds']['ate_rmse_m']}）  最大 {d['ate_max_m']:.3f} m")
print(f"  终点漂移   {d['final_drift_m']:.3f} m（阈值 {d['thresholds']['final_drift_m']}）"
      f"  dx {d['final_drift_dx']:+.3f} / dy {d['final_drift_dy']:+.3f}")
if d.get("map"):
    print(f"  地图一致   SLAM 占据 {d['map']['cells']} 个点，命中世界几何 {d['map']['match_ratio']*100:.1f}%")
print(f"  RESULT     {d['result']}")
EOF
rc=$?

echo "[slam1] 指标退出码 $metrics_rc / 汇总退出码 $rc"
exit $(( route_rc != 0 || rc != 0 ))
