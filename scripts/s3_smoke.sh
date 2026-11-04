#!/usr/bin/env bash
# S3 冒烟：5 分钟判定"双目视觉里程计到底能不能用"，避免白跑 40 分钟的整条路线。
#
# 它自己负责全套：重建机器人（640×480@10 双目）-> 干净仿真（odom_tf:=false）
#   -> 相机桥（渲染）-> stereo_info（camera_info）-> rtabmap stereo_odometry
#   -> s3_check.py（测频率 + 开 6 m 比尺度）-> 全部收掉。
#
# 前置：已装 rtabmap：  sudo apt install -y ros-humble-rtabmap-ros
# 用法：bash scripts/s3_smoke.sh
#       CHECK=scripts/s3_diag.py bash scripts/s3_smoke.sh   # 换用归因诊断（5 分钟，走 20 m 进巷）
#       ODOM_PARAMS=config/rtabmap_stereo_ff.yaml CHECK=scripts/s3_diag.py \
#         bash scripts/s3_smoke.sh                          # 换 VO 估计器做 A/B（帧到帧+SIFT）
#       EXTRA_NODES='.venv/bin/python scripts/floorflow_odom.py' CHECK=scripts/s4_check.py \
#         bash scripts/s3_smoke.sh                          # S4：地面带光流里程计（make s4-smoke）
set -uo pipefail
cd "$(dirname "$0")/.." || exit 2

ROS=${ROS:-/opt/ros/humble/setup.bash}
PY=${PY:-.venv/bin/python}
mkdir -p results

# 起跑前先清残留：上一轮若有 floorflow / ekf 没被收掉，会和本轮同时发布同一话题，
# 让 EKF 收到互相矛盾的输入（S5 冒烟实测：3 个 ekf + 1 个 floorflow 把输出喂成 NaN）。
for _pat in '[f]loorflow_odom.py' '[e]kf_node'; do
  if pgrep -f "$_pat" >/dev/null 2>&1; then
    echo "[s3-smoke] 清掉上一轮残留的 $_pat"
    pkill -9 -f "$_pat" 2>/dev/null
    sleep 0.5
  fi
done

cleanup() {
  local procs="slam_toolbox scan_relay pointcloud_to_laserscan stereo_odometry stereo_info image_frame_relay parameter_bridge"
  # 附加节点收尾：先按进程组 + PID 杀（wrapper 层数不定，只杀直接子进程会留孤儿），
  # 最后按名字兜底扫一遍。注意**不能在启动阶段按 EXTRA_NODES 的字符串 pkill**：
  # 那个命令行就在跑本脚本的 shell 里，`pkill -f` 会连 wrapper 一起杀掉。
  if [ -n "${EXTRA_PID:-}" ]; then
    # 整组杀（setsid 建立的进程组，组号 = 组长 PID）；再按直接子进程兜底
    kill -9 "-${EXTRA_PID}" 2>/dev/null
    pkill -9 -P "$EXTRA_PID" 2>/dev/null
    kill -9 "$EXTRA_PID" 2>/dev/null
    sleep 0.5
    # 最后验收：EXTRA 里起的节点不许活到下一轮（S5 踩过僵尸污染）
    for pat in '[f]loorflow_odom.py' '[e]kf_node'; do
      pgrep -f "$pat" >/dev/null 2>&1 && { pkill -9 -f "$pat" 2>/dev/null; echo "[s3-smoke] 警告：强制清掉残留 $pat"; }
    done
  fi
  local p
  for p in $procs; do pkill -f "[${p:0:1}]${p:1}" 2>/dev/null; done
  sleep 1
  for p in $procs; do pkill -9 -f "[${p:0:1}]${p:1}" 2>/dev/null; done
  # 冒烟结束就拆仿真：相机在渲染，留着整夜会挂 GPU
  make down > /dev/null 2>&1 || true
  echo "[s3-smoke] 已收尾（含仿真）"
}
trap cleanup EXIT

# 0) rtabmap 在不在
if ! bash -lc "source $ROS && ros2 pkg prefix rtabmap_odom" > /dev/null 2>&1; then
  echo "[s3-smoke] ❌ 没装 rtabmap。先跑：sudo apt install -y ros-humble-rtabmap-ros"
  exit 2
fi
echo "[s3-smoke] rtabmap 已就绪"

# 1) 机器人模型必须是 640×480@10 的双目（slam1_run.sh 的默认 CAM_ARGS 会改回 320×240）
echo "[s3-smoke] 重建机器人模型（S3 相机参数）…"
CAM_W=${CAM_W:-320}; CAM_H=${CAM_H:-240}
# cam_always_on=true 是"左右目同相"的手段，但如果它让相机**每个物理步**都要一帧渲染，
# 渲染线程会被灌满、发布出来的永远是旧帧（实测画面更新率 0.05 Hz）。
# 所以留成可切换的：CAM_ALWAYS_ON=false 用来验证"新鲜度是不是被它拖死的"。
CAM_ALWAYS_ON=${CAM_ALWAYS_ON:-true}
# 320×240 是有依据的：S1 双目深度在 320 档已 PASS（中位 1.99% / p95 9.49%），
# 而渲染成本是像素数 —— 640×480 在 2 台相机下把 RTF 压到 ≈0.2，203 m 路线要跑 1 小时。
make robot CAM_ARGS="cam_width:=$CAM_W cam_height:=$CAM_H cam_rate:=10 enable_rgbd:=false cam_always_on:=$CAM_ALWAYS_ON" \
  > /dev/null 2>&1 \
  || { echo "[s3-smoke] make robot 失败"; exit 2; }

# 2) 干净仿真，odom_tf:=false（视觉里程计独占 odom->base_footprint）
echo "[s3-smoke] 重启仿真…"
make down > /dev/null 2>&1 || true
nohup make sim SIM_ARGS="gui:=false odom_tf:=false" > results/sim_s3_smoke.log 2>&1 &
for _ in $(seq 1 60); do
  pgrep -f "[w]arehouse.sdf" > /dev/null && break
  sleep 2
done
if ! pgrep -f "[w]arehouse.sdf" > /dev/null; then
  echo "[s3-smoke] ❌ 仿真起不来，日志末尾："; tail -12 results/sim_s3_smoke.log; exit 2
fi
# 2b) 先证明**传感器系统本身活着**（点云有数据），再起相机桥。
# 踩过：在传感器子系统还没初始化完时就起桥，gz 侧订阅会悬空 —— 桥建好了、一条不转发，
# 表现成"/stereo_left 无数据"，很容易误判成相机坏了（实际是时序）。
echo "[s3-smoke] 等点云（确认传感器系统已就绪）…"
lidar_ok=0
for i in $(seq 1 6); do
  if bash -lc "source $ROS && timeout 30 $PY scripts/clock_probe.py --topic /lidar3d/points --type sensor_msgs/msg/PointCloud2" 2>&1 | grep -qE "best_effort +[1-9]"; then
    lidar_ok=1; break
  fi
  echo "[s3-smoke]   点云还没有（第 $i/6 次），等 10 s"
  sleep 10
done
[ "$lidar_ok" = "1" ] && echo "[s3-smoke] 传感器系统已就绪" || echo "[s3-smoke] ⚠️ 点云也没有 —— 后面大概率也失败"

# 3) 相机桥（= 渲染开关），等双目出图（最长 160 s：冷启动要初始化 GPU 渲染）
echo "[s3-smoke] 起相机桥…"
nohup bash scripts/cam_bridge.sh > results/s3_smoke_bridge.log 2>&1 &
sleep 10
stereo_ok=0
for i in $(seq 1 8); do
  out=$(bash -lc "source $ROS && timeout 20 $PY scripts/clock_probe.py --topic /stereo_left --type sensor_msgs/msg/Image" 2>&1)
  if echo "$out" | grep -qE "(best_effort|reliable|transient) +[1-9]"; then stereo_ok=1; break; fi
  echo "[s3-smoke]   双目还没有（第 $i/8 次），10 s 后再试"
  # 区分"gz 没发"与"桥没转发"：直接看 gz 侧话题表
  echo "[s3-smoke]   gz 侧话题：$(bash -lc "ign topic -l 2>/dev/null" | grep -E 'stereo|rgbd' | tr '\n' ' ')"
  sleep 10
done
# 3b) 另一条已知失败模式：桥的 gz 侧订阅悬空（起得太早）→ 重启桥一次通常就好
if [ "$stereo_ok" != "1" ]; then
  echo "[s3-smoke] 双目仍无数据 → 重启相机桥再试（gz 侧订阅悬空的常见修法）"
  pkill -f "[p]arameter_bridge" 2>/dev/null; sleep 2
  nohup bash scripts/cam_bridge.sh > results/s3_smoke_bridge.log 2>&1 &
  sleep 10
  for i in $(seq 1 6); do
    out=$(bash -lc "source $ROS && timeout 20 $PY scripts/clock_probe.py --topic /stereo_left --type sensor_msgs/msg/Image" 2>&1)
    if echo "$out" | grep -qE "(best_effort|reliable|transient) +[1-9]"; then stereo_ok=1; break; fi
    echo "[s3-smoke]   重启后仍无（第 $i/6 次）"
    sleep 10
  done
fi
if [ "$stereo_ok" != "1" ]; then
  echo "[s3-smoke] ❌ /stereo_left 无数据。桥日志："
  tail -8 results/s3_smoke_bridge.log
  echo "[s3-smoke]   gz 侧话题：$(bash -lc "ign topic -l 2>/dev/null" | grep -E 'stereo|rgbd' | tr '\n' ' ')"
  echo "[s3-smoke]   gz 进程：$(pgrep -af '[w]arehouse.sdf' | head -1)"
  exit 3
fi
echo "[s3-smoke] /stereo_left 有数据"

# 4) 图像 frame 改写（gz 的 header.frame_id 是作用域名，TF 里没有）+ camera_info + 视觉里程计
echo "[s3-smoke] 起图像 frame 中继…"
nohup bash -lc "source $ROS && $PY scripts/image_frame_relay.py --in-imu /imu --out-imu /imu_tf" \
  > results/s3_smoke_relay.log 2>&1 &
sleep 3
# NO_ODOM=1 跳过 rtabmap（S4 用 floorflow_odom 独占 odom->base_footprint，
# 再起一个 rtabmap 会变成两个 TF 发布者打架）
if [ -n "${NO_ODOM:-}" ]; then
  echo "[s3-smoke] NO_ODOM=1：跳过 camera_info 与 rtabmap（S4 模式）"
else
echo "[s3-smoke] 起 camera_info 与 rtabmap stereo_odometry…"
nohup bash -lc "source $ROS && $PY scripts/stereo_info.py --image-topic /stereo_left_tf --width $CAM_W --height $CAM_H" \
  > results/s3_smoke_info.log 2>&1 &
sleep 3
nohup bash -lc "source $ROS && ros2 run rtabmap_odom stereo_odometry --ros-args \
    --params-file ${ODOM_PARAMS:-config/rtabmap_stereo.yaml} \
    -r left/image_rect:=/stereo_left_tf -r right/image_rect:=/stereo_right_tf \
    -r left/camera_info:=/stereo_left/camera_info -r right/camera_info:=/stereo_right/camera_info \
    -r odom:=/odometry/visual -r odom_info:=/odometry/visual_info -r imu:=/imu_tf" \
    > results/s3_smoke_odom.log 2>&1 &
sleep 12
pub=$(bash -lc "source $ROS && ros2 topic info /odometry/visual 2>/dev/null" | awk '/Publisher count/ {print $3}' | head -1)
echo "[s3-smoke] /odometry/visual 发布者 ${pub:-0}"
if [ -z "${pub:-}" ] || [ "${pub:-0}" -lt 1 ]; then
  echo "[s3-smoke] ⚠️ 视觉里程计还没发 odom（可能仍在同步图像/camera_info），继续等一下"
  sleep 10
fi
fi

# 4b) 可选附加节点（S4 的 floorflow_odom 走这里 —— 不复制启动代码，见技能库"全套栈只写一遍"）
if [ -n "${EXTRA_NODES:-}" ]; then
  echo "[s3-smoke] 起附加节点：$EXTRA_NODES"
  # **必须 setsid**：附加节点若是 wrapper 脚本（nohup bash -> bash -lc -> bash x.sh -> 节点），
# 树会深两层，`pkill -P $EXTRA_PID` 只杀直接子进程，孙进程全部变孤儿继续发布话题 ——
# S5 冒烟连跑三次后系统里堆了 3 个 ekf_node + 1 个 floorflow，互相打架把 EKF 喂成 NaN。
# setsid 让整棵树自成进程组，收尾时用负数 PID 整组杀。
nohup setsid bash -lc "source $ROS && $EXTRA_NODES" > results/s3_smoke_extra.log 2>&1 &
  EXTRA_PID=$!
  sleep 6
  tail -2 results/s3_smoke_extra.log 2>/dev/null | sed 's/^/[s3-smoke]   /'
fi

# 5) 判定
bash -lc "source $ROS && $PY ${CHECK:-scripts/s3_check.py}"
rc=$?
echo
echo "[s3-smoke] rtabmap 日志末尾（排障用）："
tail -12 results/s3_smoke_odom.log
exit $rc
