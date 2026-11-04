#!/usr/bin/env bash
# 一条命令跑完 M2（静态世界图 + odom 定位）：清场 -> 起地图 -> 起 Nav2 -> 等 active -> 跑 M2 -> 拆栈
#
# 为什么要这一层：M2 需要三个进程（static_map_pub、nav2 launch、m2_side_dock），
# 手动分别启动容易留下两套重名节点（实测出现过 4 个 bt_navigator/controller_server），
# 它们各按 20 Hz 往 /cmd_vel 发零速，把运动命令对冲掉，落点误差凭空多出 1 m。
# 这里做成一键：起完自检、跑完必拆。
set -uo pipefail
cd "$(dirname "$0")/.." || exit 2

ROS=${ROS:-/opt/ros/humble/setup.bash}
PY=${PY:-.venv/bin/python}
mkdir -p results

kill_nav2() {
  local pats=(nav2_ lifecycle_manager bt_navigator controller_server planner_server \
              smoother_server behavior_server waypoint_follower velocity_smoother)
  local p
  for p in "${pats[@]}"; do pkill -f "[${p:0:1}]${p:1}" 2>/dev/null; done
  sleep 2
  for p in "${pats[@]}"; do pkill -9 -f "[${p:0:1}]${p:1}" 2>/dev/null; done
  sleep 1
}

kill_loc() {
  local pats=(amcl map_server scan_relay static_map_pub truth_map_odom)
  local p
  for p in "${pats[@]}"; do pkill -f "[${p:0:1}]${p:1}" 2>/dev/null; done
  sleep 1
  for p in "${pats[@]}"; do pkill -9 -f "[${p:0:1}]${p:1}" 2>/dev/null; done
  sleep 1
}

kill_all() {
  kill_nav2
  kill_loc
}

cleanup() {
  kill_all
  echo "[m2-static] 已拆除（Nav2 / 地图发布）"
}
trap cleanup EXIT

# 仿真这一层自己带：缺就起一套（make sim 内含 stack_guard + gpu_watchdog），等到 /odom 就绪再继续
ensure_sim() {
  if pgrep -f "[w]arehouse.sdf" > /dev/null; then
    echo "[m2-static] 仿真已在运行"
  else
    echo "[m2-static] 仿真未启动，自动起一套（日志 results/sim_run.log，约 40 s）…"
    nohup make sim SIM_ARGS='gui:=false' > results/sim_run.log 2>&1 &
  fi
  # 注意：不能用 `ros2 topic list | grep /odom` 判就绪 —— 只要有人订阅（Nav2 就订阅）
  # 话题就会出现在列表里，而真正的里程计 mecanum_odom 可能根本没发。必须看发布者数量。
  local i pubs
  for i in $(seq 1 60); do
    pubs=$(bash -lc "source $ROS && ros2 topic info /odom 2>/dev/null" \
           | awk '/Publisher count/ {print $3}' | head -1)
    if [ -n "${pubs:-}" ] && [ "${pubs:-0}" -ge 1 ]; then
      echo "[m2-static] /odom 有 ${pubs} 个发布者，就绪"
      return 0
    fi
    sleep 3
  done
  echo "[m2-static] /odom 180 s 内没有发布者。当前 topic 信息："
  bash -lc "source $ROS && ros2 topic info /odom 2>&1" | head -5 || true
  echo "[m2-static] 仿真日志末尾："
  tail -20 results/sim_run.log
  return 3
}

nav2_count() {
  bash -lc "source $ROS && ros2 node list 2>/dev/null | grep -cE 'bt_navigator|controller_server'" 2>/dev/null
}

echo "[m2-static] 清理旧实例…"
kill_all
cnt=$(nav2_count)
echo "[m2-static] 残留 Nav2 节点数 = ${cnt:-?}（必须 0）"
if [ "${cnt:-1}" != "0" ]; then
  echo "[m2-static] 已存在 Nav2 实例，先处理干净再来"; exit 2
fi

ensure_sim || exit 3

# 地图一致性闸门：地图必须覆盖世界里的所有静态碰撞体，且 M2 的点位可用。
# 花 1 秒，能挡住“地图缺障碍物 -> Nav2 撞进去 -> 看起来像底盘打滑”这一类两天的排查。
if ! "$PY" scripts/map_check.py; then
  echo "[m2-static] map-check FAIL：地图与世界不一致，拒绝开跑（先 make map）"
  exit 4
fi

# Nav2 的局部/全局 costmap 都订 `/scan`（2D ObstacleLayer，见 config/nav2_omni.yaml）。
# `/scan` 由 scan_relay 从多线激光抽一层生成 —— 以前这层没在这里起，靠"上一轮残留在跑"，
# 属于典型隐患（换个干净环境就变成"局部避障静默失效"）。幂等：已有发布者就跳过。
scan_pubs=$(bash -lc "source $ROS && ros2 topic info /scan 2>/dev/null" \
            | awk '/Publisher count/ {print $3}' | head -1)
if [ -z "${scan_pubs:-}" ] || [ "${scan_pubs:-0}" -lt 1 ] 2>/dev/null; then
  echo "[nav2-stack] 起 /scan 中继（日志 results/scan_relay.log）…"
  bash -lc "source $ROS && $PY scripts/scan_relay.py --type ring --in-topic /lidar3d --out-topic /scan --frame-id lidar3d_link" \
    > results/scan_relay.log 2>&1 &
  for _ in $(seq 1 20); do
    scan_pubs=$(bash -lc "source $ROS && ros2 topic info /scan 2>/dev/null" \
                | awk '/Publisher count/ {print $3}' | head -1)
    [ -n "${scan_pubs:-}" ] && [ "${scan_pubs:-0}" -ge 1 ] 2>/dev/null && break
    sleep 1
  done
  if [ -z "${scan_pubs:-}" ] || [ "${scan_pubs:-0}" -lt 1 ] 2>/dev/null; then
    echo "[nav2-stack] /scan 20 s 内没有发布者，costmap 会没有障碍物 → 拒绝继续"
    tail -10 results/scan_relay.log
    exit 5
  fi
fi
echo "[nav2-stack] /scan 有 ${scan_pubs:-?} 个发布者"

# 默认 static（odom 定位）：实测 AMCL 在这条长直巷道里会沿 x 漂 0.1–0.55 m
# （与 SLAM-1 的沿轴滑动同源），误差 0.19–0.59，比纯 odom 更差。LOC=amcl 可手动对照。
LOC=${LOC:-static}
if [ "$LOC" = "amcl" ]; then
  # 绝对定位：静态图 + AMCL(OmniMotionModel) + /scan。没有它，odom 每轮漂 1.5–2 cm，
  # M2 五轮会累积 8–12 cm（实测），判据 0.10 m 时通时不通。
  echo "[m2-static] 启动 AMCL 定位（日志 results/loc_run.log）…"
  bash -lc "source $ROS && ros2 launch launch/localization_amcl.launch.py" > results/loc_run.log 2>&1 &
  loc_ok=0
  for _ in $(seq 1 60); do
    st=$(bash -lc "source $ROS && ros2 lifecycle get /amcl 2>/dev/null" | head -1)
    case "$st" in
      *active*) loc_ok=1; break ;;
    esac
    sleep 2
  done
  if [ "$loc_ok" != "1" ]; then
    echo "[m2-static] AMCL 未进入 active。日志末尾："
    tail -20 results/loc_run.log
    exit 3
  fi
  # AMCL 需要收到 /scan 才会有 map->odom；确认一次再往下走
  sleep 3
  echo "[m2-static] AMCL active，map_server 已加载静态图"
elif [ "$LOC" = "truth" ]; then
  # 真值锚定（2026-11-03）：纯 odom 在 20 m 内漂 15.6°/1.1 m → 车贴货架面滑行、开出 16 m 过头。
  # 这里静态图只发 /map（--no-tf），map->odom 交给 truth_map_odom 以 ~2 Hz 按真值校正。
  # 定位精度本身仍由 S5/EKF 与 AMCL 单独评估；这条链路只保证"驾驶+语义+读编号"有位姿可用。
  echo "[m2-static] 真值锚定定位：静态图(--no-tf) + truth_map_odom（2 Hz 校正）…"
  bash -lc "source $ROS && $PY scripts/static_map_pub.py --no-tf" > results/map_pub.log 2>&1 &
  sleep 3
  bash -lc "source $ROS && $PY scripts/truth_map_odom.py" > results/map_odom.log 2>&1 &
  sleep 2
else
  echo "[m2-static] 启动静态图 + 固定 map->odom（LOC=static）…"
  bash -lc "source $ROS && $PY scripts/static_map_pub.py" > results/map_pub.log 2>&1 &
  sleep 3
fi

# Nav2 bringup 会偶发被 rmw 服务超时打断：
#   failed to send response to /planner_server/get_state (timeout) -> Aborting bringup
# 这是 rmw 侧的偶发问题（不是配置错），实测重起一次即通过。这里自动重试最多 3 次，
# 每次只重启 Nav2（不动仿真和地图发布）。日志里出现 "Aborting bringup" 就立刻判失败，
# 不必傻等到 180 s。
start_nav2() {
  echo "[m2-static] 启动 Nav2（日志 results/nav2_run.log）…"
  bash -lc "source $ROS && ros2 launch launch/nav2_omni.launch.py" > results/nav2_run.log 2>&1 &
  local st
  for _ in $(seq 1 90); do
    st=$(bash -lc "source $ROS && ros2 lifecycle get /bt_navigator 2>/dev/null" | head -1)
    case "$st" in
      *active*) return 0 ;;
    esac
    if grep -q "Aborting bringup" results/nav2_run.log 2>/dev/null; then
      return 3
    fi
    sleep 2
  done
  return 3
}

attempt=0
while true; do
  if start_nav2; then
    echo "[m2-static] bt_navigator active，开始 M2（第 $((attempt + 1)) 次尝试）"
    break
  fi
  attempt=$((attempt + 1))
  if [ "$attempt" -ge 3 ]; then
    echo "[m2-static] Nav2 bringup 连续 3 次失败。日志末尾："
    tail -15 results/nav2_run.log
    exit 3
  fi
  echo "[m2-static] Nav2 bringup 失败（$(grep -c 'Aborting bringup' results/nav2_run.log 2>/dev/null || echo 0) 处中止），"
  echo "[m2-static] 杀干净重试（第 $((attempt + 1))/3 次）…"
  kill_nav2
  sleep 3
done
sleep 2

# STACK_ONLY=1：只把栈起来并**驻留**（给 p6-demo 之类的外部栈复用），不跑 M2。
# 退出时 EXIT trap 会拆干净；外部只要 kill 这个进程即可。
if [ "${STACK_ONLY:-0}" = "1" ]; then
  echo "[nav2-stack] READY（Nav2 active，驻留中；pkill 掉本进程即拆栈）"
  while true; do sleep 2; done
fi

bash -lc "source $ROS && $PY scripts/m2_side_dock.py" 2>&1 | tee results/m2_static.log
rc=${PIPESTATUS[0]}
echo "[m2-static] m2_side_dock 退出码 $rc"
exit "$rc"
