#!/usr/bin/env bash
# 一条命令用 **Nav2** 开过去（在线避障）。没起 Nav2 栈就自动起，跑完自动拆。
#
# 为什么要有：2026-11-03 用手写的 `make drive-to`（开路 cmd_vel，无避障）沿 C 巷
# 中线走，正面撞上静态叉车。Nav2 栈本来就有（D31 已修好全局 costmap），
# 只是没有给它一条"手动开车"的入口 —— 这个脚本就是那个入口。
#
# 分工：
#   make nav-to     ← 一切手动/演示驾驶（本脚本，在线避障）
#   make drive-to   ← 只给脚本做精确停位（开路，无避障，自己过 path_guard 几何预检）
#
# 用法：
#   bash scripts/nav_to.sh --x -10.80 --y -2.15 --yaw 90
#   KEEP=1 bash scripts/nav_to.sh --x -10.80 --y -2.15      # 跑完不拆栈（继续录）
#   BUDGET=300 bash scripts/nav_to.sh ...
set -o pipefail

ROS_SETUP=${ROS_SETUP:-/opt/ros/humble/setup.bash}
PY=${PY:-.venv/bin/python}
KEEP=${KEEP:-0}
BUDGET=${BUDGET:-400}
# 定位方式：truth（默认，真值锚定，仿真里最可靠）/ static（纯 odom）/ amcl
LOC=${LOC:-truth}
NAVPID=""

zero_cmd() {
  # 拆栈时必须把 cmd_vel 归零：mecanum 插件会一直保持最后一条速度指令，
  # 实测拆栈后机器人仍在"滑行"（RTF 1.76% 时看起来像卡住不动，其实还在走）。
  bash -lc "source $ROS_SETUP && timeout 3 ros2 topic pub --once /cmd_vel \
    geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'" \
    >/dev/null 2>&1 || true
}

cleanup() {
  zero_cmd
  if [ "$KEEP" = "1" ]; then
    echo "[nav] KEEP=1 —— 不拆栈（收工自己跑 make down）"
    return 0
  fi
  if [ -n "$NAVPID" ]; then
    echo "[nav] 拆 Nav2 栈（本脚本起的）"
    kill -TERM -"$NAVPID" 2>/dev/null
    sleep 3
    kill -9 -"$NAVPID" 2>/dev/null
  fi
}
trap cleanup EXIT

# ---- 1) 仿真必须已经在跑（与 drive-to 一致：不代你起仿真）
if ! pgrep -f "[i]gn gazebo" >/dev/null 2>&1; then
  echo "[nav] ✗ 仿真没起。先起：make sim SIM_ARGS='gui:=true'"
  exit 4
fi

# ---- 2) Nav2 已经在跑吗？三个节点必须都 active（READY 只保证 bt_navigator）
nav_active() {
  local ns st
  for ns in controller_server planner_server bt_navigator; do
    # **必须 timeout**：planner_server 服务不响应时，ros2 CLI 会永久挂住（2026-11-03 实测，
    # 脚本卡死 6 分钟不报错）。任何 ros2 CLI 调用都不能裸调。
    st=$(timeout 6 bash -lc "source $ROS_SETUP && ros2 lifecycle get /$ns 2>/dev/null" | head -1)
    case "$st" in active*) ;; *) return 1 ;; esac
  done
  return 0
}

if nav_active; then
  echo "[nav] Nav2 已在运行（复用现有栈，不重复起）"
else
  echo "[nav] Nav2 没在跑 → 起栈（静态图 + /scan 中继 + bringup）→ results/nav2_stack.log"
  # STACK_ONLY=1：只起栈并驻留；**不能漏**，否则它会顺带跑 5 轮 M2（实测白等 240 s）
  LOC="$LOC" STACK_ONLY=1 nohup setsid bash scripts/m2_static_run.sh > results/nav2_stack.log 2>&1 &
  NAVPID=$!

  ok=0
  for _ in $(seq 1 120); do
    if grep -q 'READY' results/nav2_stack.log 2>/dev/null; then ok=1; break; fi
    sleep 2
  done
  if [ "$ok" != "1" ]; then
    echo "[nav] ✗ Nav2 栈 240 s 未就绪。日志末尾："
    tail -25 results/nav2_stack.log
    exit 7
  fi
  # RTF 过低（<5%）时 Nav2 的定时器/服务会大面积超时 —— 先查 GPU 是不是没用上
  if pgrep -f "[i]gn gazebo gui" >/dev/null 2>&1; then
    if ! nvidia-smi --query-compute-apps=pid --format=csv,noheader 2>/dev/null | grep -q . ; then
      echo "[nav] ⚠️ nvidia-smi 看不到算图进程 —— GUI 很可能在核显(iris)上软件渲染，"
      echo "[nav]    RTF 会掉到个位数、Nav2 服务大面积超时。用 NVIDIA 起仿真："
      echo "[nav]    __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia make sim SIM_ARGS='gui:=true'"
    fi
  fi

  # lifecycle 是逐个激活的：只等 READY 会发目标被拒收（踩过）
  ok=0
  for i in $(seq 1 45); do
    if nav_active; then ok=1; break; fi
    [ $((i % 5)) -eq 0 ] && echo "[nav] 等三节点 active…（第 ${i} 次 / 45）"
    sleep 2
  done
  if [ "$ok" != "1" ]; then
    echo "[nav] ✗ controller/planner/bt 未全部 active —— 不发目标。各节点实际状态："
    for ns in controller_server planner_server bt_navigator; do
      echo "      /$ns = $(timeout 6 bash -lc "source $ROS_SETUP && ros2 lifecycle get /$ns 2>/dev/null" | head -1)"
    done
    echo "[nav]   排查：RTF 是否过低（ign gazebo gui 是否在用核显软件渲染）、nav2_run.log 有无 Aborting bringup"
    exit 8
  fi
  echo "[nav] Nav2 全部 active"
fi

# ---- 2.5) 自检 map->odom 是否按真值锚定（2026-11-03：写死出生点导致整条路线错位 44 m）
if [ -f results/map_pub.log ]; then
  if grep -q '回退到 --spawn' results/map_pub.log; then
    echo "[nav] ✗✗ map->odom 回退到了固定偏移（真值/odom 读取失败）"
    echo "[nav]    这种情况下导航一定会开错地方 —— 先 make down 重起仿真再跑"
    exit 9
  fi
  line=$(grep -o '按真值算 map->odom = ([^)]*)' results/map_pub.log | tail -1)
  [ -n "$line" ] && echo "[nav] $line"
  read_line=$(grep -o '读数 OK：[^\n]*' results/map_pub.log | tail -1)
  [ -n "$read_line" ] && echo "[nav] $read_line"
  # 偏移离出生点太远 = 仿真被手动开脏了（odom 累积误差），提示但不阻断
  # 与出生点比较：偏移≈出生点 = odom 干净（出生点由 catalog 派生，不再写死字符串）
  if echo "$line" | grep -qE 'map->odom = \(' && awk -v sx="$SPAWN_X" -v line="$line" 'BEGIN{
        match(line, /map->odom = \(([-0-9.]+),/, m);
        exit !(m[1] != "" && (m[1] - sx) < 0.5 && (sx - m[1]) < 0.5)
      }'; then
    echo "[nav] map->odom 与出生点一致（车在出生点、odom 干净）"
  else
    echo "[nav] ⚠️ map->odom 不是出生点值 —— 说明 odom 已被手动驾驶污染（本次已按真值修正）；"
    echo "[nav]    录制/判据前建议 make down 重起仿真，让 odom 从 0 开始，基线才干净。"
  fi
fi

# ---- 3) 发目标（nav_to.py 会再核对一遍前置条件）
# **必须 source ROS**：nav_to.py 里的 rclpy（TruthPoller）与 ros2 CLI 都依赖它，
# 而 .venv/bin/python 不会自动带上 /opt/ros/humble（踩过：FileNotFoundError: 'ros2'）。
# shellcheck disable=SC1090
source "$ROS_SETUP"
"$PY" scripts/nav_to.py --budget "$BUDGET" "$@"
rc=$?
echo "[nav] 退出码 $rc"
exit $rc
