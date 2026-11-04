#!/usr/bin/env bash
# P6-2：一条自然语言指令端到端（起仿真 + 相机桥 → 跑 p6_demo.py → 拆栈）
#
# 用法
#   bash scripts/p6_demo.sh                                  # 默认指令（UTT 环境变量可改）
#   UTT="去 10 号位" ANSWER="C 巷右边第一层" bash scripts/p6_demo.sh
#   DRY=1 bash scripts/p6_demo.sh                             # 只走 resolve/plan，不动机器人（不用仿真）
#   P6_ARGS="--no-verify" bash scripts/p6_demo.sh              # 跳过 VLM 复核（先验通路）
#
# 为什么要有这一层：agent 侧不启动 ROS（铁律），但一条指令的端到端需要
#   sim + 相机桥 + demo 三者同时在线 —— 把编排写成脚本，Dan 只跑一条命令。
set -o pipefail
cd "$(dirname "$0")/.." || exit 1
ROOT=$(pwd)
PY=$ROOT/.venv/bin/python
UTT=${UTT:-去 C-L-21-1}
ANSWER=${ANSWER:-}
DRY=${DRY:-0}

# 默认值要在 DRY 分支**之前**给：dry 路径也会用到 $VERIFY / $POSE（否则传空串给 argparse
# 会报 `--verify: invalid choice: ''`，实测踩过一次）。
VERIFY=${VERIFY:-vlm}
POSE=${POSE:-truth}
NAV=${NAV:-0}
if [ "$NAV" = "1" ]; then ACT_BACKEND=nav2; else ACT_BACKEND=${ACT_BACKEND:-drive_to}; fi

# ---------------------------------------------------------------- dry：完全离线
if [ "$DRY" = "1" ]; then
  echo "[p6] DRY 模式：只走 resolve/plan，不动机器人"
  PYTHONNOUSERSITE=1 PYTHONPATH= $PY scripts/p6_demo.py --dry --utterance "$UTT" \
    --answer "$ANSWER" ${BLOCK:+--block "$BLOCK"} ${SUBSTITUTE:+--substitute} \
    ${NOLANE:+--no-lane-offset} --verify "$VERIFY" --pose-source "$POSE" \
    --act-backend "$ACT_BACKEND" "$@"
  exit $?
fi

# ---------------------------------------------------------------- ROS 环境
if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

GUARD=${GUARD:-0}
VERIFY=${VERIFY:-vlm}
POSE=${POSE:-truth}
# 动态障碍（运行时注入的推车）：DYN=1 时在机器人开到横穿点附近时横穿行驶线，
# 用来测前向安全层的**真阳性**。世界 SDF 零改动（见 scripts/move_obstacle.py 文件头）。
DYN=${DYN:-0}
DYN_X=${DYN_X:--16.0}
DYN_AISLE=${DYN_AISLE:-C}
DYN_HOLD=${DYN_HOLD:-4.0}
# Nav2 后端：NAV=1 时起 Nav2 栈（静态图 + /scan 中继 + nav2 bringup），act 改吃 Nav2 单目标。
# 意义：动态障碍从"停/等"升级成"绕行"（代价是 bringup 复杂、与安全层/控制权共存要小心）。
NAV=${NAV:-0}
if [ "$NAV" = "1" ]; then ACT_BACKEND=nav2; else ACT_BACKEND=${ACT_BACKEND:-drive_to}; fi
# GUI=1 → 仿真带 Gazebo 图形界面（**录制演示用**）。默认 headless：无显示器、且 GUI 会分走
#   渲染算力，把 RTF 从 0.44 拉到 0.25~0.35，判据的时间预算会被顶到边界。
GUI=${GUI:-0}
if [ "$GUI" = "1" ]; then SIM_GUI="gui:=true"; else SIM_GUI="gui:=false"; fi
# 预算按 RTF 折算墙钟：开 GUI 时显式传小一点（如 RTF=0.30），否则航段可能超时。
RTF=${RTF:-0.44}
# KEEP=1 → 跑完**不拆栈**（留着继续录；收工自己跑 make down）。
KEEP=${KEEP:-0}
CAMPID=""
SIMPID=""
GUARDPID=""
DYNPID=""
NAVPID=""
cleanup() {
  if [ "${KEEP:-0}" = "1" ]; then
    echo "[p6] KEEP=1 —— **不拆栈**（仿真 + 相机桥保持在线，可以继续录制）"
    echo "[p6]   收工务必手动： bash scripts/stack_down.sh   （或 make down）"
    return 0
  fi
  echo "[p6] 收尾（Nav2 栈 / 动态障碍 / 安全层 / 相机桥 / 仿真 / 整套栈）"
  # Nav2 栈用 SIGTERM 让它自己的 EXIT trap 跑（拆 nav2/scan_relay/static_map_pub），再兜底 KILL
  if [ -n "$NAVPID" ]; then
    kill -TERM -"$NAVPID" 2>/dev/null
    sleep 3
    kill -9 -"$NAVPID" 2>/dev/null
  fi
  [ -n "$DYNPID" ] && kill -9 -"$DYNPID" 2>/dev/null
  if [ "$DYN" = "1" ] && [ -f results/p6_dyn.log ]; then
    echo "[p6] 动态障碍时间线（results/p6_dyn.log）："
    sed 's/^/      /' results/p6_dyn.log
  fi
  [ -n "$GUARDPID" ] && kill -9 -"$GUARDPID" 2>/dev/null
  if [ "$GUARD" = "1" ] && [ -f results/p6_guard.log ]; then
    n=$(grep -c '\[GUARD\] 前方' results/p6_guard.log 2>/dev/null; true)
    echo "[p6] 安全层压住次数：${n:-0}（日志 results/p6_guard.log）"
  fi
  [ -n "$CAMPID" ] && kill -9 -"$CAMPID" 2>/dev/null
  [ -n "$SIMPID" ] && kill -9 -"$SIMPID" 2>/dev/null
  bash scripts/stack_down.sh >/dev/null 2>&1 || true
  n=$(pgrep -cf 'ign gazebo|mecanum_odom|floorflow|ekf_node|slam_toolbox' 2>/dev/null || true)
  echo "[p6] 残留进程数（含本脚本自身匹配，应为 0）：${n:-0}"
}
trap cleanup EXIT

# ---------------------------------------------------------------- 1. 清场 + 建模型
echo "[p6] 清场"
bash scripts/stack_down.sh >/dev/null 2>&1 || true
# `CAM_PITCH` 只用于**故意**制造俯仰误差做实验（默认 0）。正=仰角、负=俯角（见 xacro
# `cam_pitch_rad = -cam_pitch*pi/180`）。投影模型始终假设 pitch=0 —— 这正是"标定误差"。
CAM_PITCH=${CAM_PITCH:-0}
echo "[p6] 重建机器人模型（S3/S5 相机档 320x240@10，cam_pitch=${CAM_PITCH}°）"
make robot CAM_ARGS="cam_width:=320 cam_height:=240 cam_rate:=5 cam_pitch:=${CAM_PITCH}" >/dev/null \
  || { echo "[p6] make robot 失败"; exit 1; }
export CAM_PITCH_DEG="$CAM_PITCH"

# ---------------------------------------------------------------- 2. 起仿真
if [ "$GUI" = "1" ]; then
  echo "[p6] 起仿真（**Gazebo GUI**，DISPLAY=${DISPLAY:-<空>}）→ results/p6_sim.log"
  echo "[p6]   注意：GUI 会分走渲染算力 → RTF 下降、墙钟变长；预算按 RTF=$RTF 折算"
  [ -z "${DISPLAY:-}" ] && { echo "[p6] 警告：DISPLAY 为空，GUI 起不来（先确认在图形会话里）"; }
else
  echo "[p6] 起仿真（headless）→ results/p6_sim.log"
fi
mkdir -p results
nohup setsid make sim SIM_ARGS="$SIM_GUI" > results/p6_sim.log 2>&1 &
SIMPID=$!
# **不要用 `grep -q` 判就绪**：本脚本开了 pipefail，grep 命中即退出会让上游 ros2 收到
# SIGPIPE，整条管道返回非零 → if 永远为假（踩过：仿真其实已起，脚本白等 120 s）。
# 用 slam1_run.sh 已验证的 awk 取 Publisher count。
ROS=${ROS:-/opt/ros/humble/setup.bash}
pubs=0
for i in $(seq 1 40); do
  pubs=$(bash -lc "source $ROS && ros2 topic info /odom 2>/dev/null" \
         | awk '/Publisher count/ {print $3}' | head -1)
  if [ -n "${pubs:-}" ] && [ "${pubs:-0}" -gt 0 ] 2>/dev/null; then
    echo "[p6] 仿真就绪（/odom 有 $pubs 个发布者，等了 $((i * 3)) s）"
    break
  fi
  sleep 3
  if [ "$i" = "40" ]; then
    echo "[p6] 仿真 120 s 未就绪，放弃。话题现状："
    bash -lc "source $ROS && ros2 topic list" 2>/dev/null | grep -E 'odom|stereo|scan' || true
    tail -15 results/p6_sim.log
    exit 1
  fi
done

# ---------------------------------------------------------------- 3. 起相机桥
echo "[p6] 起相机桥（渲染开关）→ results/p6_cam.log"
nohup setsid make cam-bridge > results/p6_cam.log 2>&1 &
CAMPID=$!
cpub=0
for i in $(seq 1 30); do
  cpub=$(bash -lc "source $ROS && ros2 topic info /stereo_left 2>/dev/null" \
         | awk '/Publisher count/ {print $3}' | head -1)
  if [ -n "${cpub:-}" ] && [ "${cpub:-0}" -gt 0 ] 2>/dev/null; then
    echo "[p6] 相机就绪（/stereo_left 有 $cpub 个发布者，等了 $((i * 2)) s）"
    break
  fi
  sleep 2
  if [ "$i" = "30" ]; then
    echo "[p6] 相机 60 s 未就绪，放弃。相机桥日志："
    tail -15 results/p6_cam.log
    exit 1
  fi
done
sleep 3

# ---------------------------------------------------------------- 4. 跑 demo
# **不要清 PYTHONPATH**：rclpy / cv_bridge 都在 `setup.bash` 加进 PYTHONPATH 的
# `/opt/ros/humble/lib/python3.10/site-packages` 里，清掉就是 `No module named 'rclpy'`。
# 先自检一次，别让它在开了 50 m 之后才炸。
echo "[p6] 自检：$PY 能不能 import rclpy / cv2"
if ! $PY -c 'import rclpy, cv2, numpy' 2>/dev/null; then
  echo "[p6] 失败：venv 里的 python 看不到 rclpy（PYTHONPATH 没带上 ROS）。"
  echo "      期望：PYTHONPATH 含 /opt/ros/humble/lib/python3.10/site-packages"
  echo "      现状：PYTHONPATH=${PYTHONPATH:-<空>}"
  exit 1
fi

if [ "$GUARD" = "1" ]; then
  echo "[p6] 起前向安全层 → results/p6_guard.log"
  nohup setsid bash -lc "source $ROS && $PY scripts/forward_guard.py" > results/p6_guard.log 2>&1 &
  GUARDPID=$!
  sleep 3
  # **活性检查**：日志里没有"就绪"就是没起来。没有这一步，"压住 0 次"会被误读成
  # "正常行驶不插手"——实测踩过：节点因 ParameterAlreadyDeclaredException 每次启动即崩，
  # 而判据只看到 0 次压住，得出了空结论。
  if ! grep -q '就绪' results/p6_guard.log 2>/dev/null; then
    echo "[p6] 安全层没起来（日志里没有"就绪"）——这次"压住 0 次"不作数。日志："
    tail -20 results/p6_guard.log
    exit 6
  fi
  echo "[p6] 安全层就绪"
fi

if [ "$DYN" = "1" ]; then
  if [ "$GUARD" != "1" ]; then
    echo "[p6] 注意：DYN=1 但 GUARD 没开 —— 这台推车没人会理它（测避障要 GUARD=1）"
  fi
  echo "[p6] 起动态障碍（推车横穿行驶线 x=${DYN_X}，停 ${DYN_HOLD} 仿真秒）→ results/p6_dyn.log"
  nohup setsid bash -lc "source $ROS && $PY scripts/move_obstacle.py --aisle $DYN_AISLE --x $DYN_X \
      --hold $DYN_HOLD --trigger ${DYN_TRIGGER:-5.0} --near ${DYN_NEAR:-1.6}" \
    > results/p6_dyn.log 2>&1 &
  DYNPID=$!
  sleep 1
fi

if [ "$NAV" = "1" ]; then
  echo "[p6] 起 Nav2 栈（静态图 + /scan 中继 + nav2 bringup）→ results/nav2_stack.log"
  # **STACK_ONLY=1**：只要它把栈起来并驻留（否则它会顺带跑 5 轮 M2 —— 实测漏传过一次，
  # 白等 240 s 还以为是"栈没起来"）。
  STACK_ONLY=1 nohup setsid bash scripts/m2_static_run.sh > results/nav2_stack.log 2>&1 &
  NAVPID=$!
  navok=0
  for i in $(seq 1 120); do
    if grep -q 'READY' results/nav2_stack.log 2>/dev/null; then navok=1; break; fi
    sleep 2
  done
  if [ "$navok" != "1" ]; then
    echo "[p6] Nav2 栈 240 s 未就绪 —— 这轮不作数。日志末尾："
    tail -25 results/nav2_stack.log
    exit 7
  fi
  # ⚠️ `READY` 只保证 **bt_navigator** active；lifecycle manager 是逐个激活的，此时
  # controller_server / planner_server 还没有 action server → 发目标会被**直接拒收**
  # （实测：demo 里 0.5 s 就返回 MOTION_NAV2_FAILED）。必须等三个都 active。
  navok=0
  for _ in $(seq 1 60); do
    navok=1
    for ns in controller_server planner_server bt_navigator; do
      st=$(bash -lc "source $ROS && ros2 lifecycle get /$ns 2>/dev/null" | head -1)
      case "$st" in active*) ;; *) navok=0 ;; esac
    done
    [ "$navok" = "1" ] && break
    sleep 2
  done
  if [ "$navok" != "1" ]; then
    echo "[p6] controller/planner 未全部 active（$ns: ${st:-?}）→ 这轮不作数，不跑 demo"
    exit 8
  fi
  echo "[p6] Nav2 全部 active（bt_navigator + controller + planner），act 后端 = $ACT_BACKEND"
fi

echo "[p6] 跑一条指令：$UTT"
# shellcheck disable=SC2086
$PY scripts/p6_demo.py --utterance "$UTT" --answer "$ANSWER" ${BLOCK:+--block "$BLOCK"} \
  ${SUBSTITUTE:+--substitute} ${NOLANE:+--no-lane-offset} --verify "$VERIFY" --pose-source "$POSE" \
  --act-backend "$ACT_BACKEND" --rtf "$RTF" ${P6_ARGS:-}
rc=$?
echo "[p6] demo 退出码 $rc"

# ---- 动态障碍判据：推车自检过了 + 安全层确实压住过 + 机器人仍到达
if [ "$DYN" = "1" ]; then
  wait "$DYNPID" 2>/dev/null
  dynrc=$?
  # GUARD 没开就别读 p6_guard.log（那是上一轮遗留的，会给出假的"压住 N 次"）
  if [ "$GUARD" = "1" ]; then
    blocks=$(grep -c '正在压住' results/p6_guard.log 2>/dev/null; true)
  else
    blocks=0
  fi
  blocks=${blocks:-0}
  bypass=$(grep -c '绕过/通过' results/p6_dyn.log 2>/dev/null; true)
  bypass=${bypass:-0}
  echo "[p6] 动态障碍判据：推车自检 ${dynrc:-?}（0=过）、安全层压住 ${blocks} 次、" \
       "绕过/通过 ${bypass}、demo 退出码 $rc（0=仍到达）"
  if [ "${dynrc:-9}" != "0" ]; then
    echo "[p6] 判据 FAIL：推车自检未过 → 这次实验无效（先看 results/p6_dyn.log）"
    [ "$rc" = "0" ] && rc=4
  fi
  # 后端决定"过"的标准：drive_to 只能停/等（要求安全层真压住过）；
  # nav2 要绕过去（要求挡路期间机器人越过横穿点）。两者都要求最终到达。
  if [ "$ACT_BACKEND" = "nav2" ]; then
    # 双条件（D31-6）：**机器人越过推车位** 且 **路径在推车 x 处偏出 ≥0.5 m**。
    # 只看前者会放过"擦过/推过"（实测假阳性两次）。
    dev=$(grep -oE '偏出 [0-9.]+ m' results/p6_dyn.log 2>/dev/null | tail -1 | grep -oE '[0-9.]+')
    echo "[p6] nav2 绕行判据：越过=$bypass、路径偏出=${dev:-?} m（需 ≥0.61 = 半宽0.26+inflation0.35）"
    if [ "$bypass" -lt 1 ] 2>/dev/null; then
      echo "[p6] 判据 FAIL（nav2）：没绕过去 —— 挡路期间机器人停在横穿点之前"
      [ "$rc" = "0" ] && rc=5
    elif ! awk -v d="${dev:-0}" 'BEGIN{exit !(d >= 0.61)}'; then
      echo "[p6] 判据 FAIL（nav2）：只擦过没绕（路径偏出 ${dev:-0} m < 0.61）"
      [ "$rc" = "0" ] && rc=5
    fi
  else
    if [ "$blocks" -lt 1 ] 2>/dev/null; then
      echo "[p6] 判据 FAIL（drive_to）：安全层一次都没压住 → 障碍没真正挡在路上"
      [ "$rc" = "0" ] && rc=5
    fi
  fi
  # 落一份机器可读的记录，`make accept` 用它自动认这条判据（不靠人读日志）
  events=$(grep -oE '共压住 [0-9]+ 次' results/p6_guard.log 2>/dev/null | tail -1 | grep -oE '[0-9]+' || true)
  # "过"的标准随后端变：drive_to 要"安全层压住过"、nav2 要"挡路期间绕过横穿点"
  if [ "$ACT_BACKEND" = "nav2" ]; then
    if [ "${dynrc:-9}" = "0" ] && [ "${bypass:-0}" -ge 1 ] && [ "${rc:-9}" = "0" ]; then pv=true; else pv=false; fi
  else
    if [ "${dynrc:-9}" = "0" ] && [ "${blocks:-0}" -ge 1 ] && [ "${rc:-9}" = "0" ]; then pv=true; else pv=false; fi
  fi
  # 记录里自带"障碍是否真的进了 costmap"的证据（D31 续 5/6：只报路径偏出会把静态绕行误算成避障）
  gd=$(grep -oE '/global_costmap/costmap: .*最近距离 [0-9.]+ m' results/p6_dyn.log | tail -1 | grep -oE '[0-9.]+ m$' | grep -oE '[0-9.]+')
  ld=$(grep -oE '/local_costmap/costmap: .*最近距离 [0-9.]+ m' results/p6_dyn.log | tail -1 | grep -oE '[0-9.]+ m$' | grep -oE '[0-9.]+')
  cat > "results/p6_dyn_$(date +%Y%m%dT%H%M%S).json" <<JSON
{
  "dyn_x": ${DYN_X},
  "aisle": "${DYN_AISLE}",
  "dyn_rc": ${dynrc:-9},
  "act_backend": "${ACT_BACKEND}",
  "path_dev_m": ${dev:-null},
  "global_min_d_m": ${gd:-null},
  "local_min_d_m": ${ld:-null},
  "bypass": ${bypass:-0},
  "guard_block_lines": ${blocks:-0},
  "guard_block_events": ${events:-0},
  "demo_rc": ${rc:-9},
  "utterance": "${UTT}",
  "pass": $pv
}
JSON
  echo "[p6] 动态障碍记录 → results/p6_dyn_$(date +%Y%m%dT%H%M%S).json"
fi
exit $rc
