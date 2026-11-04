#!/usr/bin/env bash
# Nav2 避障**快速试验台**（约 2 分钟一轮，替代 8 分钟的整条 demo）
#
# 为什么单独建这个台子：调一次避障参数如果只能靠 p6-demo（20 m 行程 / 230 s），
# 每轮 8 分钟，试几个参数就是半小时。这里把推车挪到**离出生点 5 m**处，
# 目标只走 10 m —— 同样的物理问题，一轮 2 分钟，可以放心做 A/B。
#
# 用法
#   bash scripts/avoid_probe.sh                      # 默认：推车 -26、目标 -21
#   CART_X=-26 GOAL_X=-21 HOLD=40 bash scripts/avoid_probe.sh
#   TUNE="global_costmap.global_costmap.inflation_layer.inflation_radius=0.28" \
#     bash scripts/avoid_probe.sh                    # 起栈后临时改参数（逗号分隔多个）
#
# 判据（脚本自己打印，不靠人看 RViz）
#   [dyn] 挡路期间机器人 x: A → B      —— 推进 >1 m 且越过推车 → 绕过去了
#   [dyn] Nav2 全局规划 y 范围          —— 偏出行驶线 >0.3 m 才算真绕行
#   [dyn] /local_costmap 推车位附近致命格 —— 0 = 激光有回波但 costmap 没标上
#   [probe] 结果：BYPASSED / WAITED / FAILED
set -o pipefail
# ⚠️ 本脚本名**不能**含 `nav2_`：`m2_static_run.sh` 起栈时用 `pkill -f "[n]av2_"`
# 清理旧 Nav2 进程，会按命令行匹配 —— 名字里带 nav2_ 的脚本会被**自己起的栈杀掉**
# （踩过：开局打印到"起 Nav2 栈"就 `已杀死`，连超时提示都来不及打）。同理日志名也别带。
cd "$(dirname "$0")/.." || exit 1
ROOT=$(pwd)
PY=$ROOT/.venv/bin/python
ROS_SETUP=/opt/ros/humble/setup.bash

CART_X=${CART_X:--26.0}
GOAL_X=${GOAL_X:--21.0}
GOAL_Y=${GOAL_Y:--1.60}
# ⚠️ 推车必须挡在**机器人真实会走的直线**上，不能照抄巷里的行驶线 y：
#   从出生点 (-31,0) 到目标 (-21,-1.6) 的直线在 x=-26 处 y=-0.80，
#   把车放在 y=-1.60 时机器人从北侧 0.8 m 擦过去，**根本不用绕** ——
#   这曾让一轮试验台报出假 "BYPASSED"（路径 y 范围 -1.62~-0.87 就是那条斜线本身）。
# 出生点 X 从 catalog 取（原来写死 +31.0，换世界就插值错）
SPAWN_X=${SPAWN_X:-$("${PY:-python3}" -c 'from depot_vln.catalog import get_catalog; print(get_catalog().spawn["x"])')}
CART_Y=${CART_Y:-$(awk -v cx="$CART_X" -v gx="$GOAL_X" -v gy="$GOAL_Y" -v sx="$SPAWN_X" \
  'BEGIN{printf "%.2f", (cx-sx)/(gx-sx)*gy}')}
HOLD=${HOLD:-40}            # 仿真秒：车停在路上多久（40 ≈ 90 墙钟秒，够机器人做决定）
GOAL_TIMEOUT=${GOAL_TIMEOUT:-200}
TUNE=${TUNE:-}

# shellcheck disable=SC1091
source "$ROS_SETUP" 2>/dev/null || { echo "[probe] 找不到 $ROS_SETUP"; exit 1; }

SIMPID="" NAVPID="" DYNPID=""
cleanup() {
  echo "[probe] 收尾"
  [ -n "$DYNPID" ] && kill -9 -"$DYNPID" 2>/dev/null
  if [ -n "$NAVPID" ]; then
    kill -TERM -"$NAVPID" 2>/dev/null; sleep 3; kill -9 -"$NAVPID" 2>/dev/null
  fi
  [ -n "$SIMPID" ] && kill -TERM -"$SIMPID" 2>/dev/null && sleep 2 && kill -9 -"$SIMPID" 2>/dev/null
  make down >/dev/null 2>&1
  echo "[probe] 残留进程数（应 0）：$(pgrep -fc '[w]arehouse.sdf' || true)$(pgrep -fc '[n]av2' || true)"
}
trap cleanup EXIT INT TERM

# ---------------------------------------------------------------- 1. 模型 + 仿真
echo "[probe] 重建机器人模型 + 起仿真（headless）"
make robot CAM_ARGS="cam_width:=320 cam_height:=240 cam_rate:=5 cam_pitch:=0" >/dev/null \
  || { echo "[probe] make robot 失败"; exit 1; }
nohup setsid make sim SIM_ARGS='gui:=false' > results/probe_sim.log 2>&1 &
SIMPID=$!
for _ in $(seq 1 60); do
  pubs=$(ros2 topic info /odom 2>/dev/null | awk '/Publisher count/ {print $3}' | head -1)
  [ -n "${pubs:-}" ] && [ "${pubs:-0}" -ge 1 ] 2>/dev/null && break
  sleep 2
done
echo "[probe] 仿真就绪（/odom 发布者 ${pubs:-?}）"

# ---------------------------------------------------------------- 2. Nav2 栈
echo "[probe] 起 Nav2 栈 → results/probe_stack.log"
STACK_ONLY=1 nohup setsid bash scripts/m2_static_run.sh > results/probe_stack.log 2>&1 &
NAVPID=$!
for _ in $(seq 1 120); do
  grep -q 'READY' results/probe_stack.log 2>/dev/null && break
  sleep 2
done
if ! grep -q 'READY' results/probe_stack.log 2>/dev/null; then
  echo "[probe] Nav2 栈 240 s 未就绪，日志末尾："
  tail -20 results/probe_stack.log
  exit 7
fi
echo "[probe] bt_navigator active；再等 controller/planner 也 active"
# READY 只保证 bt_navigator 起来了：lifecycle manager 是一个个激活的，BT 先 active 时
# 底下还没有 action server → 此时发目标会被**直接拒收**（`Goal was rejected`，踩过）。
for _ in $(seq 1 60); do
  ok=1
  for ns in controller_server planner_server bt_navigator; do
    st=$(ros2 lifecycle get "/$ns" 2>/dev/null | head -1)
    case "$st" in active*) ;; *) ok=0 ;; esac
  done
  [ "$ok" = "1" ] && break
  sleep 2
done
if [ "$ok" != "1" ]; then
  echo "[probe] controller/planner 仍未全部 active（$ns: ${st:-?}）→ 不继续"
  exit 8
fi
echo "[probe] Nav2 全部 active"

# 可选：临时改参数（A/B 调参用，不改文件）
if [ -n "$TUNE" ]; then
  IFS=',' read -ra kv <<< "$TUNE"
  for item in "${kv[@]}"; do
    node=${item%%=*}; val=${item#*=}; node=${node%.*}
    echo "[probe] 参数 $node = $val"
    ros2 param set "/$node" "${item#*=}" >/dev/null 2>&1 || \
      ros2 param set "/${item%%=*}" "$val" >/dev/null 2>&1 || \
      echo "[probe] （改参数失败：$item）"
  done
  sleep 2
fi

# ---------------------------------------------------------------- 目标串（CONTROL 与正式流程共用）
# **必须放在 CONTROL 分支之前**：之前 CONTROL 里引用了还没定义的 $GOAL，发出去的是空目标，
# `ros2 action send_goal` 什么也没做 → 一个 /plan 采样都没取到（踩过一次）。
GOAL="{pose: {header: {frame_id: 'map'}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.000000}}}}"

# ---------------------------------------------------------------- 对照实验（CONTROL=1）
# **为什么必须有这个模式**：`/scan` 最近回波 2.0 m 那种情况下，激光可能根本没打到推车，
# 那么"路径偏出 0.69 m"就可能来自**静态地图的货架**而不是障碍避让 —— 假阳性。
# 对照：撤掉推车、同样的起点终点，看路径在同一个 x 处还偏不偏。
if [ "${CONTROL:-0}" = "1" ]; then
  echo "[probe] CONTROL：**不放推车**，只测路径在 x=$CART_X 处的 y（对照基线）"
  straight=$(awk -v cx="$CART_X" -v gx="$GOAL_X" -v gy="$GOAL_Y" \
    'BEGIN{printf "%.2f", (cx-sx)/(gx-sx)*gy}')
  ( timeout "$GOAL_TIMEOUT" ros2 action send_goal /navigate_to_pose \
      nav2_msgs/action/NavigateToPose "$GOAL" > results/probe_goal.log 2>&1 ) &
  GPID=$!
  $PY scripts/plan_probe.py --x "$CART_X" --seconds 50 --straight-y "$straight" \
    | tee results/probe_control.log
  wait $GPID 2>/dev/null
  echo "---- NavigateToPose ----"
  grep -E 'status' results/probe_goal.log | tail -2
  dev=$(grep -oE '偏出 均值 [-+0-9.]+ m' results/probe_control.log | tail -1 | grep -oE '[-+0-9.]+')
  echo
  echo "---- 对照判决 ----"
  # 落一份机器可读的对照记录：P6-2f 改成**配对判据**（有车偏出 − 无车偏出 ≥ 0.5 m），
  # 只比"有车时的偏出"会把静态地图本来就有的绕行误算成避障（已踩：0.69 m 全来自基线）。
  if [ -n "${dev:-}" ]; then
    cat > "results/avoid_control_$(date +%Y%m%dT%H%M%S).json" <<JSON
{
  "cart_x": $CART_X, "straight_y": $straight,
  "control_dev_m": $dev, "kind": "control_no_cart"
}
JSON
  fi
  if [ -z "${dev:-}" ]; then
    echo "[probe] 对照没取到路径采样 → 这次不算（先查 results/probe_control.log）"
  elif awk -v d="$dev" 'BEGIN{exit !((d < 0 ? -d : d) < 0.15)}'; then
    echo "[probe] 对照偏出 ${dev} m（≈0）→ 有车那次的 0.69 m **是推车造成的**：避障成立，P6-2f 保住"
  else
    echo "[probe] 对照偏出 ${dev} m（不≈0）→ 那个偏移来自**静态地图**，有车那次是**假阳性**：P6-2f 必须降级"
  fi
  exit 0
fi

# ---------------------------------------------------------------- 3. 推车就位（不等机器人）
echo "[probe] 摆推车到 ($CART_X,$CART_Y)，停 $HOLD 仿真秒"
# `--block-y` 必须显式传：不传时 move_obstacle 用"该巷行驶线"（C 巷 = 中线+0.55 = -1.60），
# 而试验台里机器人走的是出生点→目标的斜线 —— 车会被摆到路径旁边 0.44 m，变成"擦过"。
nohup setsid $PY scripts/move_obstacle.py --aisle C --x "$CART_X" --block-y "$CART_Y" \
  --trigger 0 --hold "$HOLD" --rtf 0.44 --speed 1.0 > results/probe_dyn.log 2>&1 &
DYNPID=$!
sleep 4      # 让车先摆到位

# ---------------------------------------------------------------- 4. 发一个目标（GOAL 已在上面构造）
echo "[probe] NavigateToPose → ($GOAL_X,$GOAL_Y)，最多等 $GOAL_TIMEOUT s"
goalc=1
for attempt in 1 2 3; do
  timeout "$GOAL_TIMEOUT" ros2 action send_goal /navigate_to_pose \
    nav2_msgs/action/NavigateToPose "$GOAL" > results/probe_goal.log 2>&1
  goalc=$?
  if grep -q 'SUCCEEDED\|ABORTED\|CANCELED' results/probe_goal.log 2>/dev/null; then
    echo "[probe] 目标结果：$(grep -oE 'status: [A-Z]+' results/probe_goal.log | tail -1)"
    break
  fi
  echo "[probe] 第 $attempt 次：目标没收下（$(grep -oE 'rejected|Waiting.*' results/probe_goal.log | head -1)），8 s 后重试"
  sleep 8
done

# 3) 等推车那边的测量跑完（它会 hold 一段时间并采 4 组探针）；
#    不能目标一结束就收尾 —— 那样会杀掉还在造车/测量的进程（踩过：日志只剩两行）
echo "[probe] 等动态障碍测量结束（最多 150 s）"
for _ in $(seq 1 150); do
  kill -0 "$DYNPID" 2>/dev/null || break
  sleep 1
done

# ---------------------------------------------------------------- 5. 报告
echo
echo "================ 探针结果 ================"
grep -E '\[dyn\]' results/probe_dyn.log || echo "（没有 [dyn] 日志）"
echo "---- NavigateToPose ----"
grep -E 'status|SUCCEEDED|ABORTED|CANCELED' results/probe_goal.log | tail -3 || echo "（没有 goal 结果，退出码 $goalc）"
echo "---- 判决 ----"
xs=$(grep -oE '挡路期间机器人 x: [-+0-9.]+ → [-+0-9.]+' results/probe_dyn.log | tail -1)
end=$(echo "$xs" | awk '{print $NF}')
dev=$(grep -oE '偏出 [0-9.]+ m' results/probe_dyn.log | tail -1 | grep -oE '[0-9.]+')
echo "[probe] 推车位置 ($CART_X, $CART_Y) —— 由起点→目标直线算出，保证真挡路"
echo "[probe] 路径绕行量 ${dev:-?} m（<0.5 m = 没绕，只是擦过）"
if [ -n "${end:-}" ] && [ -n "${dev:-}" ] \
   && awk -v e="$end" -v c="$CART_X" -v d="$dev" 'BEGIN{exit !(e > c + 0.5 && d >= 0.5)}'; then
    echo "[probe] 结果：BYPASSED（车仍挡路时机器人越过 x=$CART_X，且路径偏出 ${dev} m）"
  # 把两层 costmap 的"最近致命格距离"也记进去 —— 验收要判"障碍真的被标上了"，
  # 只看路径偏出会把"静态地图绕行"误算成避障（踩过两次）。
  gd=$(grep -oE '/global_costmap/costmap: .*最近距离 [0-9.]+ m' results/probe_dyn.log | tail -1 | grep -oE '[0-9.]+ m$' | grep -oE '[0-9.]+')
  ld=$(grep -oE '/local_costmap/costmap: .*最近距离 [0-9.]+ m' results/probe_dyn.log | tail -1 | grep -oE '[0-9.]+ m$' | grep -oE '[0-9.]+')
  cat > "results/avoid_probe_$(date +%Y%m%dT%H%M%S).json" <<JSON
{
  "cart_x": $CART_X, "cart_y": $CART_Y,
  "goal_x": $GOAL_X, "goal_y": $GOAL_Y,
  "path_dev_m": ${dev:-0},
  "path_clear_m": ${dev:-0},
  "global_min_d_m": ${gd:-null}, "local_min_d_m": ${ld:-null},
  "robot_x_end": ${end:-0},
  "pass": true, "backend": "nav2"
}
JSON
elif echo "$xs" | grep -q '→'; then
  echo "[probe] 结果：WAITED（停在推车之前；路径偏出 ${dev:-?} m）"
  cat > "results/avoid_probe_$(date +%Y%m%dT%H%M%S).json" <<JSON
{
  "cart_x": $CART_X, "cart_y": $CART_Y,
  "goal_x": $GOAL_X, "goal_y": $GOAL_Y,
  "path_dev_m": ${dev:-0}, "robot_x_end": ${end:-0},
  "pass": false, "backend": "nav2"
}
JSON
else
  echo "[probe] 结果：FAILED（没读到姿态采样，查 results/probe_dyn.log）"
fi
