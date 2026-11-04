#!/usr/bin/env bash
# 诊断 L23：**单线雷达到底看不看得见低矮障碍**（静止靶标，不需要 Nav2、不需要机器人移动）
#
# 背景：受控台里推车（0.62×0.52×0.90 m）就在车前 ~1.55 m，但挡路期间 `/scan` 最近回波是
# **2.00 m** —— 说明这一圈没扫到它。`scan_relay` 报告 count=360 每圈=360 线数=1（单线），
# `lidar_z=0.55`、车高 0.90 m，**理论上应该看得见**。这个脚本就是去证明它到底看不看得见。
#
# 做法：机器人停在出生点 (-31,0) 不动、朝 +x；把推车依次摆到正前方 1.0 / 2.0 / 3.0 / 5.0 m，
# 每次读 `/scan` 的最近回波。
#   看得见 → 最近回波 ≈ 距离 − 0.31（车的前表面）
#   看不见 → 最近回波一直停在远处的货架距离（~2 m 以上，且不随距离变化）
#
#   bash scripts/scan_target_probe.sh
# **不要用 `set -u`**：`source /opt/ros/humble/setup.bash` 里有未定义变量，`-u` 会让 shell
# 直接退出（而且我把 stderr 重定向到 /dev/null，表现成"一声不响退出码 1"——实测踩过一次）。
# 其他脚本（p6_demo.sh / avoid_probe.sh）都只用 `set -o pipefail`，这里保持一致。
set -o pipefail
cd "$(dirname "$0")/.." || exit 1
ROOT=$(pwd)
PY=$ROOT/.venv/bin/python
ROS_SETUP=/opt/ros/humble/setup.bash
# shellcheck disable=SC1091
if [ -f "$ROS_SETUP" ]; then
  # shellcheck disable=SC1091
  source "$ROS_SETUP"
else
  echo "[scanprobe] 找不到 $ROS_SETUP"; exit 1
fi

SIMPID="" RELAYPID=""
cleanup() {
  echo "[scanprobe] 收尾"
  [ -n "$RELAYPID" ] && kill -9 -"$RELAYPID" 2>/dev/null
  [ -n "$SIMPID" ] && kill -TERM -"$SIMPID" 2>/dev/null && sleep 2 && kill -9 -"$SIMPID" 2>/dev/null
  make down >/dev/null 2>&1
}
trap cleanup EXIT INT TERM

echo "[scanprobe] 起仿真（headless）+ 机器人模型"
make robot CAM_ARGS="cam_width:=320 cam_height:=240 cam_rate:=5 cam_pitch:=0" >/dev/null || exit 1
nohup setsid make sim SIM_ARGS='gui:=false' > results/scanprobe_sim.log 2>&1 &
SIMPID=$!
for _ in $(seq 1 60); do
  pubs=$(ros2 topic info /odom 2>/dev/null | awk '/Publisher count/ {print $3}' | head -1)
  [ -n "${pubs:-}" ] && [ "${pubs:-0}" -ge 1 ] 2>/dev/null && break
  sleep 2
done
echo "[scanprobe] 仿真就绪"

echo "[scanprobe] 起 /scan 中继 → results/scanprobe_relay.log"
nohup setsid make scan-relay > results/scanprobe_relay.log 2>&1 &
RELAYPID=$!
for _ in $(seq 1 30); do
  pubs=$(ros2 topic info /scan 2>/dev/null | awk '/Publisher count/ {print $3}' | head -1)
  [ -n "${pubs:-}" ] && [ "${pubs:-0}" -ge 1 ] 2>/dev/null && break
  sleep 1
done
if [ -z "${pubs:-}" ] || [ "${pubs:-0}" -lt 1 ] 2>/dev/null; then
  echo "[scanprobe] /scan 没起来，看 results/scanprobe_relay.log"; exit 5
fi
grep -h '锁定布局' results/scanprobe_relay.log | tail -1

# 用现成的 SDF 造车（move_obstacle 生成的 dynamic + gravity=false 版本）
SDF=$ROOT/world/fixtures/dyn_cart.sdf
[ -f "$SDF" ] || { echo "[scanprobe] 缺 $SDF"; exit 6; }

svc() {  # $1=service $2=reqtype $3=req
  ign service -s "/world/depot/$1" --reqtype "$2" --reptype ignition.msgs.Boolean \
    --timeout 5000 --req "$3" 2>/dev/null | grep -q 'data: true'
}

echo "[scanprobe] 造一台推车（先放到远侧 y=+3 作为基线）"
svc create ignition.msgs.EntityFactory \
  "sdf_filename: \"$SDF\", name: \"dyn_cart\", pose: {position: {x: -31.0, y: 3.0, z: 0.45}}" \
  || echo "[scanprobe] create 没返回 true（可能已存在，继续试 set_pose）"
sleep 2

min_scan() { $PY scripts/scan_min.py 8; }   # 见 scripts/scan_min.py

echo
echo "================ 静止靶标结果 ================"
printf '%-10s %-14s %-12s %s\n' "设定距离" "最近回波(m)" "预期(看得见)" "判读"
base=$(min_scan)
printf '%-10s %-14s %-12s %s\n' "无车基线" "$base" "-" "环境里最近的货架/墙"

for d in 1.0 2.0 3.0 5.0; do
  x=$(awk -v d="$d" 'BEGIN{printf "%.3f", -31.0 + d}')
  svc set_pose ignition.msgs.Pose "name: \"dyn_cart\", position: {x: $x, y: 0.000, z: 0.45}" >/dev/null
  sleep 2
  got=$(min_scan)
  exp=$(awk -v d="$d" 'BEGIN{printf "%.2f", d - 0.31}')
  # 看得见 → |回波 − (距离−0.31)| ≤ 0.30；看不见 → 回波比预期大 0.5 m 以上
  verdict=$(awk -v g="$got" -v e="$exp" 'BEGIN{
      d = (g > e ? g - e : e - g);
      if (g != g || e != e) print "读不到";
      else if (d <= 0.30) print "**看得见**";
      else if (g > e + 0.5) print "**看不见**";
      else print "可疑";
    }')
  printf '%-10s %-14s %-12s %s\n' "${d} m" "$got" "$exp" "$verdict"
done

echo
echo "判读：四行全部「看得见」→ L23 不成立（雷达能看见低矮障碍，之前的 2.00 m 另有原因）"
echo "      任一行「看不见」→ 确认盲区：改 scan_relay 的环选择（多环取最小 / 选最近环）"
echo "=============================================="
