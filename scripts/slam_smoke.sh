#!/usr/bin/env bash
# S1 冒烟：点云链 -> slam_toolbox -> 存图。一条命令跑完，用来判断"新扫描链能不能建图"。
#
# 与 SLAM-1（P4）的区别：这里只验"链路 + 建图能出东西"，不做 200 m 固定路线与三配置对比。
# 走 12 m 直线（C 巷中线），存到 maps/depot_slam_smoke，然后把图的基本统计打出来。
set -uo pipefail
cd "$(dirname "$0")/.." || exit 2

ROS=${ROS:-/opt/ros/humble/setup.bash}
PY=${PY:-.venv/bin/python}
mkdir -p results maps

cleanup() {
  local p
  for p in slam_toolbox scan_relay pointcloud_to_laserscan; do
    pkill -f "[${p:0:1}]${p:1}" 2>/dev/null
  done
  sleep 1
  for p in slam_toolbox scan_relay pointcloud_to_laserscan; do
    pkill -9 -f "[${p:0:1}]${p:1}" 2>/dev/null
  done
}
trap cleanup EXIT

if ! pgrep -f "[w]arehouse.sdf" > /dev/null; then
  echo "[slam-smoke] 仿真没在跑：先 make sim SIM_ARGS='gui:=false'"
  exit 2
fi

echo "[slam-smoke] 起 slam（日志 results/slam_smoke.log）…"
bash -lc "source $ROS && ros2 launch launch/slam.launch.py" > results/slam_smoke.log 2>&1 &
sleep 10

# /scan 必须有数据才值得往下走（p2l QoS 是 BEST_EFFORT，用同一个探针确认）
out=$(bash -lc "source $ROS && timeout 40 $PY scripts/clock_probe.py --topic /scan --type sensor_msgs/msg/LaserScan" 2>&1)
n=$(echo "$out" | awk '/best_effort/ {print $2}' | head -1)
if [ -z "${n:-}" ] || [ "${n:-0}" -le 0 ]; then
  echo "[slam-smoke] /scan 没数据，链路有问题："
  echo "$out" | tail -12
  exit 3
fi
echo "[slam-smoke] /scan 有数据（6 s 收 $n 条），开始走 12 m"

# 车头摆正 + 压巷道中线（y=-2.15）走 12 m；map_drive 只用 gz 话题，不依赖 ROS
$PY scripts/map_drive.py --turn-to 0 --distance 0 --max-seconds 60 || true
$PY scripts/map_drive.py --vx 0.25 --distance 12 --hold-y -2.15 --max-seconds 300 || true

echo "[slam-smoke] 存图 -> maps/depot_slam_smoke"
bash -lc "source $ROS && ros2 run nav2_map_server map_saver_cli -f maps/depot_slam_smoke \
          --ros-args -p use_sim_time:=true" 2>&1 | tail -8

if [ -f maps/depot_slam_smoke.pgm ]; then
  $PY - <<'EOF'
# 统计 + 几何对账。
# nav2 pgm 的三档：0=占据, 254=自由, 205=未知（之前用 v>200 判自由，把未知也算成自由，
# 于是打印出"已知 100%"这种假数据 —— 判据脚本里必须精确比较这三个值）。
import collections
import json
import sys
sys.path.insert(0, "scripts")
from depot_geometry import load_pgm

img, w, h = load_pgm("maps/depot_slam_smoke.pgm")
occ = sum(1 for v in img if v < 100)
free = sum(1 for v in img if v == 254)
unk = sum(1 for v in img if v not in (0,) and 100 <= v <= 210 and v != 254)
known = (len(img) - unk) / len(img) * 100

meta = {}
for line in open("maps/depot_slam_smoke.yaml"):
    if ":" in line and not line.strip().startswith("#"):
        k, v = line.split(":", 1)
        meta[k.strip()] = v.strip()
res = float(meta["resolution"])
x0, y0 = [float(v) for v in meta["origin"].strip("[]").split(",")[:2]]

print(f"[slam-smoke] 图 {w}x{h} @{res} m = {w*res:.1f}x{h*res:.1f} m  "
      f"占据 {occ} 自由 {free} 未知 {unk} 已知 {known:.1f}%")

# 几何对账：货架带（世界 y 已知，间隔 4.3 m）在图里应出现在同样的 y 上
ys = []
for r in range(h):
    base = r * w
    for c in range(w):
        if img[base + c] < 100:
            ys.append(y0 + (h - r - 0.5) * res)
hist = collections.Counter(round(y / 0.2) * 0.2 for y in ys)
peaks = [y for y, n in hist.most_common(8)]

layout = json.loads(open("world/layout.json").read())
# 切片 z∈[0.1,0.6] 落在货架第一层台面（z=0.15），所以图里是**整个 1.1 m 深的货架体**，
# 直方图峰出现在货架**中心**（间隔 4.3 m），不是货架端面。按中心 + 南北墙对账。
expected = [b["center_y"] for b in layout["blocks"]]
walls_y = [-19.9, 19.9]
hit = [p for p in peaks if any(abs(p - e) < 0.50 for e in expected + walls_y)]
print(f"[slam-smoke] 占据带峰值 y = {sorted(peaks)}")
print(f"[slam-smoke] 与世界货架中心/墙对齐 {len(hit)}/{len(peaks)}（判据 ≥6/8）")

ok = occ > 500 and known > 5 and len(hit) >= 6
print("RESULT: PASS  —— 建图链路可用，且几何与世界对得上" if ok
      else "RESULT: FAIL  —— 地图空或几何对不上（看上面的峰值行）")
EOF
else
  echo "[slam-smoke] RESULT: FAIL —— 没有生成 pgm"
  exit 3
fi
