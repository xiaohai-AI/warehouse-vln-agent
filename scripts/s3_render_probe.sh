#!/usr/bin/env bash
# 渲染路径定性：ign gazebo 到底走 NVIDIA GPU 还是 llvmpipe 软件光栅。
#
# 前三次探针都抓错了进程（make -> /bin/sh -c -> ruby -> ign gazebo，前三层 cmdline 里
# 都带 "warehouse.sdf" / "ign gazebo" 字样），所以 CPU 0%、maps 0 全是假阴性。
# 这次按**进程名（comm=ruby）**选，并把所有候选打出来，不再误判。
#
# 判据：
#   * maps 里有 libnvidia-* / /dev/nvidia*            -> 真 GPU
#   * maps 里有 swrast / libLLVM / llvmpipe            -> 软件光栅（这就是 0.05 Hz 的根因）
#   * nvidia-smi 进程列表里出现该 pid                   -> 真 GPU（最硬的一条）
#   * 传感器在出图期间 GPU util 一直 0%                 -> 软件光栅
set -uo pipefail
cd "$(dirname "$0")/.." || exit 2
mkdir -p results

pkill -f "[p]arameter_bridge" 2>/dev/null
make down > /dev/null 2>&1 || true
sleep 2

echo "[probe] 起仿真（gui:=false）…"
nohup make sim SIM_ARGS="gui:=false" > results/sim_render_probe.log 2>&1 &
for _ in $(seq 1 45); do pgrep -f "[i]gn gazebo" > /dev/null && break; sleep 1; done

echo
echo "=== 0) 候选进程（判断谁才是真正的渲染进程）"
for p in $(pgrep -f "[i]gn gazebo"); do
  printf "  pid=%-7s comm=%-10s threads=%-4s cmd=%s\n" "$p" \
    "$(cat /proc/"$p"/comm 2>/dev/null)" \
    "$(awk '/Threads/{print $2}' /proc/"$p"/status 2>/dev/null)" \
    "$(tr '\0' ' ' < /proc/"$p"/cmdline 2>/dev/null | cut -c1-70)"
done

pid=""
for p in $(pgrep -f "[i]gn gazebo"); do
  [ "$(cat /proc/"$p"/comm 2>/dev/null)" = "ruby" ] && pid=$p
done
if [ -z "$pid" ]; then
  pid=$(for p in $(pgrep -f "[i]gn gazebo"); do
          echo "$(awk '/Threads/{print $2}' /proc/"$p"/status 2>/dev/null) $p"
        done | sort -rn | head -1 | awk '{print $2}')
fi
if [ -z "${pid:-}" ]; then
  echo "[probe] ❌ 找不到渲染进程"; exit 2
fi
echo "[probe] 选定 pid=$pid comm=$(cat /proc/"$pid"/comm 2>/dev/null) DISPLAY=${DISPLAY:-<未设置>}"

echo
echo "[probe] 起相机桥（触发渲染）…"
nohup bash scripts/cam_bridge.sh > results/probe_bridge.log 2>&1 &
for i in $(seq 1 6); do
  if bash -lc "source /opt/ros/humble/setup.bash && timeout 30 .venv/bin/python scripts/clock_probe.py --topic /stereo_left --type sensor_msgs/msg/Image" 2>&1 | grep -qE "best_effort +[1-9]"; then
    echo "[probe] /stereo_left 有数据（渲染已触发）"; break
  fi
  echo "[probe]   等双目…（$i/6）"; sleep 10
done

echo
echo "=== 1) 该进程映射的图形库（硬证据）"
for pat in 'libnvidia' '/dev/nvidia' 'swrast' 'llvmpipe' 'libLLVM' 'libEGL' 'libGL' 'libvulkan'; do
  n=$(grep -c "$pat" /proc/"$pid"/maps 2>/dev/null || echo 0)
  printf "  %-12s %s 处\n" "$pat" "$n"
done
echo "  打开的设备节点："
ls -l /proc/"$pid"/fd 2>/dev/null | grep -oE '/dev/(nvidia[^ ]*|dri/[^ ]*)' | sort -u | head -6

echo
echo "=== 2) GPU 利用率采样（渲染期间，10 次 × 1 s；全 0 就是没用 GPU）"
for _ in $(seq 1 10); do
  nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader 2>/dev/null | tr '\n' ' '
  sleep 1
done
echo

echo
echo "=== 3) nvidia-smi 里的进程（出现 pid=$pid 才算真用 GPU）"
nvidia-smi 2>/dev/null | sed -n '/Processes/,$p' | head -12

echo
echo "=== 4) CPU 占用（软件光栅会吃满一个核以上）"
ps -o pid,%cpu,nlwp,etime,comm -p "$pid" 2>/dev/null
echo "  整机：$(uptime | sed 's/.*load average/load average/')"

echo
echo "=== 5) 渲染引擎日志"
grep -iE 'ogre|render|engine|vulkan|egl' results/sim_render_probe.log | head -8

echo
pkill -f "[p]arameter_bridge" 2>/dev/null
make down > /dev/null 2>&1 || true
echo "[probe] GPU 内核错误计数（应为 0）：$(journalctl -k -b 0 --no-pager 2>/dev/null | grep -c 'nvidia-modeset: ERROR')"
