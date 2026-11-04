#!/usr/bin/env bash
# SEG-1 全 6 条巷道的点云采集（边开边拼）。分批跑，每批结束车停在确定位置。
#
# 为什么不是一个横扫：
#   * 死循环驱动会把自己锁进错误假设里；分批跑，一批失败只损失那几分钟，车还在已知位置。
#   * A/B/C/D/E/F 用**交错方向**（西→东、东→西），省掉每巷 57 m 的空返；横通道 x=±31.5
#     在所有巷道 y 上都是通的（已用固化碰撞体逐点验过）。
#
# 两个必须偏离中线的地方（也是验证过的）：
#   * C 巷（y=-2.15）有叉车横在 y∈[-3.01,-2.39]、x∈[-18.5,-17.3]，中线擦边 →
#     全线走 y=-1.85（北偏 0.3 m），到货架面还剩 0.99 m 余量。
#   * F 巷（y=+10.75）在 x=11.29–12.01 有一道**横跨整条巷宽**的护栏（y 9.16–12.34，
#     高 1.14 m）→ 车过不去，护栏以东 7.5 个 bay 只能从东侧看。所以 F 分两段抓：
#     F_east 28.5→13.2、F_west -28.5→10.4，合并后覆盖整条巷。
#
# 用法
#   GROUP=1 bash scripts/seg1_capture_all.sh     # A + B
#   GROUP=2 bash scripts/seg1_capture_all.sh     # C + D
#   GROUP=3 bash scripts/seg1_capture_all.sh     # F西 + E
#   GROUP=B bash scripts/seg1_capture_all.sh     # 单巷续抓：车已在该巷里，直接开抓（不归位）
#                                                #   B/C/D/E/F 都可这样单独补跑）
#
# 每批开头先跑 seg1_norm.py 归位（只做纯横/纵向移动，绝不斜穿货架），所以
# **任何一批都可以从任意位置重跑**；已抓好的 npz 会自动跳过（FORCE=1 强制重抓）。
set -o pipefail

source /opt/ros/humble/setup.bash
cd "$(dirname "$0")/.." || exit 1
PY=${PY:-.venv/bin/python}
GROUP=${GROUP:-1}

# 速度按实测 RTF ≈ 0.35 定（墙钟 = 仿真时间 / 0.35）：
#   采集 0.45 m/s + 6 Hz -> 点间距 7.5 cm（与原来 0.28@4Hz 同密度），57 m 约 6 分钟走完；
#   转场 0.6 m/s（不采点，只求快）。低于这个速度跑不完 57 m 就会撞 420 s 预算。
# go 走 seg1_go.py 的安全路由（横通道 + 目标巷道，绝不斜穿货架）：
# 分批采集里起点是"上一批停在哪"，直接用直线会横切货架。
go() {   # go X Y
  echo "=== [go] -> ($1, $2)"
  "$PY" scripts/seg1_go.py --x "$1" --y "$2" --speed 0.60 --tol 0.25 --max-seconds 420 || exit 1
}
cap() {  # cap TO_X AISLE_Y OUT —— 已有成品就跳过（断点续跑）；FORCE=1 可强制重抓
  local tx=$1 ay=$2 out=$3
  if [ -f "$out" ] && [ "${FORCE:-0}" != "1" ]; then
    local sz; sz=$(stat -c%s "$out" 2>/dev/null || echo 0)
    if [ "$sz" -gt 1000000 ]; then
      echo "=== [cap] 跳过（已存在 ${sz} B）：$out"
      return 0
    fi
  fi
  echo "=== [cap] -> ($tx, $ay)  $out"
  # 必须用 --to= 形式：负坐标写成 `--to -28.5,-6.45` 会被 argparse 当成选项
  # （报 "argument --to: expected one argument"，B 巷第一次就是这么挂的）
  "$PY" scripts/capture_cloud_path.py "--to=$tx,$ay" "--aisle-y=$ay" \
        --speed 0.45 --hz 6.0 --max-seconds 600 --out "$out" || exit 1
}

# 归位：GROUP=1/2/3 先只做"纯横向/纯纵向"移动回到横通道，保证后续 go 安全。
# 单巷续抓（GROUP=B / C / D / E / F）**不归位** —— 车已经在那条巷里，直接开抓。
if [ "$GROUP" != "B" ] && [ "$GROUP" != "C" ] && [ "$GROUP" != "D" ] \
   && [ "$GROUP" != "E" ] && [ "$GROUP" != "F" ]; then
  "$PY" scripts/seg1_norm.py || echo "（归位未完全到位，继续：后续 go 会自我纠正）"
fi

case "$GROUP" in
B|b)    cap -28.5 -6.45 results/seg1_path_B.npz ;;
C|c)    cap  28.5 -1.85 results/seg1_path_C.npz ;;
D|d)    cap -28.5  2.15 results/seg1_path_D.npz ;;
E|e)    cap  28.5  6.45 results/seg1_path_E.npz ;;
F|f)    cap  10.4 10.75 results/seg1_path_F_west.npz ;;
1)
  go -31.5 -10.75 ; go -28.5 -10.75 ; cap 28.5 -10.75 results/seg1_path_A.npz
  go  31.5 -10.75 ; go  31.5 -6.45 ; go  28.5 -6.45 ; cap -28.5 -6.45 results/seg1_path_B.npz
  ;;
2)
  go -31.5 -6.45  ; go -31.5 -1.85 ; go -28.5 -1.85 ; cap 28.5 -1.85 results/seg1_path_C.npz
  go  31.5 -1.85  ; go  31.5 2.15  ; go  28.5 2.15  ; cap -28.5 2.15 results/seg1_path_D.npz
  ;;
3)
  go -31.5 2.15   ; go -31.5 10.75 ; go -28.5 10.75 ; cap 10.4 10.75 results/seg1_path_F_west.npz
  go -31.5 10.75  ; go -31.5 6.45  ; go -28.5 6.45  ; cap 28.5 6.45 results/seg1_path_E.npz
  # F 巷只抓西段：中段 x 12.0–28.1 被 barrier_00/01 前后围死（都横跨整条巷宽、高 1.14 m），
  # 车进不去、雷达（0.47 m）也看不进去 → 这 7 个 bay 物理不可观测，报告里如实标注。
  ;;
*)
  echo "GROUP 只能是 1 / 2 / 3"; exit 2
  ;;
esac

echo "=== 本批完成。已有点云："
ls -l results/seg1_path_*.npz 2>/dev/null | awk '{print "   ", $NF, $5" B"}'
