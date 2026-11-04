#!/usr/bin/env bash
# 交付自检（离线，不开仿真）：把 16 条判据的记账重算一遍，并核对支撑产物是否齐。
#
# 用途：给一个干净环境做「这份交付物是自洽的」验证，约 30 s。
#      真跑复现（会起仿真）见 docs/DELIVERY.md §1 的第 2、3 条命令。
#
#   bash scripts/delivery_check.sh
#
# 退出码：0 = 全部通过；非 0 = 有判据未通过或有产物缺失（会逐条列出）
set -uo pipefail
cd "$(dirname "$0")/.." || exit 1
ROOT=$(pwd)
PY=$ROOT/.venv/bin/python
[ -x "$PY" ] || PY=python3.10

rc=0

echo "=============================================================="
echo "1/3  重算判据（make accept → docs/ACCEPTANCE.md）"
echo "=============================================================="
# accept.py 在有 FAIL 判据时会返回非零 —— 那不代表脚本坏了，判据结果看下面的合计行。
if ! $PY scripts/accept.py; then
  echo "[WARN] accept 返回非零（通常是存在 FAIL 判据，见下）"
fi
acc_total=$(grep -oE '合计：PASS [0-9]+ / FAIL [0-9]+ / 未跑 [0-9]+' docs/ACCEPTANCE.md | tail -1)
echo
echo "→ $acc_total"
# 不硬编码 PASS 数（证据会随复现增加）；只要求 **FAIL 0 且 未跑 0**。
# 有 FAIL 就把那一行打出来 —— 交付自检的作用是暴露它，不是掩盖它。
if ! echo "$acc_total" | grep -qE 'FAIL 0 / 未跑 0'; then
  echo "[FAIL] 有判据未通过或未跑 —— 逐行看 docs/ACCEPTANCE.md："
  # `--text`：中文判据行含多字节字符，grep 会误判为二进制并吞掉输出（踩过）
  grep --text -E '\| (FAIL|未跑) \|' docs/ACCEPTANCE.md | cut -c1-160
  rc=1
fi

echo
echo "=============================================================="
echo "2/3  文档一致性（make docs-check）"
echo "=============================================================="
if ! $PY scripts/docs_consistency.py --strict; then
  echo "[FAIL] 文档与产物不一致（见上）"
  rc=1
fi

echo
echo "=============================================================="
echo "3/3  支撑产物是否齐（判据只认 results/ 里的东西）"
echo "=============================================================="
missing=0
need=(
  "results/m1.json" "results/m1_truth.json" "results/m2.json"
  "results/s1_depth.json" "results/slam1.json" "results/slam1_s5.json"
  "results/graph1.json" "results/seg1.json" "results/seg2.json"
  "results/p1_world.log" "results/p61_selftest.log"
)
for f in "${need[@]}"; do
  if [ -f "$f" ]; then
    printf '  OK    %s\n' "$f"
  else
    printf '  MISS  %s\n' "$f"; missing=$((missing+1)); rc=1
  fi
done

# 按类计数（只报数量，判定在 accept 里做）
n_demo=$(ls results/p6_demo_*.json 2>/dev/null | wc -l)
# 产物里 dry 和真跑混在一起（实测 64 份 = 28 dry + 36 真跑）。只报总数会让人以为跑了 64 次真机，
# 所以分开数：判据只认真跑（dry=false）。
n_real=$($PY - <<'PY' 2>/dev/null || echo 0
import glob, json
n = 0
for f in glob.glob("results/p6_demo_*.json"):
    try:
        if not json.load(open(f)).get("dry"):
            n += 1
    except Exception:
        pass
print(n)
PY
)
n_dyn=$(ls results/p6_dyn_*.json 2>/dev/null | wc -l)
n_probe=$(ls results/avoid_probe_*.json 2>/dev/null | wc -l)
n_npz=$(ls results/depth_dumps/*.npz 2>/dev/null | wc -l)
n_bench=$(ls results/bench_llm/bench_*_constrained.csv 2>/dev/null | wc -l)
printf '  ----  p6_demo=%s（真跑 %s / dry %s）  p6_dyn=%s  avoid_probe=%s  depth_dumps=%s  bench=%s\n' \
  "$n_demo" "${n_real:-0}" "$((n_demo - ${n_real:-0}))" "$n_dyn" "$n_probe" "$n_npz" "$n_bench"
for pair in "p6_demo_real:${n_real:-0}:10" "p6_dyn:$n_dyn:1" "avoid_probe:$n_probe:1" \
            "depth_dumps:$n_npz:10" "constrained_bench:$n_bench:1"; do
  name=${pair%%:*}; rest=${pair#*:}; cnt=${rest%%:*}; min=${rest##*:}
  if [ "${cnt:-0}" -lt "$min" ]; then
    echo "[MISS] $name 只有 $cnt 份（至少需要 $min）"; rc=1
  fi
done

echo
echo "=============================================================="
if [ "$rc" = "0" ]; then
  echo "交付自检：通过（16 条判据记账 + 文档一致 + 产物齐）"
else
  echo "交付自检：未通过（上面标 FAIL/MISS 的项）"
fi
echo "真跑复现见 docs/DELIVERY.md §1；限制清单见 docs/LIMITATIONS.md"
echo "=============================================================="
exit $rc
