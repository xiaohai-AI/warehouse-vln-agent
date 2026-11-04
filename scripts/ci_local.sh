#!/usr/bin/env bash
# 离线判据链 —— 本地和 CI 跑的是**同一条命令**（CI 的 workflow 就调这个脚本，避免两处漂移）。
#
# 不需要 ROS / rclpy / Gazebo / GPU / 显示器，纯 Python + 已提交的产物。
# 需要的 Python 包见 requirements-ci.txt。
#
# 用法
#   bash scripts/ci_local.sh              # 跑全部 4 步
#   bash scripts/ci_local.sh --quiet      # 只在失败时打印
#
# 退出码：0 = 全绿；1 = 有判据 FAIL 或文档不一致。
set -o pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.." || exit 1
QUIET=0
[ "${1:-}" = "--quiet" ] && QUIET=1

# 解释器优先级：CI_PY（强制）→ 仓库自己的 .venv → $PY → python3.10
#   · 先 .venv：本地 shell 里残留的 `export PY=...` 指到别的解释器时会静默用错环境
#     （踩过：CI 脚本拿 /tmp/ci-venv 跑出 PASS 15/FAIL 1）。
#   · CI_PY：用来**强制**在隔离 venv 里验（模拟 runner），例如
#       PATH=/tmp/ci-venv2/bin:$PATH CI_PY=/tmp/ci-venv2/bin/python bash scripts/ci_local.sh
if [ -n "${CI_PY:-}" ]; then
  PY_BIN="$CI_PY"
elif [ -x .venv/bin/python ]; then
  PY_BIN=.venv/bin/python
elif [ -n "${PY:-}" ]; then
  PY_BIN="$PY"
else
  PY_BIN=python3.10
fi

FAILED=()
run() {                      # run <名字> <命令...>
  local name="$1"; shift
  echo ""
  echo "=============================================================="
  echo "== $name"
  echo "=============================================================="
  if [ "$QUIET" = "1" ]; then
    if ! "$@" >/tmp/ci_step.log 2>&1; then
      echo "  [FAIL] $name"; tail -20 /tmp/ci_step.log; FAILED+=("$name")
    else
      echo "  [PASS] $name"
    fi
  else
    "$@" || { echo "  [FAIL] $name"; FAILED+=("$name"); }
  fi
}

echo "Python: $("$PY_BIN" --version 2>&1)  →  $PY_BIN"
echo "仓库:   $(pwd)"

# 依赖预检：缺依赖时给一句人话，而不是让 make 里抛 ModuleNotFoundError 半屏
"$PY_BIN" - <<'PYEOF' || { echo ""; echo "✗ 依赖不全 —— 先跑： python3.10 -m pip install -r requirements-ci.txt"; exit 1; }
import importlib, sys
need = {"numpy": "深度审计", "yaml": "世界配置/布局", "langgraph": "P6-1 约束层自检（对抗端到端）",
        "pydantic": "Agent 层"}
missing = []
for mod, why in need.items():
    try:
        importlib.import_module(mod)
    except Exception:
        missing.append(f"{mod}（{why}）")
if missing:
    print("  缺: " + "、".join(missing))
    print(f"  当前解释器: {sys.executable}")
    sys.exit(1)
print("  依赖齐全: numpy / yaml / langgraph / pydantic")
PYEOF

# 1) 判据账本（会重写 docs/ACCEPTANCE.md，这是它的职责）
run "1/4 判据账本 make accept" make accept PY="$PY_BIN"

# 2) 文档与产物一致（9 项：合计=行数、判据版本、存档数…）
run "2/4 文档一致 make docs-check" make docs-check PY="$PY_BIN"

# 3) 交付自检（16 条判据记账 + 文档一致 + 产物齐，不硬编码 PASS 数）
run "3/4 交付自检 scripts/delivery_check.sh" bash scripts/delivery_check.sh

# 4) 前向安全层纯逻辑自检（7 例，秒级，不需要 ROS）
run "4/4 安全层自检 make guard-selftest" make guard-selftest PY="$PY_BIN"

echo ""
echo "=============================================================="
if [ ${#FAILED[@]} -eq 0 ]; then
  echo "离线判据链：全部通过（4/4 步）"
  grep -oE '合计：PASS [0-9]+ / FAIL [0-9]+ / 未跑 [0-9]+' docs/ACCEPTANCE.md 2>/dev/null || true
  echo "=============================================================="
  exit 0
fi
echo "离线判据链：${#FAILED[@]} 步失败 —— ${FAILED[*]}"
echo "=============================================================="
exit 1
