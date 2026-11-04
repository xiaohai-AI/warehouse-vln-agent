"""运行时不许读真值 —— 这是 **import 图层面**的性质，不靠自觉。"""
from __future__ import annotations

import ast
from pathlib import Path

from _source import code_lines

ROOT = Path(__file__).resolve().parents[1]
RUNTIME = [
    *sorted((ROOT / "depot_vln").glob("*.py")),
    *sorted((ROOT / "depot_vln" / "agent").rglob("*.py")),
]
# `depot_vln/eval/` 是**评测侧**：它就是要读真值来打分，不在此列。
# 明确允许碰真值的模块（都在"评测/打分/替身"这一侧，`sim.py` 就是替身装载器）
ALLOWED = {"sim.py", "scoring.py", "truth.py"}
FORBIDDEN = ("depot_vln.truth", "depot_vln.scoring", "depot_vln.sim", "ground_truth")


def _imports(path: Path) -> list[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    out: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module:
            out.append(node.module)
        elif isinstance(node, ast.Import):
            out.extend(a.name for a in node.names)
    return out


def test_runtime_modules_do_not_import_truth():
    offenders = []
    for p in RUNTIME:
        if p.name in ALLOWED:
            continue
        for mod in _imports(p):
            if any(mod == f or mod.startswith(f + ".") for f in FORBIDDEN[:3]):
                offenders.append(f"{p.relative_to(ROOT)}: import {mod}")
        for i, line in code_lines(p):
            if "ground_truth" in line:
                offenders.append(f"{p.relative_to(ROOT)}:{i}: 用到了 ground_truth 路径")
    assert not offenders, "运行时代码不得依赖真值：\n" + "\n".join(offenders)


def test_world_model_has_no_truth_attribute():
    from depot_vln.world import WorldModel

    assert not hasattr(WorldModel, "truth")
    assert not any("truth" in m for m in _imports(ROOT / "depot_vln" / "world.py"))
