"""反硬编码回归守护。

这个测试存在的理由：重构前同一个世界事实被抄在 6+ 处（`list("ABCDEF")`、`24`、
`(-31.0, 0.0)`、`{"C": 0.55}`…）。删掉它们不难，难的是**不让它们长回来**。
所以把关口写成测试：`depot_vln/` 里出现这些字面量就红。
"""
from __future__ import annotations

import re
from pathlib import Path

from _source import code_lines

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "depot_vln"

# (正则, 说明) —— 只放"一旦出现就一定是硬编码"的模式，避免误伤
PATTERNS = [
    (re.compile(r"""(?<![\w.])-?31\.0\b"""), "写死的出生点 X（应从 catalog.spawn 派生）"),
    (re.compile(r"""["']ABCDEF["']"""), "写死的巷道表（应来自 config/world.yaml）"),
    (re.compile(r"""list\(\s*["']ABCDEF["']\s*\)"""), "写死的巷道表"),
    (re.compile(r"""bays_per_aisle\s*=\s*24|bays_per_aisle\s*==\s*24"""), "写死的 bay 数"),
    (re.compile(r"""LANE_DY\s*=\s*\{"""), "写死的车道偏移（应在 config/nav.yaml）"),
]

# 这些文件是"世界事实"的合法居所
EXEMPT = {
    "catalog.py",     # 世界事实的唯一入口
    "layout.py",      # 几何派生（LEVEL_SIDES 等）
    "sim.py",         # 替身装载器
    "scoring.py",     # 真值打分
    "truth.py",       # 世界生成器的真值导出
}


def _sources() -> list[Path]:
    return [p for p in PKG.rglob("*.py") if p.name not in EXEMPT]


def test_no_hardcoded_world_facts():
    hits = []
    for p in _sources():
        for i, line in code_lines(p):
            for pat, why in PATTERNS:
                if pat.search(line):
                    hits.append(f"{p.relative_to(ROOT)}:{i}: {why}\n    {line.strip()}")
    assert not hits, "发现硬编码的世界事实：\n" + "\n".join(hits)


def test_agent_does_not_name_sides_or_levels():
    """侧/层必须从 catalog 来：agent 目录里不许出现 ("L", "R") / (1, 2) 这类枚举。"""
    bad = []
    for p in (PKG / "agent").rglob("*.py"):
        for i, line in code_lines(p):
            if re.search(r"""\(\s*["']L["']\s*,\s*["']R["']\s*\)""", line):
                bad.append(f"{p.relative_to(ROOT)}:{i}: {line.strip()}")
            if re.search(r"""for\s+\w+\s+in\s+\(\s*1\s*,\s*2\s*\)""", line):
                bad.append(f"{p.relative_to(ROOT)}:{i}: {line.strip()}")
    assert not bad, "侧/层枚举写死在 agent 层：\n" + "\n".join(bad)
