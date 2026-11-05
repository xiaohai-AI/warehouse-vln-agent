"""地图生成 / 校验收口必须能 import。

2026-11-05：`scripts/depot_geometry.py` 在"清理一次性脚本"时被误删，但
`make world` / `make map` / `make map-check` 仍 import 它 —— 三条命令同时坏掉，
而没有任何断言会红。这个测试就是那条断言。
"""
from __future__ import annotations

import importlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_map_entrypoints_import_their_shared_geometry():
    for mod in ("scripts.map_check", "scripts.make_static_map"):
        assert importlib.import_module(mod) is not None, f"{mod} 无法 import"


def test_depot_vln_imports_follow_sys_path_setup():
    """脚本里的 `from depot_vln ...` 必须在 `sys.path.insert(仓库根)` 之后。

    踩过（2026-11-05）：static_map_pub / slam_route / m2_side_dock 在 sys.path 设置前
    import depot_vln。结果 Nav2 能起，但没有 /map 和 map->odom，planner 一直报
    "Invalid frame ID map"，车永远不动。
    """
    import re

    bad = []
    for path in sorted((ROOT / "scripts").glob("*.py")):
        lines = path.read_text(encoding="utf-8").splitlines()
        depot = [i for i, line in enumerate(lines, 1)
                 if re.match(r"\s*(from|import)\s+depot_vln\b", line)]
        if not depot:
            continue
        path_setup = [i for i, line in enumerate(lines, 1)
                      if re.search(r"sys\.path\.(insert|append)", line)]
        if not path_setup or min(path_setup) > min(depot):
            bad.append(f"{path.name}: depot_vln import at {min(depot)}, "
                       f"sys.path setup at {min(path_setup) if path_setup else 'missing'}")
    assert not bad, "脚本在 sys.path 设置前 import depot_vln：\n" + "\n".join(bad)
