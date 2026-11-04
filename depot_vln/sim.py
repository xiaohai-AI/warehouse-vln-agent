"""离线替身装载器 —— 唯一允许读 `ground_truth/` 的**开发**模块。

为什么单独一个文件
------------------
`depot_vln/world.py`（运行时世界模型）一个 truth 引用都没有，所以"运行时偷看真值"
在 import 图层面就是不可能的（`tests/test_no_truth_in_runtime.py` 会检查）。

但离线开发需要一个能跑的世界。这里把"从世界生成器采样一份库存"集中到一个文件，
并且**在 provenance 里写死 `provider=sim-sampled`**：任何用它跑出来的结论都必须
带着这个标签，不能混进"感知准确率"。
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from depot_vln.inventory import Inventory
from depot_vln.world import WorldModel, load_grid, load_world

ROOT = Path(__file__).resolve().parents[1]


def load_truth_slots() -> dict[str, Any]:
    return json.loads((ROOT / "ground_truth" / "slots.json").read_text(encoding="utf-8"))["slots"]


def sim_inventory(
    grid: dict[str, Any] | None = None,
    *,
    coverage: float = 1.0,
    noise: float = 0.0,
    seed: int = 0,
) -> Inventory:
    """从世界生成器采样一份**显式替身**库存（没有感知产物时的离线基线）。"""
    del grid  # 采样只需要真值表；参数留着是为了调用点明确"这是哪个世界的替身"
    rows = [dict(v, address=k) for k, v in load_truth_slots().items()]
    return Inventory.sampled(rows, coverage=coverage, noise=noise, seed=seed)


def load_sim_world(
    *,
    coverage: float = 1.0,
    noise: float = 0.0,
    seed: int = 0,
    blocked: list[dict[str, Any]] | None = None,
    start: tuple[float, float] | None = None,
) -> WorldModel:
    inv = sim_inventory(coverage=coverage, noise=noise, seed=seed)
    return load_world(inv, blocked=blocked, start=start)


__all__ = ["load_truth_slots", "sim_inventory", "load_sim_world", "WorldModel", "load_grid"]
