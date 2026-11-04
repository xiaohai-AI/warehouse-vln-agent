"""打分（真值侧）—— **只给评测与演示用，图里永远看不到**。

这是运行时唯一允许读 `ground_truth/` 的地方旁边的文件：它不参与任何决策，
只回答"你到了目标之后看到的东西，与真值是否一致"。
`depot_vln/world.py` / `agent/` 都不 import 本模块（有测试检查）。
"""
from __future__ import annotations

import json
from functools import lru_cache
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]


@lru_cache(maxsize=1)
def truth_slots() -> dict[str, Any]:
    return json.loads((ROOT / "ground_truth" / "slots.json").read_text(encoding="utf-8"))["slots"]


def score_target(target: dict[str, Any], observation: dict[str, Any] | None = None) -> dict[str, Any]:
    """占用、类别**分开判**；观测缺失就留 None（不猜）。

    旧实现只返回 `matches_truth = (地址在真值表里存在)`，却叫 matches_truth ——
    实测会给出"真值 carton_m、观测 carton_s"还报 true 的假绿灯。
    """
    out: dict[str, Any] = {
        "slot_in_truth": False,
        "truth_occupied": None,
        "truth_class": None,
        "truth_reachable": None,
        "obs_occupied": None,
        "obs_class": None,
        "occupied_match": None,
        "class_match": None,
    }
    if target.get("kind") != "slot":
        out["slot_in_truth"] = True          # 巷/面级目标没有单点真值，不算失配
        return out
    gt = truth_slots().get(target.get("address"))
    if gt is None:
        return out
    out.update({
        "slot_in_truth": True,
        "truth_occupied": gt["occupied"],
        "truth_class": gt["class"],
        "truth_reachable": gt.get("reachable"),
    })
    if not observation:
        return out
    occ = observation.get("occupied")
    cls = observation.get("cls") or observation.get("class")
    out["obs_occupied"] = occ
    out["obs_class"] = cls
    if occ is not None:
        out["occupied_match"] = bool(occ) == bool(gt["occupied"])
    if cls and gt["occupied"]:
        out["class_match"] = str(cls) == str(gt["class"])
    return out


def score_observation(target: dict[str, Any], observation: dict[str, Any] | None = None) -> dict[str, Any]:
    return score_target(target, observation)
