"""确定性"地面真值"生成：每个货位的实际占用与类别。

运行时禁止读 ground_truth/；只有打分脚本（scripts/score_*）可以读。
种子来自 config/world.yaml:seed，所以同一份 config 永远产生同一份真值。
"""
from __future__ import annotations

import math
import random
from typing import Any

# 闭集类别权重（世界生成器定义类别集合，P5 先闭集、开集作为 stretch）
CLASS_WEIGHTS = {
    "carton_m": 0.30,
    "carton_s": 0.22,
    "carton_l": 0.18,
    "tote": 0.12,
    "pallet": 0.10,
    "drum": 0.08,
}


def _pick_class(rng: random.Random) -> str:
    r = rng.random()
    acc = 0.0
    for cls, w in CLASS_WEIGHTS.items():
        acc += w
        if r <= acc:
            return cls
    return "carton_m"


def blocked_x_ranges(cfg: dict[str, Any], layout: dict[str, Any]) -> list[dict[str, Any]]:
    """把 `blocked:` 里的 bay 段换算成 X 区间（世界生成器据此放实体路障）。"""
    bays = {(b["aisle"], b["bay"]): b for b in layout["bays"]}
    out: list[dict[str, Any]] = []
    for seg in cfg.get("blocked") or []:
        first = bays[(seg["aisle"], int(seg["bay_from"]))]
        last = bays[(seg["aisle"], int(seg["bay_to"]))]
        aisle = next(a for a in layout["aisles"] if a["name"] == seg["aisle"])
        out.append(
            {
                "aisle": seg["aisle"],
                "bay_from": int(seg["bay_from"]),
                "bay_to": int(seg["bay_to"]),
                "x0": first["x0"],
                "x1": last["x1"],
                # 段两端各一道路障：单道只封一侧，从横通道另一头仍可进（生成即错的典型）
                "barrier_xs": [first["x0"] - 0.35, last["x1"] - 0.35],
                "center_y": aisle["center_y"],
                "width": aisle["width"],
                "reason": seg.get("reason", "BLOCKED"),
            }
        )
    return out


def build_truth(cfg: dict[str, Any], layout: dict[str, Any]) -> dict[str, Any]:
    seed = int(cfg.get("seed", 0))
    rng = random.Random(seed)
    sem = cfg["semantics"]
    p_occ = float(sem["occupancy_prob"])
    l2_scale = float(sem.get("level2_occ_scale", 1.0))
    shapes = {k: v.get("shape", "box") for k, v in sem["classes"].items()}
    sizes = {k: v["size"] for k, v in sem["classes"].items()}
    colors = {k: v["color"] for k, v in sem["classes"].items()}

    blocked = blocked_x_ranges(cfg, layout)

    def is_blocked(slot: dict[str, Any]) -> bool:
        for seg in blocked:
            if (
                slot["aisle"] == seg["aisle"]
                and seg["x0"] - 1e-9 <= slot["pose"]["x"] <= seg["x1"] + 1e-9
            ):
                return True
        return False

    slots: dict[str, dict[str, Any]] = {}
    cargo: list[dict[str, Any]] = []
    for slot in layout["slots"]:
        p = p_occ * (l2_scale if slot["level"] >= 2 else 1.0)
        occupied = rng.random() < p
        cls = _pick_class(rng) if occupied else None
        blocked_flag = is_blocked(slot)
        rec = {
            "address": slot["address"],
            "aisle": slot["aisle"],
            "side": slot["side"],
            "bay": slot["bay"],
            "level": slot["level"],
            "occupied": bool(occupied),
            "class": cls,
            "reachable": not blocked_flag,
            "reason_code": "BLOCKED" if blocked_flag else None,
            # 真值自身带满置信度；这是打分的参照，不是运行时观测
            "confidence": 1.0,
            "observed_at": 0.0,
            "source": "generator",
        }
        slots[slot["address"]] = rec
        if occupied:
            sz = sizes[cls]
            # 货箱贴着层板放，略微随机偏移（避免"排得太整齐"的假象）
            dx = rng.uniform(-0.10, 0.10)
            dy = rng.uniform(-0.08, 0.08)
            yaw = rng.uniform(-0.06, 0.06)
            cargo.append(
                {
                    "slot": slot["address"],
                    "class": cls,
                    "side": slot["side"],
                    "shape": shapes[cls],
                    "size": sz,
                    "color": colors[cls],
                    "pose": [
                        slot["pose"]["x"] + dx,
                        slot["pose"]["y"] + dy,
                        slot["deck_z"] + sz[2] / 2 + 0.005,
                        yaw,
                    ],
                }
            )

    # bay 级汇总（语言层的"属性/关系"查询要用聚合统计）
    per_bay: dict[str, dict[str, Any]] = {}
    for b in layout["bays"]:
        key = f"{b['aisle']}-{b['bay']:02d}"
        occ = [slots[a] for a in b["slots"] if slots[a]["occupied"]]
        counts: dict[str, int] = {}
        for s in occ:
            counts[s["class"]] = counts.get(s["class"], 0) + 1
        per_bay[key] = {
            "aisle": b["aisle"],
            "bay": b["bay"],
            "center_x": b["center_x"],
            "occupied_slots": len(occ),
            "empty_slots": len(b["slots"]) - len(occ),
            "class_counts": counts,
            "reachable": any(slots[a]["reachable"] for a in b["slots"]),
        }

    # 巷道/侧面的类别聚合 —— "去放纸箱最多的那排货架" 的真值答案
    per_rack_face: dict[str, dict[str, int]] = {}
    for addr, s in slots.items():
        if not s["occupied"]:
            continue
        key = f"{s['aisle']}-{s['side']}"
        per_rack_face.setdefault(key, {})
        per_rack_face[key][s["class"]] = per_rack_face[key].get(s["class"], 0) + 1

    carton_like = {c for c in CLASS_WEIGHTS if c.startswith("carton")}
    carton_totals = {
        k: sum(v.get(c, 0) for c in carton_like) for k, v in per_rack_face.items()
    }

    occupied_n = sum(1 for s in slots.values() if s["occupied"])
    return {
        "seed": seed,
        "slots": slots,
        "bays": per_bay,
        "rack_faces": per_rack_face,
        "carton_totals": carton_totals,
        "blocked": blocked,
        "cargo": cargo,
        "stats": {
            "n_slots": len(slots),
            "occupied": occupied_n,
            "occupancy_rate": round(occupied_n / max(1, len(slots)), 4),
            "n_cargo": len(cargo),
            "carton_faces": {k: v for k, v in sorted(carton_totals.items())},
        },
    }
