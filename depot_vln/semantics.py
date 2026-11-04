"""货位语义状态库（SPEC §7.1 T2 / §7.2 属性）。

一条规则：**只有观测能改这里的值**。ground_truth 永不进入这个模块。
置信度随时间衰减；再次观测刷新；冲突时保留双方并标 stale。
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

ROOT = Path(__file__).resolve().parents[1]
HALF_LIFE_S = 1800.0  # 置信度半衰期 30 分钟（仓库一次巡检的时间尺度）


@dataclass
class SlotState:
    address: str
    occupied: bool
    cls: str | None
    confidence: float
    last_seen: float
    source: str = "vision"
    stale: bool = False
    conflicts: list[dict[str, Any]] = field(default_factory=list)

    def effective_confidence(self, now: float, half_life: float = HALF_LIFE_S) -> float:
        if self.last_seen <= 0:
            return self.confidence
        age = max(0.0, now - self.last_seen)
        return self.confidence * (0.5 ** (age / half_life))


class SemanticMap:
    def __init__(self, slot_grid: dict[str, Any], now: float = 0.0):
        self.grid = slot_grid
        self.slots: dict[str, SlotState] = {}
        self.now = now
        self._bays = {(b["aisle"], b["bay"]): b for b in slot_grid["bays"]}

    # ------------------------------------------------------------ write
    def observe(
        self,
        address: str,
        occupied: bool,
        cls: str | None,
        confidence: float,
        t: float,
        source: str = "vision",
    ) -> SlotState:
        self.now = max(self.now, t)
        prev = self.slots.get(address)
        if prev is not None and prev.occupied != occupied:
            # 冲突：双方都保留，谁新谁当 current，旧的进 conflicts 并标 stale
            prev.conflicts.append(
                {
                    "occupied": prev.occupied,
                    "cls": prev.cls,
                    "confidence": prev.confidence,
                    "at": prev.last_seen,
                }
            )
            prev.stale = True
        st = SlotState(
            address=address,
            occupied=occupied,
            cls=cls,
            confidence=max(0.0, min(1.0, confidence)),
            last_seen=t,
            source=source,
            stale=prev is not None and prev.occupied != occupied,
            conflicts=prev.conflicts if prev else [],
        )
        self.slots[address] = st
        return st

    # ------------------------------------------------------------ read
    def get(self, address: str) -> SlotState | None:
        return self.slots.get(address)

    def observed(self) -> Iterable[SlotState]:
        return self.slots.values()

    def coverage(self) -> float:
        return len(self.slots) / max(1, len(self.grid["slots"]))

    def bay_summary(self, aisle: str, bay: int) -> dict[str, Any]:
        b = self._bays[(aisle, bay)]
        seen = [self.slots[a] for a in b["slots"] if a in self.slots]
        counts: dict[str, int] = {}
        occ_slots = []
        for s in seen:
            if s.occupied and s.cls:
                counts[s.cls] = counts.get(s.cls, 0) + 1
                occ_slots.append(s)
        return {
            "aisle": aisle,
            "bay": bay,
            "observed_slots": len(seen),
            "occupied_slots": len(occ_slots),
            "empty_slots": sum(1 for s in seen if not s.occupied),
            "class_counts": counts,
            "confidence": min((s.effective_confidence(self.now) for s in occ_slots), default=0.0),
            "any_stale": any(s.stale for s in seen),
        }

    def rack_face_counts(self, aisle: str, side: str) -> dict[str, int]:
        counts: dict[str, int] = {}
        for a, s in self.slots.items():
            if s.occupied and s.cls and a.startswith(f"{aisle}-{side}-"):
                counts[s.cls] = counts.get(s.cls, 0) + 1
        return counts

    def rank_rack_faces_by_class(self, classes: set[str]) -> list[dict[str, Any]]:
        out: list[dict[str, Any]] = []
        for aisle in self.grid["aisles"]:
            for side in ("L", "R"):
                c = self.rack_face_counts(aisle["name"], side)
                total = sum(v for k, v in c.items() if k in classes)
                out.append(
                    {
                        "rack_face": f"{aisle['name']}-{side}",
                        "aisle": aisle["name"],
                        "side": side,
                        "count": total,
                        "class_counts": c,
                    }
                )
        out.sort(key=lambda r: (-r["count"], r["rack_face"]))
        return out

    def empty_slots(self, aisle: str | None = None) -> list[str]:
        return sorted(
            a
            for a, s in self.slots.items()
            if not s.occupied and (aisle is None or a.startswith(f"{aisle}-"))
        )

    def to_json(self, path: str | Path | None = None) -> dict[str, Any]:
        obj = {
            "schema": "depot-vln/semantic_map@1",
            "now": self.now,
            "coverage": round(self.coverage(), 4),
            "slots": {
                a: {
                    "occupied": s.occupied,
                    "class": s.cls,
                    "confidence": round(s.effective_confidence(self.now), 4),
                    "raw_confidence": round(s.confidence, 4),
                    "last_seen": round(s.last_seen, 3),
                    "source": s.source,
                    "stale": s.stale,
                    "conflicts": s.conflicts,
                }
                for a, s in sorted(self.slots.items())
            },
        }
        if path:
            Path(path).parent.mkdir(parents=True, exist_ok=True)
            Path(path).write_text(json.dumps(obj, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        return obj


def load_slot_grid(path: str | Path | None = None) -> dict[str, Any]:
    p = Path(path) if path else ROOT / "world" / "slot_grid.json"
    return json.loads(p.read_text(encoding="utf-8"))
