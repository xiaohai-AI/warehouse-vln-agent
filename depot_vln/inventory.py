"""库存快照（Inventory）—— 语义层的**唯一数据来源**。

这一层要解决的问题
------------------
重构前，`depot_vln/mock/world.py` 的 `_scan()` 直接遍历 `ground_truth/slots.json`，
把真值抄进 `SemanticMap`，然后对外宣称"模拟了一次巡检"。后果：

  * 运行时（agent 的工具层）事实上在读真值 —— 与 SPEC 里"运行时禁止读 ground_truth"
    的硬约束相矛盾，只是被"mock"这个名字掩盖了；
  * 任何"语义地图准不准"的结论都失去意义：地图本来就是真值；
  * 没有地方能表达"这个货位我还没看过" —— 每个 slot 都有确定答案。

现在分成三件事，边界是类型而不是自觉：

    Inventory          一次观测的结果快照（每个货位：看过/没看过、占用、类别、置信度、时间）
    SemanticMap        对 Inventory 做时间衰减/冲突合并后的查询视图（depot_vln/semantics.py）
    SimInventory       显式的**替身**：在没跑感知时从世界生成器采样，provenance 里写明
                       `provider=sim-sampled`，任何报告都必须带着这个标签。

`Inventory` 本身不 import 任何 truth 模块：采样所需的真值行由调用方传入。
"""
from __future__ import annotations

import json
import random
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Mapping

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PATH = ROOT / "results" / "inventory.json"
SCHEMA = "depot-vln/inventory@1"


@dataclass(frozen=True)
class InventoryEntry:
    """一个货位的一次观测。`occupied=None` 表示看过但判不出占用（与"没看过"不同）。"""

    address: str
    occupied: bool | None
    cls: str | None = None
    confidence: float = 0.0
    observed_at: float = 0.0
    source: str = "vision"
    stale: bool = False

    @property
    def known(self) -> bool:
        return self.occupied is not None


@dataclass
class Inventory:
    """库存快照 + 出处。出处不是装饰：bench 与报告都要按它分组，不能混着报。"""

    entries: dict[str, InventoryEntry] = field(default_factory=dict)
    provenance: dict[str, Any] = field(default_factory=dict)

    # ------------------------------------------------------------ 查询
    def get(self, address: str) -> InventoryEntry | None:
        return self.entries.get(address)

    def known(self) -> dict[str, InventoryEntry]:
        return {a: e for a, e in self.entries.items() if e.known}

    def observed(self) -> dict[str, InventoryEntry]:
        return dict(self.entries)

    def coverage(self, total: int | None = None) -> float:
        if total:
            return len(self.known()) / max(1, total)
        return 0.0

    def __len__(self) -> int:
        return len(self.entries)

    def __contains__(self, address: str) -> bool:
        return address in self.entries

    # ------------------------------------------------------------ 构造
    @classmethod
    def from_rows(
        cls,
        rows: Iterable[Mapping[str, Any]],
        *,
        provenance: Mapping[str, Any] | None = None,
        now: float | None = None,
    ) -> "Inventory":
        entries: dict[str, InventoryEntry] = {}
        t = time.time() if now is None else now
        for r in rows:
            addr = r.get("address") or r.get("addr")
            if not addr:
                continue
            occ = r.get("occupied")
            if occ is not None:
                occ = bool(occ)
            entries[str(addr).upper()] = InventoryEntry(
                address=str(addr).upper(),
                occupied=occ,
                cls=r.get("class") or r.get("cls"),
                confidence=float(r.get("confidence", 0.9 if occ is not None else 0.0)),
                observed_at=float(r.get("observed_at", r.get("t", t))),
                source=str(r.get("source", "vision")),
                stale=bool(r.get("stale", False)),
            )
        return cls(entries=entries, provenance=dict(provenance or {"provider": "unknown"}))

    @classmethod
    def from_perception(cls, path: str | Path | None = None) -> "Inventory":
        """读感知产出的库存快照（`results/inventory.json`）。缺文件就明确报错，不悄悄兜底。"""
        p = Path(path) if path else DEFAULT_PATH
        if not p.exists():
            raise FileNotFoundError(
                f"没有库存快照 {p}。先跑 `make inventory`（把 SEG-2 / verify 的观测汇总成快照），"
                "或用 `--inventory sim` 显式使用替身。"
            )
        obj = json.loads(p.read_text(encoding="utf-8"))
        rows = obj.get("slots") or obj.get("entries") or []
        if isinstance(rows, dict):
            rows = [dict(v, address=k) for k, v in rows.items()]
        prov = dict(obj.get("provenance") or {"provider": "perception"})
        prov.setdefault("file", str(p))
        return cls.from_rows(rows, provenance=prov, now=obj.get("now"))

    @classmethod
    def sampled(
        cls,
        truth_rows: Iterable[Mapping[str, Any]],
        *,
        coverage: float = 1.0,
        noise: float = 0.0,
        seed: int = 0,
        now: float | None = None,
        label: str = "sim-sampled",
    ) -> "Inventory":
        """**替身**：没跑感知时，按 coverage 采样世界生成器的占用/类别，供离线开发与语言层评测。

        明确标注（`provenance.provider = sim-sampled`）：它不是感知结果，不能当作
        "我的语义地图有多准"的证据。`coverage<1` 时未采样的货位保持"没看过"，
        这样上层"未知"路径在离线也能被跑到。
        """
        rng = random.Random(seed)
        t = time.time() if now is None else now
        rows: list[dict[str, Any]] = []
        n_total = 0
        for r in truth_rows:
            addr = r.get("address") or r.get("addr")
            if not addr:
                continue
            n_total += 1
            if rng.random() > coverage:
                continue
            occ = bool(r["occupied"])
            if noise > 0 and rng.random() < noise:
                occ = not occ
            cls_name = r.get("class") if occ else None
            rows.append(
                {
                    "address": addr,
                    "occupied": occ,
                    "class": cls_name,
                    "confidence": max(0.35, 0.95 - rng.uniform(0, 0.25) * (1 if noise else 0)),
                    "observed_at": t,
                    "source": "sim-sampled",
                }
            )
        inv = cls.from_rows(rows, provenance={
            "provider": label,
            "coverage": round(len(rows) / max(1, n_total), 4),
            "noise": noise,
            "seed": seed,
            "note": "替身数据：来自世界生成器采样，不是感知结果",
        }, now=t)
        return inv

    # ------------------------------------------------------------ 合并 / 输出
    def merge(self, other: "Inventory", *, prefer: str = "newest") -> "Inventory":
        out = dict(self.entries)
        for addr, e in other.entries.items():
            prev = out.get(addr)
            if prev is None or (prefer == "newest" and e.observed_at >= prev.observed_at):
                out[addr] = e
        prov = dict(self.provenance)
        prov["merged_with"] = other.provenance
        return Inventory(entries=out, provenance=prov)

    def to_json(self, path: str | Path | None = None) -> dict[str, Any]:
        obj = {
            "schema": SCHEMA,
            "provenance": self.provenance,
            "now": max((e.observed_at for e in self.entries.values()), default=0.0),
            "n_observed": len(self.entries),
            "n_known": len(self.known()),
            "slots": {
                a: {
                    "occupied": e.occupied,
                    "class": e.cls,
                    "confidence": round(e.confidence, 4),
                    "observed_at": round(e.observed_at, 3),
                    "source": e.source,
                    "stale": e.stale,
                }
                for a, e in sorted(self.entries.items())
            },
        }
        if path:
            p = Path(path)
            p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(json.dumps(obj, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        return obj

    def summary(self) -> dict[str, Any]:
        occ = [e for e in self.entries.values() if e.known and e.occupied]
        return {
            "provider": self.provenance.get("provider", "?"),
            "observed": len(self.entries),
            "known": len(self.known()),
            "occupied": len(occ),
            "classes": sorted({e.cls for e in occ if e.cls}),
        }
