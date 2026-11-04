#!/usr/bin/env python3
"""把感知产物汇总成库存快照 `results/inventory.json`（语义层的唯一运行时数据来源）。

输入（有哪个用哪个，全部是**感知输出**，不是真值）：
  results/seg2.json        SEG-2：双目深度判占用 + 类别（每个被拍到的货位一条）
  results/p6_demo_*.json   P6-2：到货位后 verify 的现场观测

输出 schema：depot-vln/inventory@1（见 depot_vln/inventory.py），
`provenance` 里写明来源与观测到的数量 —— 覆盖率是算出来的，不是宣称的。

用法
  python3.10 scripts/build_inventory.py                 # 汇总现有产物
  python3.10 scripts/build_inventory.py --out /tmp/i.json
"""
from __future__ import annotations

import argparse
import glob
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from depot_vln.inventory import Inventory  # noqa: E402


def from_seg2(path: Path) -> tuple[list[dict], dict]:
    obj = json.loads(path.read_text(encoding="utf-8"))
    rows = []
    for rec in obj.get("detail") or []:
        addr = rec.get("addr")
        pred = (rec.get("pred") or "").strip().lower()
        if not addr:
            continue
        # 检测器用 `empty` 作为哨兵标签（不是货类）。直接 bool(pred) 会把空位记成"有货" ——
        # 这正是"两个模型都说对，地图却是反的"那类错误的来源。
        occupied = pred not in ("", "empty", "none", "null", "-", "n/a")
        rows.append({
            "address": addr,
            "occupied": occupied,
            "class": pred if occupied else None,
            "confidence": 0.9 if occupied else 0.75,
            "observed_at": obj.get("now") or 0.0,
            "source": "seg2/perception",
        })
    return rows, {"source": str(path.relative_to(ROOT)), "n": len(rows)}


def from_p6_demos(pattern: str) -> tuple[list[dict], dict]:
    rows, n_files = [], 0
    for f in sorted(glob.glob(str(ROOT / pattern))):
        try:
            d = json.loads(Path(f).read_text(encoding="utf-8"))
        except Exception:
            continue
        if d.get("dry"):
            continue
        obs = d.get("observation") or {}
        addr = (d.get("target") or {}).get("address")
        if not addr or not obs.get("ok") or obs.get("occupied") is None:
            continue
        n_files += 1
        rows.append({
            "address": addr,
            "occupied": bool(obs["occupied"]),
            "class": obs.get("cls"),
            "confidence": float(obs.get("confidence") or 0.7),
            "observed_at": float(d.get("t") or 0.0),
            "source": "p6-demo/verify",
        })
    return rows, {"source": pattern, "n": len(rows), "files": n_files}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "results" / "inventory.json"))
    args = ap.parse_args()

    sources: list[dict] = []
    rows: list[dict] = []

    p = ROOT / "results" / "seg2.json"
    if p.exists():
        r, meta = from_seg2(p)
        rows += r
        sources.append(meta)
    else:
        print("[inventory] 没有 results/seg2.json（跳过 SEG-2）")

    r, meta = from_p6_demos("results/p6_demo_*.json")
    if meta["n"]:
        rows += r
        sources.append(meta)

    if not rows:
        print("[inventory] 没有任何感知产物 —— 不生成快照（运行时应该显式用 sim 替身）")
        return 1

    inv = Inventory.from_rows(rows, provenance={
        "provider": "perception",
        "sources": sources,
        "built_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "note": "只包含真实观测过的货位；未观测的货位在运行时是 'unknown'，不是 '空'",
    }, now=time.time())
    obj = inv.to_json(args.out)
    total = len(json.loads((ROOT / "world" / "slot_grid.json").read_text(encoding="utf-8"))["slots"])
    print(f"[inventory] {obj['n_observed']} 个货位观测 -> {args.out}")
    print(f"            覆盖率 {obj['n_observed']}/{total} = {obj['n_observed'] / total:.1%}")
    for s in sources:
        print(f"            来源 {s}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
