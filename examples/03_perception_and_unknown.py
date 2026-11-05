#!/usr/bin/env python3
"""Demo 3 — what "I have only seen 8% of the warehouse" looks like to the agent.

`results/inventory.json` is a **perception** snapshot (48 observed slots out of 576),
not ground truth. This script loads it through the same `WorldModel` the robot uses,
so you can watch the agent refuse to answer questions the map cannot support —
instead of guessing.

Run from the repository root:

    .venv/bin/python examples/03_perception_and_unknown.py
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from depot_vln.agent.nlu import RuleNLU  # noqa: E402
from depot_vln.inventory import Inventory  # noqa: E402
from depot_vln.world import load_world  # noqa: E402


def target_name(target: dict) -> str:
    kind = target.get("kind")
    if kind == "slot":
        return target["address"]
    if kind == "rack_face":
        return f"{target['aisle']}-{target['side']} face"
    return str(target.get("aisle"))


def main() -> int:
    inventory = Inventory.from_perception()          # results/inventory.json
    world = load_world(inventory)
    nlu = RuleNLU()

    total = len(world.grid["slots"])
    print(f"perception inventory : {len(inventory)} observed / {total} slots "
          f"({100 * len(inventory) / total:.1f}% coverage)")
    print(f"provenance           : {inventory.provenance.get('sources')}")

    questions = [
        "纸箱最多的是哪排货架",              # needs the whole search scope
        "去 A 巷放油桶的那面货架",           # only one aisle, but 24 unseen slots per face
        "去 A-L-01-1",                      # a direct address always resolves
    ]

    for question in questions:
        query = nlu.understand(question, {})["query"]
        candidates, reason = world.resolve(query)
        print(f"\n> {question}")
        print(f"  query : {query}")
        if reason:
            print(f"  verdict: {reason}  (refused, did not guess)")
            continue
        print(f"  candidates: {len(candidates)}")
        for cand in candidates[:3]:
            print(f"    - {target_name(cand['target']):16s} score={cand['score']}  why={cand['why']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
