#!/usr/bin/env python3
"""Demo 1 — one sentence -> slots -> query -> candidates -> plan. **Offline, no ROS/GPU.**

This is the whole "language layer" without the graph: `RuleNLU` extracts slots,
`compile_query` (called by the NLU) decides what kind of query it is, and
`WorldModel.resolve()` turns that into zero or more candidate targets. Only when
there is exactly one candidate do we ask for a plan.

Run from the repository root:

    .venv/bin/python examples/01_query_to_plan.py
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from depot_vln.agent.nlu import RuleNLU          # noqa: E402
from depot_vln.sim import load_sim_world          # noqa: E402


def target_name(target: dict) -> str:
    kind = target.get("kind")
    if kind == "slot":
        return f"slot {target['address']}"
    if kind == "rack_face":
        return f"rack face {target['aisle']}-{target['side']}"
    if kind == "aisle":
        return f"aisle {target['aisle']}"
    return str(target)


def main() -> int:
    nlu = RuleNLU()
    # Explicit stand-in inventory: ground-truth sampled, provenance = sim-sampled.
    world = load_sim_world()

    utterances = [
        "去 B-L-07-2",                                   # complete address
        "去 10 号位",                                     # underdetermined -> many candidates
        "去 A 巷最里面那个空位",                            # relation + aisle
        "去 A 巷 11 号位第一层放油桶的那格",                # class + bay -> legitimately empty
        "去 A 巷放油桶的那面货架",                          # class + aisle -> one rack face
    ]

    for utterance in utterances:
        out = nlu.understand(utterance, {})
        query = out["query"]
        candidates, reason = world.resolve(query)

        print(f"\n> {utterance}")
        print(f"  query      : {query}")
        if reason:
            print(f"  verdict    : rejected ({reason})")
            continue

        print(f"  candidates : {len(candidates)}")
        for cand in candidates[:3]:
            print(f"    - {target_name(cand['target']):28s} "
                  f"score={cand['score']:<4} reachable={cand['reachable']}")
        if len(candidates) > 3:
            print(f"    ... {len(candidates) - 3} more")

        if len(candidates) == 1:
            plan = world.plan(candidates[0]["target"])
            if plan is None:
                print("  plan       : UNREACHABLE")
            else:
                print(f"  plan       : {plan['path_length']:.1f} m, "
                      f"eta {plan['eta']:.1f} s, entry={plan['entry']['type']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
