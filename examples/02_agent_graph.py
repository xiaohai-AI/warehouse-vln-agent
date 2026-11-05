#!/usr/bin/env python3
"""Demo 2 — run the full LangGraph agent on natural-language commands. **Offline, no ROS/GPU.**

The graph is `understand -> resolve -> [clarify] -> plan -> act -> verify -> respond`.
Ambiguous commands raise a LangGraph `interrupt`; this script answers it with
`Command(resume=...)`, exactly like a UI/voice front-end would.

Run from the repository root:

    .venv/bin/python examples/02_agent_graph.py
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from langgraph.types import Command  # noqa: E402

from depot_vln.agent.graph import build_graph, memory_checkpointer  # noqa: E402
from depot_vln.agent.nlu import RuleNLU  # noqa: E402
from depot_vln.agent.tools import OfflineToolLayer  # noqa: E402
from depot_vln.sim import load_sim_world  # noqa: E402


def target_name(state: dict) -> str:
    target = state.get("target") or {}
    kind = target.get("kind")
    if kind == "slot":
        return target["address"]
    if kind == "rack_face":
        return f"{target['aisle']}-{target['side']}"
    if kind == "aisle":
        return target["aisle"]
    return "-"


def run_case(utterance: str, answer: str | None = None) -> dict:
    world = load_sim_world()
    graph = build_graph(
        OfflineToolLayer(world), RuleNLU(), world=world,
        checkpointer=memory_checkpointer(),
    )
    config = {"configurable": {"thread_id": "demo"}, "recursion_limit": 30}
    state = graph.invoke({
        "thread_id": "demo", "utterance": utterance, "history": [],
        "clarify_count": 0, "recover_count": 0, "alt_index": 0, "seq": 0,
        "trajectory": [], "node_trace": [], "notes": [],
    }, config)

    while "__interrupt__" in state:
        payload = state["__interrupt__"][0].value
        print(f"  agent asks : {payload['question']}")
        print(f"  user says  : {answer}")
        state = graph.invoke(Command(resume=answer), config)
        answer = None
    return state


def main() -> int:
    cases = [
        ("去 10 号位", "C 巷左边第一层"),   # ambiguous -> clarify -> drive
        ("去 B-L-07-2", None),              # direct
        ("去 F-L-20-1", None),              # behind a barrier -> BLOCKED
    ]

    for utterance, answer in cases:
        print(f"\n> {utterance}")
        state = run_case(utterance, answer)
        print(f"  node trace : {' > '.join(state.get('node_trace', []))}")
        print(f"  target     : {target_name(state)}")
        print(f"  reason     : {state.get('reason_code')}")
        print(f"  answer     : {state.get('answer')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
