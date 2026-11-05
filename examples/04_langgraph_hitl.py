#!/usr/bin/env python3
"""Demo 4 — LangGraph human-in-the-loop: interrupt, persist, resume in a **new process**.

This is the LangGraph-specific demo:

  1. `--step 1` runs an ambiguous utterance. `resolve` finds 24 candidates,
     the `clarify` node calls `interrupt()`, and the whole graph state is
     checkpointed into SQLite. The process exits while the graph is suspended.
  2. `--step 2 --answer "C 巷左边第一层"` starts a **new process**, loads the
     same `thread_id` from the same SQLite file, resumes with
     `Command(resume=...)`, and runs `resolve -> plan -> act -> verify -> respond`.

No ROS, no Gazebo, no LLM: the deterministic rule NLU and the offline tool layer
make the demo reproducible. **The Gazebo window is not used** — the offline tool
layer simulates motion in memory. For a LangGraph run that actually drives the
simulator, use Demo 2 in the GUI with an ambiguous utterance.

Usage (from the repository root):

    .venv/bin/python examples/04_langgraph_hitl.py --step 1
    .venv/bin/python examples/04_langgraph_hitl.py --step 2 --answer "C 巷左边第一层"

Step 1 resets the demo checkpoint by default (so it always starts at the interrupt);
pass --keep to inspect an existing checkpoint instead, or --reset explicitly.

Optional:

    --graph          print the compiled graph as Mermaid (step 1)
    --db PATH        checkpoint file (default: state/demo_hitl.sqlite)
    --reset          delete the checkpoint before step 1 (already the default)
    --keep           keep an existing checkpoint (do not start a fresh demo)
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from langgraph.types import Command  # noqa: E402

from depot_vln.agent.graph import build_graph, sqlite_checkpointer  # noqa: E402
from depot_vln.agent.nlu import RuleNLU  # noqa: E402
from depot_vln.agent.tools import OfflineToolLayer  # noqa: E402
from depot_vln.sim import load_sim_world  # noqa: E402

THREAD_ID = "demo4-langgraph-hitl"
UTTERANCE = "去 10 号位"          # 6 aisles × 2 sides × 2 levels = 24 candidates
ANSWER = "C 巷左边第一层"          # resolves to C-L-10-1


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


def make_graph(db: str):
    world = load_sim_world()
    graph = build_graph(
        OfflineToolLayer(world),
        RuleNLU(),
        world=world,
        checkpointer=sqlite_checkpointer(db),
    )
    return graph


def config() -> dict:
    return {"configurable": {"thread_id": THREAD_ID}, "recursion_limit": 30}


def step1(db: str, show_graph: bool, keep: bool) -> int:
    graph = make_graph(db)
    if show_graph:
        print("=== compiled graph (Mermaid) ===")
        print(graph.get_graph().draw_mermaid())
        print()

    # 默认已经重置过 DB；--keep 时如果还有旧 checkpoint，把状态说清楚，
    # 不要走 invoke 后返回终态、再误报 "图没有 interrupt"。
    if keep:
        snap = graph.get_state(config())
        if snap.values:
            if snap.next:
                print(f"该 thread 已有挂起的 checkpoint（next={snap.next}）。"
                      f"直接跑 --step 2，或用 --reset 重新开始。")
            else:
                print("该 thread 已经跑完（checkpoint 是终态）。"
                      "要重新演示请用 --step 1 --reset。")
            return 2

    state = graph.invoke({
        "thread_id": THREAD_ID,
        "utterance": UTTERANCE,
        "history": [],
        "clarify_count": 0,
        "recover_count": 0,
        "alt_index": 0,
        "seq": 0,
        "trajectory": [],
        "node_trace": [],
        "notes": [],
    }, config())

    if "__interrupt__" not in state:
        print("预期是歧义指令，但图没有 interrupt；请检查基准/NLU。")
        return 1

    payload = state["__interrupt__"][0].value
    print(f"utterance      : {UTTERANCE}")
    print(f"query          : {state.get('query')}")
    print(f"node trace     : {' > '.join(state.get('node_trace', []))}")
    print(f"interrupt      : {payload['question']}")
    print(f"candidates     : {len(payload['candidates'])} 个")
    print(f"checkpoint     : {Path(db).resolve()}")
    print()
    print("图已挂起，进程退出；下一步用新进程 resume：")
    print(f'  .venv/bin/python examples/04_langgraph_hitl.py --step 2 --answer "{ANSWER}"')
    return 0


def step2(db: str, answer: str) -> int:
    if not Path(db).exists():
        print(f"没有 checkpoint：{db}\n先跑 --step 1。")
        return 1

    graph = make_graph(db)
    snap = graph.get_state(config())
    if not snap.values:
        print(f"checkpoint 里没有 thread {THREAD_ID!r}；先跑 --step 1。")
        return 1
    if not snap.next:
        print("该 thread 已经跑完（checkpoint 是终态），没有挂起的 interrupt。")
        print("要重新演示请先跑 --step 1（默认会重置）。")
        return 2

    try:
        state = graph.invoke(Command(resume=answer), config())
    except Exception as exc:  # noqa: BLE001
        print(f"resume 失败（checkpoint 里没有挂起的 thread {THREAD_ID!r}？）：{exc}")
        return 1

    print(f"resume answer  : {answer}")
    print(f"node trace     : {' > '.join(state.get('node_trace', []))}")
    print(f"target         : {target_name(state)}")
    print(f"reason         : {state.get('reason_code')}")
    print(f"answer         : {state.get('answer')}")
    return 0 if state.get("reason_code") is None else 1


def main() -> int:
    ap = argparse.ArgumentParser(description="LangGraph interrupt -> persist -> resume demo")
    ap.add_argument("--step", type=int, choices=(1, 2), required=True)
    ap.add_argument("--answer", default=ANSWER, help="step 2 的澄清回答")
    ap.add_argument("--db", default=str(ROOT / "state" / "demo_hitl.sqlite"),
                    help="SQLite checkpoint 路径")
    ap.add_argument("--graph", action="store_true", help="step 1 额外打印 Mermaid 图")
    ap.add_argument("--reset", action="store_true",
                    help="step 1 前删除旧 checkpoint（默认就重置；保留此参数兼容）")
    ap.add_argument("--keep", action="store_true",
                    help="保留已有 checkpoint（step 1 不重置；用于观察已完成/挂起状态）")
    args = ap.parse_args()

    db = str(Path(args.db).expanduser())
    # step 1 默认从干净 checkpoint 开始；--keep 才保留旧的（用于观察/排查）。
    if args.step == 1 and (args.reset or not args.keep):
        for suffix in ("", "-wal", "-shm"):
            Path(db + suffix).unlink(missing_ok=True)

    if args.step == 1:
        return step1(db, args.graph, args.keep)
    return step2(db, args.answer)


if __name__ == "__main__":
    raise SystemExit(main())
