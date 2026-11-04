#!/usr/bin/env python3
"""调试：打印一条指令在图上每一步的状态（排查 use 用，不进交付物）。"""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from depot_vln.agent.graph import build_graph, memory_checkpointer
from depot_vln.agent.nlu import RuleNLU, parse_clarification
from depot_vln.agent.tools import MockToolLayer
from depot_vln.mock.world import load_mock_world

utt = sys.argv[1] if len(sys.argv) > 1 else "去 10 号位"
answer = sys.argv[2] if len(sys.argv) > 2 else "C 巷左边第一层"
print("utterance:", utt)
print("clarify answer:", answer, "->", parse_clarification(answer))

world = load_mock_world()
tools = MockToolLayer(world)
g = build_graph(tools, RuleNLU(), world=world, checkpointer=memory_checkpointer())
cfg = {"configurable": {"thread_id": "dbg"}, "recursion_limit": 30}
st = g.invoke({"thread_id": "dbg", "utterance": utt, "clarify_count": 0, "recover_count": 0,
               "alt_index": 0, "seq": 0, "trajectory": [], "node_trace": [], "notes": [], "history": []}, cfg)
print("step1 node_trace:", st.get("node_trace"))
print("step1 reason:", st.get("reason_code"), "query:", st.get("query"), "n_cands:", len(st.get("candidates") or []))
if "__interrupt__" in st:
    print("interrupt:", st["__interrupt__"][0].value)
    from langgraph.types import Command
    st = g.invoke(Command(resume=answer), cfg)
    print("step2 node_trace:", st.get("node_trace"))
    print("step2 constraints:", st.get("constraints"), "reason:", st.get("reason_code"))
    print("step2 target:", (st.get("target") or {}).get("address"))
    print("step2 n_cands:", len(st.get("candidates") or []))
print("answer:", st.get("answer"))
