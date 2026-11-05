"""图的端到端行为：每一条都是"结构性质"，不是提示词效果。"""
from __future__ import annotations

import pytest

from depot_vln.agent.graph import build_graph, memory_checkpointer
from depot_vln.agent.nlu import RuleNLU
from depot_vln.agent.tools import OfflineToolLayer
from depot_vln.inventory import Inventory
from depot_vln.sim import load_sim_world


def run(utterance: str, answer: str | None = None, world=None, **state_overrides):
    world = world or load_sim_world()
    graph = build_graph(OfflineToolLayer(world), RuleNLU(), world=world,
                        checkpointer=memory_checkpointer())
    cfg = {"configurable": {"thread_id": "t"}, "recursion_limit": 30}
    init = {
        "thread_id": "t", "utterance": utterance, "history": [],
        "clarify_count": 0, "recover_count": 0, "alt_index": 0, "seq": 0,
        "trajectory": [], "node_trace": [], "notes": [],
    }
    init.update(state_overrides)
    state = graph.invoke(init, cfg)
    asked = "__interrupt__" in state
    if asked and answer:
        from langgraph.types import Command

        state = graph.invoke(Command(resume=answer), cfg)
    return state, asked


def test_direct_reach():
    st, _ = run("去 B-L-07-2")
    assert st["target"]["address"] == "B-L-07-2"
    assert st["observation"]["ok"] is True
    assert st.get("reason_code") is None


def test_ambiguous_asks_then_resolves():
    st, asked = run("去 10 号位", answer="C 巷左边第一层")
    assert asked, "10 号位在 6 巷 × 2 侧 × 2 层都同名，必须发问"
    assert st["target"]["address"] == "C-L-10-1"


def test_blocked_target_refuses():
    st, _ = run("去 F-L-20-1")
    assert st["reason_code"] == "BLOCKED"
    assert "observation" not in st or st.get("observation") is None


def test_substitution_verifies_the_substitute_not_the_original():
    """换位后 verify 必须比对**换位目标**的旧观测。

    踩过：`candidates[0]` 还是原目标，原目标真值占用、替代位是空位时，
    换位明明成功却报 VERIFY_MISMATCH（离线可稳定复现）。
    """
    from dataclasses import replace

    from depot_vln.sim import sim_inventory
    from depot_vln.world import load_world

    inv = sim_inventory()
    entries = dict(inv.entries)
    entries["F-L-20-1"] = replace(entries["F-L-20-1"], occupied=True, cls="carton_l")
    entries["F-L-17-1"] = replace(entries["F-L-17-1"], occupied=False, cls=None)
    world = load_world(Inventory(entries=entries, provenance=inv.provenance))

    st, _ = run("去 F-L-20-1", world=world, allow_substitute=True)
    assert st.get("substituted_from") == "F-L-20-1"
    assert st["target"]["address"] == "F-L-17-1"
    assert st.get("reason_code") is None, st.get("notes")


def test_missing_slot_refuses():
    st, _ = run("去 Z-L-01-1")
    assert st["reason_code"] == "NO_SLOT"


def test_empty_bay_refuses():
    st, _ = run("去 A 巷 11 号位第一层放油桶的那格")
    assert st["reason_code"] == "EMPTY_BAY"


def test_unknown_object_is_not_guessed():
    st, _ = run("去 A 巷放冰箱的那面货架")
    assert st["reason_code"] == "NO_SLOT"


def test_partial_inventory_never_fabricates():
    """只观测了 8% 的货位时，"哪面有油桶"不能给一个确定答案。"""
    inv = Inventory.from_perception()
    world = load_sim_world()
    from depot_vln.world import load_world

    world = load_world(inv)
    st, asked = run("去 A 巷放油桶的那面货架", world=world)
    assert asked or st["reason_code"] in ("UNOBSERVED", "NO_SLOT", "AMBIGUOUS", "CLARIFY_LIMIT")
    assert st.get("observation") is None


def test_no_cmd_vel_path_outside_tool_layer():
    """图里只有 act 节点碰工具层 —— 这条在拓扑上成立（边就是这么连的）。"""
    from depot_vln.agent import graph as g

    src = open(g.__file__, encoding="utf-8").read()
    assert "cmd_vel" not in src
    nodes_src = open(g.__file__.replace("graph.py", "nodes.py"), encoding="utf-8").read()
    # act 之外没有工具调用
    assert nodes_src.count("rt.tools.navigate") == 1


def test_recursion_limit_is_explicit():
    from depot_vln.agent.graph import RECURSION_LIMIT

    assert 10 <= RECURSION_LIMIT <= 50
