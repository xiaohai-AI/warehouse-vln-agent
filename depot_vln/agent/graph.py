"""LangGraph 装配（SPEC §8.2 的图，一条不差地写进代码）。

边界即约束：
  * act 是唯一能调工具层的节点 —— "模型不许直发速度" 在这里是拓扑性质，不靠自觉
  * clarify 是唯一使用 interrupt 的地方，且只等"一句话"
  * 每个环都有上限：recursion_limit / MAX_RECOVER / MAX_CLARIFY
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

from langgraph.graph import END, START, StateGraph

from .nodes import (
    Runtime,
    make_act,
    make_clarify,
    make_plan,
    make_recover,
    make_respond,
    make_resolve,
    make_understand,
    make_verify,
    route_after_act,
    route_after_clarify,
    route_after_plan,
    route_after_recover,
    route_after_resolve,
    route_after_verify,
)
from .state import AgentState

RECURSION_LIMIT = 30  # 30 = 正常路径(~8 节点) + 2 次 clarify + 2 次 recover，跑飞即报错


def build_graph(tools, nlu, *, world=None, context: dict[str, Any] | None = None, checkpointer=None):
    rt = Runtime(tools=tools, nlu=nlu, world=world, context=context)
    g = StateGraph(AgentState)

    g.add_node("understand", make_understand(rt))
    g.add_node("resolve", make_resolve(rt))
    g.add_node("clarify", make_clarify(rt))
    g.add_node("plan", make_plan(rt))
    g.add_node("act", make_act(rt))
    g.add_node("verify", make_verify(rt))
    g.add_node("recover", make_recover(rt))
    g.add_node("respond", make_respond(rt))

    g.add_edge(START, "understand")
    g.add_edge("understand", "resolve")
    g.add_conditional_edges(
        "resolve",
        route_after_resolve,
        {"plan": "plan", "clarify": "clarify", "recover": "recover", "respond": "respond"},
    )
    g.add_conditional_edges("clarify", route_after_clarify, {"resolve": "resolve", "respond": "respond"})
    g.add_conditional_edges("plan", route_after_plan, {"act": "act", "respond": "respond"})
    g.add_conditional_edges("act", route_after_act, {"verify": "verify", "recover": "recover"})
    g.add_conditional_edges("verify", route_after_verify, {"respond": "respond", "recover": "recover"})
    g.add_conditional_edges("recover", route_after_recover, {"plan": "plan", "respond": "respond"})
    g.add_edge("respond", END)

    return g.compile(checkpointer=checkpointer)


def memory_checkpointer():
    from langgraph.checkpoint.memory import MemorySaver

    return MemorySaver()


def sqlite_checkpointer(path: str | Path = "state/depot_vln.sqlite"):
    """按 thread_id 持久化（本地，不上云 —— SPEC §8.1 可观测性）。"""
    from langgraph.checkpoint.sqlite import SqliteSaver

    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    import sqlite3

    conn = sqlite3.connect(str(p), check_same_thread=False)
    return SqliteSaver(conn)


def run_utterance(graph, utterance: str, *, thread_id: str, auto_clarify: str | None = None,
                  recursion_limit: int = RECURSION_LIMIT) -> dict[str, Any]:
    """跑一句话到 END；遇到 clarify 的 interrupt 用 auto_clarify 回答（基准用）。"""
    from langgraph.types import Command

    cfg = {"configurable": {"thread_id": thread_id}, "recursion_limit": recursion_limit}
    result = graph.invoke(
        {
            "thread_id": thread_id,
            "utterance": utterance,
            "history": [],
            "clarify_count": 0,
            "recover_count": 0,
            "alt_index": 0,
            "seq": 0,
            "trajectory": [],
            "node_trace": [],
            "notes": [],
        },
        cfg,
    )
    rounds = 0
    while "__interrupt__" in result and auto_clarify and rounds < 2:
        rounds += 1
        result = graph.invoke(Command(resume=auto_clarify), cfg)
    return result
