"""在基准上跑完整的 LangGraph，产出分类别结果与状态转移记录。

判据（与重构前一致，但样本换了、来源独立）：
  * 歧义指令的**静默猜错 = 0**：可以发问，不许不问就给一个目标；
  * 不可达指令必须给原因码（BLOCKED / NO_SLOT / EMPTY_BAY / UNOBSERVED），不许硬走；
  * 每条都记 node_trace，可回放。

"静默猜错"的精确定义：
  * 期望 clarify，但 agent 一次都没问、直接给了一个目标；
  * 期望 reject，但 agent 真的开过去了（reach）。
  —— "给了别的货位" 算 *错*，不算 *静默*；两者分开报，因为没有区分度的总分没有意义。
"""
from __future__ import annotations

import csv
import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

from depot_vln.agent.graph import build_graph, memory_checkpointer
from depot_vln.agent.tools import OfflineToolLayer
from depot_vln.eval.oracle import load_cases, validate_all
from depot_vln.inventory import Inventory
from depot_vln.sim import sim_inventory
from depot_vln.world import load_world

ROOT = Path(__file__).resolve().parents[2]
CATEGORIES = ("direct", "attribute", "ambiguous", "unreachable")


def target_str(t: dict[str, Any] | None) -> str:
    if not t:
        return ""
    kind = t.get("kind")
    if kind == "slot":
        return str(t.get("address"))
    if kind == "rack_face":
        return f"{t.get('aisle')}-{t.get('side')}"
    if kind == "aisle":
        return str(t.get("aisle"))
    return json.dumps(t, ensure_ascii=False)


@dataclass
class Row:
    id: str
    category: str
    utterance: str
    expected: str
    actual: str
    ok: bool
    silent_wrong: bool
    reason: str = ""
    clarify: int = 0
    backend: str = ""
    query: str = ""
    dropped: list[str] = field(default_factory=list)
    note: str = ""


def _behavior(state: dict[str, Any], asked: bool) -> str:
    """终态行为。澄清过但最后到达了，就是 reach（`asked` 另行记录，用于判"有没有问"）。"""
    if state.get("reason_code"):
        return "reject"
    if state.get("observation"):
        return "reach"
    if asked:
        return "clarify"
    return "none"


def run_case(case: dict[str, Any], *, inventory: Inventory, nlu, world_factory=None) -> Row:
    world = (world_factory or (lambda: load_world(inventory)))()
    tools = OfflineToolLayer(world)
    graph = build_graph(tools, nlu, world=world, checkpointer=memory_checkpointer())
    tid = f"bench-{case['id']}"
    cfg = {"configurable": {"thread_id": tid}, "recursion_limit": 30}
    init = {
        "thread_id": tid, "utterance": case["utterance"], "history": [],
        "clarify_count": 0, "recover_count": 0, "alt_index": 0, "seq": 0,
        "trajectory": [], "node_trace": [], "notes": [],
    }
    state = graph.invoke(init, cfg)
    rounds = 0
    exp = case.get("expect") or {}
    from langgraph.types import Command

    while "__interrupt__" in state and exp.get("answer") and rounds < 2:
        rounds += 1
        state = graph.invoke(Command(resume=exp["answer"]), cfg)

    asked = rounds >= 1
    beh = _behavior(state, asked)
    actual = target_str(state.get("target"))
    reason = state.get("reason_code") or ""
    want = exp.get("behavior")
    empty = str(exp.get("reason") or "")

    ok = False
    silent = False
    note = ""
    if want == "reach":
        want_t = str(exp.get("target") or "")
        ok = beh == "reach" and actual == want_t
        note = "" if ok else f"期望到达 {want_t}"
    elif want == "clarify":
        after = str(exp.get("after_clarify") or "")
        ok = asked and beh == "reach" and (not after or actual == after)
        # "静默猜"只指**没问就自己定了**（开过去 / 给了目标）。
        # 拒绝回答（UNOBSERVED 等）是失败，但不是"静默猜"—— 两者分开报。
        silent = (not asked) and beh == "reach"
        note = "" if ok else ("没发问就直接给了目标" if silent else f"期望澄清后到 {after}")
    elif want == "reject":
        ok = beh == "reject" and reason == empty
        silent = beh == "reach"
        note = "" if ok else f"期望 {empty}，实际 {reason or beh}"

    return Row(
        id=str(case["id"]), category=str(case["category"]), utterance=str(case["utterance"]),
        expected=f"{want}:{exp.get('target') or exp.get('after_clarify') or exp.get('reason') or ''}",
        actual=f"{beh}:{actual or reason}", ok=ok, silent_wrong=silent,
        reason=reason, clarify=rounds,
        backend=str(state.get("backend") or getattr(nlu, "name", "?")),
        query=json.dumps(state.get("query") or {}, ensure_ascii=False),
        dropped=list(state.get("dropped") or []), note=note,
    )


def run_bench(
    *,
    backend: str = "rule",
    inventory: str = "sim",
    noise: float = 0.0,
    coverage: float = 1.0,
    out_csv: str | Path | None = None,
    cases_path: str | Path | None = None,
) -> dict[str, Any]:
    cases = load_cases(cases_path)
    bad = validate_all(cases)
    if bad:
        raise SystemExit("基准声明与真值矛盾，拒绝运行：\n" + json.dumps(bad, ensure_ascii=False, indent=2))

    if inventory == "perception":
        inv = Inventory.from_perception()
    else:
        inv = sim_inventory(coverage=coverage, noise=noise, seed=0)

    from depot_vln.agent.nlu import build_nlu

    nlu = build_nlu(backend)
    t0 = time.perf_counter()
    rows = [run_case(c, inventory=inv, nlu=nlu) for c in cases]
    wall = time.perf_counter() - t0

    per: dict[str, dict[str, Any]] = {}
    for cat in CATEGORIES:
        rs = [r for r in rows if r.category == cat]
        if not rs:
            continue
        per[cat] = {
            "n": len(rs),
            "ok": sum(r.ok for r in rs),
            "rate": round(sum(r.ok for r in rs) / len(rs), 4),
            "silent_wrong": sum(r.silent_wrong for r in rs),
        }
    n_ok = sum(r.ok for r in rows)
    summary = {
        "schema": "depot-vln/bench_result@2",
        "backend": backend,
        "inventory": inv.provenance,
        "n": len(rows),
        "ok": n_ok,
        "rate": round(n_ok / max(1, len(rows)), 4),
        "silent_wrong": sum(r.silent_wrong for r in rows),
        "per_category": per,
        "wall_s": round(wall, 2),
        "rows": [r.__dict__ for r in rows],
    }

    if out_csv:
        p = Path(out_csv)
        p.parent.mkdir(parents=True, exist_ok=True)
        with open(p, "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0].__dict__))
            w.writeheader()
            for r in rows:
                w.writerow(r.__dict__)
        Path(str(p).replace(".csv", ".json")).write_text(
            json.dumps({k: v for k, v in summary.items() if k != "rows"}, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
    return summary


def format_table(s: dict[str, Any]) -> str:
    lines = [
        f"backend={s['backend']}  inventory={s['inventory'].get('provider')}  n={s['n']}  "
        f"wall={s['wall_s']}s",
        f"{'类别':<14}{'n':>4}{'ok':>6}{'rate':>9}{'silent':>8}",
    ]
    for cat, v in s["per_category"].items():
        lines.append(f"{cat:<14}{v['n']:>4}{v['ok']:>6}{v['rate']:>9.1%}{v['silent_wrong']:>8}")
    lines.append(f"{'合计':<14}{s['n']:>4}{s['ok']:>6}{s['rate']:>9.1%}{s['silent_wrong']:>8}")
    return "\n".join(lines)
