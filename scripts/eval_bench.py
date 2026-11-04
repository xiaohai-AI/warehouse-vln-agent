#!/usr/bin/env python3
"""在基准上跑 agent，产出分类别成功率表 + 状态转移 JSONL（判据 LANG-1 / P0 闸门）。

用法
  python3.10 scripts/eval_bench.py                 # 规则 NLU + mock 工具层（离线，默认）
  python3.10 scripts/eval_bench.py --backend ollama  # qwen2.5:7b 抽槽位
  python3.10 scripts/eval_bench.py --repeat 3       # 每个阶段对比都取中位

判据
  * 歧义指令的"静默猜错"必须 = 0（可以问，不许猜）
  * 不可达指令必须给出原因码（BLOCKED / NO_SLOT / EMPTY_BAY），不能只回"失败"
  * 每条都写进 transitions/<run>.jsonl，可回放
"""
from __future__ import annotations

import argparse
import csv
import json
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from depot_vln.agent.graph import build_graph, memory_checkpointer  # noqa: E402
from depot_vln.agent.constraint import ConstrainedNLU  # noqa: E402
from depot_vln.agent.nlu import OllamaNLU, RuleNLU  # noqa: E402
from depot_vln.agent.tools import MockToolLayer  # noqa: E402
from depot_vln.mock.world import load_mock_world  # noqa: E402

CATEGORIES = ["direct", "attribute", "ambiguous", "unreachable"]


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


def run_one(row: dict[str, str], nlu, noise: float, seed: int, transitions: list[dict[str, Any]],
            run_id: str) -> dict[str, Any]:
    world = load_mock_world(noise=noise, seed=seed)
    tools = MockToolLayer(world)
    graph = build_graph(tools, nlu, world=world, checkpointer=memory_checkpointer())
    thread_id = f"{run_id}-{row['id']}"

    from langgraph.types import Command

    cfg = {"configurable": {"thread_id": thread_id}, "recursion_limit": 30}
    init = {
        "thread_id": thread_id,
        "utterance": row["utterance"],
        "history": [],
        "clarify_count": 0,
        "recover_count": 0,
        "alt_index": 0,
        "seq": 0,
        "trajectory": [],
        "node_trace": [],
        "notes": [],
    }
    t0 = time.perf_counter()
    state = graph.invoke(init, cfg)
    questions: list[str] = []
    rounds = 0
    while "__interrupt__" in state and row.get("clarify_answer") and rounds < 2:
        intr = state["__interrupt__"]
        try:
            payload = intr[0].value if isinstance(intr, (list, tuple)) else intr.value
            if isinstance(payload, dict):
                questions.append(str(payload.get("question", "")))
        except Exception:
            questions.append(str(intr))
        rounds += 1
        state = graph.invoke(Command(resume=row["clarify_answer"]), cfg)
    wall = time.perf_counter() - t0

    trace = state.get("node_trace") or []
    seq_ok = True
    seqs = [s.get("seq") for s in (state.get("trajectory") or []) if s.get("seq") is not None]
    if seqs != sorted(seqs) or len(set(seqs)) != len(seqs):
        seq_ok = False

    rc = state.get("reason_code")
    actual_target = target_str(state.get("target"))
    clarify_count = int(state.get("clarify_count") or 0)
    if clarify_count:
        behavior = "clarify"
    elif rc:
        behavior = "reject"
    elif state.get("observation"):
        behavior = "reach"
    else:
        behavior = "none"

    exp_beh = row["expected_behavior"]
    exp_target = row.get("expected_target") or ""
    exp_after = row.get("expected_after_clarify") or ""
    exp_rc = row.get("expected_reason_code") or ""

    if exp_beh == "reach":
        behavior_ok = behavior == "reach" and (not exp_target or actual_target == exp_target)
        silent_wrong = False
    elif exp_beh == "clarify":
        asked = clarify_count >= 1 and len(questions) >= 1
        converged = bool(exp_after) and actual_target == exp_after and rc is None
        behavior_ok = asked and (converged or not exp_after)
        # 静默猜错 = 没问就直接给了一个目标（这是唯一零容忍项）
        silent_wrong = (not asked) and bool(actual_target)
    else:  # reject
        behavior_ok = behavior == "reject" and (rc or "") == exp_rc
        silent_wrong = False

    step = {
        "run": run_id,
        "id": row["id"],
        "category": row["category"],
        "utterance": row["utterance"],
        "expected_behavior": exp_beh,
        "expected_target": exp_target,
        "expected_reason_code": exp_rc,
        "actual_behavior": behavior,
        "actual_target": actual_target,
        "actual_reason_code": rc or "",
        "behavior_ok": behavior_ok,
        "silent_wrong": silent_wrong,
        "clarify_rounds": clarify_count,
        "questions": questions,
        "seq_monotonic": seq_ok,
        "wall_s": round(wall, 3),
        "sim_s": round(sum(float(s.get("elapsed") or 0) for s in (state.get("trajectory") or [])), 2),
        "nodes": len(trace),
        "node_path": ">".join(trace),
        "answer": (state.get("answer") or "").replace("\n", " "),
    }
    for node in trace:
        transitions.append(
            {
                "run": run_id,
                "id": row["id"],
                "node": node,
                "utterance": row["utterance"],
                "reason_code": rc or "",
                "t_wall": round(wall, 3),
            }
        )
    return step


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--spec", default=str(ROOT / "results" / "bench_spec.csv"))
    ap.add_argument("--backend", default="rule",
                    choices=["rule", "ollama", "ollama-constrained"],
                    help="rule=确定性基线（LANG-1 记账口径）；ollama=裸模型（对照组）；"
                         "ollama-constrained=P6-1 约束层（实验组：模型只抽槽位 + 落地闸门）")
    ap.add_argument("--repeat", type=int, default=1)
    ap.add_argument("--noise", type=float, default=0.0, help="模拟感知噪声比例（默认 0：只衡量语言层）")
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--model", default="qwen2.5:7b",
                    help="ollama 模型名（--backend ollama / ollama-constrained 生效）")
    args = ap.parse_args()

    rows = list(csv.DictReader(open(args.spec, encoding="utf-8")))
    if args.limit:
        rows = rows[: args.limit]
    if args.backend == "ollama-constrained":
        nlu = ConstrainedNLU(model=args.model)
        slug = "".join(ch if (ch.isalnum() or ch == "-") else "_" for ch in args.model)
        out_csv = (ROOT / "results" / "bench_llm"
                   / f"bench_{datetime.now().strftime('%Y%m%d')}_{slug}_constrained.csv")
    elif args.backend == "ollama":
        nlu = OllamaNLU(model=args.model)
        # **LLM 后端的结果不写 results/ 根目录**：accept.py 用 latest("bench_2*.csv") 记账，
        # 混进根目录会把 LANG-1 的账记到"对比跑"上（曾差点用 --backend ollama 覆盖掉基线）。
        slug = "".join(ch if (ch.isalnum() or ch == "-") else "_" for ch in args.model)
        out_csv = ROOT / "results" / "bench_llm" / f"bench_{datetime.now().strftime('%Y%m%d')}_{slug}.csv"
    else:
        nlu = RuleNLU()
        out_csv = ROOT / "results" / f"bench_{datetime.now().strftime('%Y%m%d')}.csv"

    run_id = datetime.now().strftime("%Y%m%dT%H%M%S")
    all_steps: list[dict[str, Any]] = []
    transitions: list[dict[str, Any]] = []
    for rep in range(args.repeat):
        for row in rows:
            all_steps.append(run_one(row, nlu, args.noise, seed=1000 + rep, transitions=transitions, run_id=run_id))

    out_csv.parent.mkdir(parents=True, exist_ok=True)
    cols = list(all_steps[0].keys()) if all_steps else []
    with open(out_csv, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=cols)
        w.writeheader()
        for s in all_steps:
            row = dict(s)
            row["questions"] = " | ".join(row["questions"]) if isinstance(row["questions"], list) else row["questions"]
            w.writerow(row)

    tr_dir = ROOT / "transitions"
    tr_dir.mkdir(exist_ok=True)
    tr_file = tr_dir / f"{run_id}.jsonl"
    with open(tr_file, "w", encoding="utf-8") as fh:
        for rec in transitions:
            fh.write(json.dumps(rec, ensure_ascii=False) + "\n")

    # 分类别汇总（多轮取中位：这里按"是否成功"取多数的稳定表达 —— 成功率 + 中位耗时）
    tag = nlu.name if args.backend == "rule" else f"{nlu.name}:{args.model}"
    print(f"== benchmark run {run_id}  backend={tag}  noise={args.noise}  repeat={args.repeat} ==")
    print()
    print(f"{'类别':<12}{'n':>4}{'成功':>8}{'成功率':>9}{'静默猜错':>9}{'中位耗时(s)':>12}")
    total_ok = 0
    silent_total = 0
    for cat in CATEGORIES:
        cat_steps = [s for s in all_steps if s["category"] == cat]
        if not cat_steps:
            continue
        ok = sum(1 for s in cat_steps if s["behavior_ok"])
        sw = sum(1 for s in cat_steps if s["silent_wrong"])
        med = statistics.median([s["wall_s"] for s in cat_steps])
        total_ok += ok
        silent_total += sw
        print(f"{cat:<12}{len(cat_steps):>4}{ok:>8}{ok / len(cat_steps):>9.1%}{sw:>9}{med:>12.3f}")
    print()
    print(f"合计 {total_ok}/{len(all_steps)} = {total_ok / max(1, len(all_steps)):.1%}")

    # 失败清单（带原因，便于诊断）
    fails = [s for s in all_steps if not s["behavior_ok"]]
    if fails:
        print()
        print(f"失败 {len(fails)} 条：")
        for s in fails[:15]:
            print(f"  {s['id']} [{s['category']}] {s['utterance']!r} -> "
                  f"{s['actual_behavior']}/{s['actual_reason_code'] or s['actual_target']} "
                  f"(期望 {s['expected_behavior']}/{s['expected_reason_code'] or s['expected_target']}) "
                  f"path={s['node_path']}")

    # LANG-1 的两条硬判据
    no_reason = [s for s in all_steps if s["expected_behavior"] == "reject" and not s["actual_reason_code"]]
    seq_bad = [s for s in all_steps if not s["seq_monotonic"]]
    print()
    print(f"LANG-1a 歧义静默猜错      {silent_total} 次  (判据: 0)")
    print(f"LANG-1b 拒绝缺原因码      {len(no_reason)} 条  (判据: 0)")
    print(f"   动作 seq 单调          {'OK' if not seq_bad else f'{len(seq_bad)} 条异常'}")
    if args.backend == "ollama":
        fb = getattr(nlu, "fallbacks", 0)
        print(f"   LLM 回退规则后端      {fb} 条  (输出没解析成 JSON 就会回退，数字会虚高)")
    if args.backend == "ollama-constrained":
        st = getattr(nlu, "stats", {"fallbacks": 0, "repairs": 0, "dropped_slots": 0})
        print(f"   LLM 回退规则后端      {st['fallbacks']} 条  (两轮都不合格才算回退)")
        print(f"   带错误重问            {st['repairs']} 次")
        print(f"   落地面板丢弃槽位      {st['dropped_slots']} 个  (模型凭空补的巷/层/bay 都在这里被拦)")
        print("   约束层口径            模型**只抽槽位**；结构由正则取；kind 由编译器推导")
    print(f"   输出                  {out_csv.relative_to(ROOT)}")
    print(f"   状态转移 JSONL        {tr_file.relative_to(ROOT)}  ({len(transitions)} 行)")

    ok_lang = silent_total == 0 and not no_reason
    print()
    print("RESULT:", "PASS" if ok_lang else "FAIL")
    return 0 if ok_lang else 1


if __name__ == "__main__":
    raise SystemExit(main())
