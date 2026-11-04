#!/usr/bin/env python3
"""P6-2：一条自然语言指令端到端跑通（真工具层）—— 判据是"开到货位前 → 复核 → 回答"。

用法
  # 离线（不动机器人、不抓帧、不需要 ROS）：只验 resolve/plan/编译链
  .venv/bin/python scripts/p6_demo.py --dry --utterance "去 C-L-21-1"

  # 真跑（需要仿真 + 相机桥已在跑）
  .venv/bin/python scripts/p6_demo.py --utterance "去 C-L-21-1"
  .venv/bin/python scripts/p6_demo.py --utterance "去 10 号位" --answer "C 巷右边第一层"

产物
  results/p6_demo_<时间>.json        单条结果（含判别、耗时、目标、原因码）
  transitions/p6_<时间>.jsonl        状态转移（与 LANG-1 同一格式，可回放）

**口径**：resolve/plan 只读 `world/slot_grid.json` + 语义图；真值只用于运动闭环与
**演示结束后的打分**（`score_target()`，只打印，不回传给图）。
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from depot_vln.agent.graph import build_graph, memory_checkpointer  # noqa: E402
from depot_vln.agent.nlu import LLMNLU, RuleNLU  # noqa: E402
from depot_vln.agent.ros_tools import SimToolLayer  # noqa: E402
from depot_vln.sim import load_sim_world  # noqa: E402


def _rule_id(args) -> str | None:
    """verify 判据的版本号（只有 depth 判据有）；旧产物没有这个字段，统计时按版本分开。"""
    if args.verify != "depth":
        return None
    from depot_vln.verify_depth import RULE_ID

    return RULE_ID


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--utterance", required=True, help="一条自然语言指令")
    ap.add_argument("--answer", default="", help="澄清回答（若指令有歧义，脚本会用它）")
    ap.add_argument("--dry", action="store_true", help="只走 resolve/plan，不动机器人、不抓帧")
    ap.add_argument("--no-verify", action="store_true", help="跳过 VLM 复核（调试用）")
    ap.add_argument("--vlm", default="qwen3-vl:8b", help="verify 用的 VLM 模型")
    ap.add_argument("--verify", default="vlm", choices=["vlm", "depth"],
                    help="vlm=看图判断（S/M 类不可靠）；depth=双目深度门限（推荐）")
    ap.add_argument("--act-backend", default="drive_to", choices=["drive_to", "nav2"],
                    help="act 用哪条控制链：drive_to=图航点+点对点（D23-1）/ "
                         "nav2=单目标交给规划器（能绕动态障碍）")
    ap.add_argument("--pose-source", default="truth", choices=["truth", "odom"],
                    help="verify 投影用哪一路位姿：truth=真值（运动层口径）/"
                         "odom=轮速里程计+静态 map→odom（部署口径）")
    ap.add_argument("--image-topic", default="/stereo_left")
    ap.add_argument("--no-lane-offset", action="store_true",
                    help="不套行驶车道偏移（故意走巷道中线；测前向安全层用）")
    ap.add_argument("--speed", type=float, default=0.35)
    ap.add_argument("--rtf", type=float, default=0.44,
                    help="仿真实时因子（预算按它折算墙钟）。headless+cam-bridge 实测 0.44；"
                         "开 Gazebo GUI 会分走渲染算力，实测降到 0.25~0.35 → 传小一点免得航段超时")
    ap.add_argument("--substitute", action="store_true",
                    help="被挡时**允许真的改道**（默认只提议、不代替用户决定）")
    ap.add_argument("--block", default="",
                    help="注入路障：'AISLE:b0-b1' = 在 bay b0..b1 与各自后一格之间立墙"
                         "（多个用逗号分隔）。例：--block 'C:20-21' 把 C 巷 21 号站台孤立")
    ap.add_argument("--backend", default="rule", choices=["rule", "llm"],
                    help="理解层后端。rule=确定性（默认，离线可用）；"
                         "ollama-constrained=P6-1 约束层（需要 ollama）")
    args = ap.parse_args()

    t_sim0 = time.time()

    # 路障注入：不动 world/、不手改 JSON，改动只在**这一轮的内存里**（可复跑、可对照）
    blocks: list[dict] = []
    if args.block:
        grid = json.loads((ROOT / "world" / "slot_grid.json").read_text(encoding="utf-8"))
        der = grid["derived"]
        x0 = -der["rack_run"] / 2
        pitch = 2.4                                  # 站台间距（semantic_graph 同源常数）
        for spec in args.block.split(","):
            aisle, _, rng = spec.partition(":")
            b0, b1 = (int(v) for v in rng.split("-"))
            # 墙立在第 b 格与第 b+1 格**之间**（中点），图才会切断这条边
            xs = [round(x0 + b * pitch, 3) for b in range(b0, b1 + 1)]
            blocks.append({"aisle": aisle, "barrier_xs": xs, "bay_from": b0, "bay_to": b1,
                           "reason": "p6_demo 注入路障"})
        for b in blocks:
            print(f"[p6] 注入路障：{b['aisle']} 巷 bay {b['bay_from']}→{b['bay_to']+1} 之间，"
                  f"x={b['barrier_xs']}")

    world = load_sim_world(blocked=blocks or None)  # 地图 + 语义图（显式替身：provider=sim-sampled）

    # preflight：真跑前必须确认**真值位姿**这条通道活着。
    # 踩过：`/world/depot/pose/info` 一次读 573 个模型，仿真刚起来时首读会撞 8 s 超时
    # （`slam_route._run` 吞 stderr 返回空串）→ 到了 act 才报 MOTION_NO_POSE。
    poller = None
    if not args.dry:
        from slam_route import TruthPoller

        poller = TruthPoller()
        t0p = time.time()
        pose0 = None
        while time.time() - t0p < 60.0:
            pose0 = poller.get(5.0)
            if pose0 is not None:
                break
            time.sleep(1.0)
        if pose0 is None:
            print("[p6] 真值位姿 60 s 没来：/world/depot/pose/info 这条通道不通。")
            import subprocess
            try:
                lst = subprocess.run(["ign", "topic", "-l"], capture_output=True,
                                     text=True, timeout=20).stdout
                print("[p6] pose 相关话题：",
                      [t for t in lst.split() if "pose" in t][:6])
            except Exception as e:
                print(f"[p6] ign 也调不动：{e}")
            return 2
        print(f"[p6] 真值就绪 x={pose0[0]:+.2f} y={pose0[1]:+.2f} "
              f"yaw={pose0[2]:+.3f}（等了 {time.time() - t0p:.0f}s）")

    tools = SimToolLayer(world, dry=args.dry, vlm_model=args.vlm,
                         image_topic=args.image_topic, speed=args.speed,
                         poller=poller,
                         lane_dy={} if args.no_lane_offset else None,
                         verify=args.verify, pose_source=args.pose_source,
                         act_backend=args.act_backend, rtf=args.rtf)
    # 图的 verify 节点通过 tools.observe 取观测；--no-verify 时换成一个"未复核"的桩
    if args.no_verify or args.dry:
        class _NoVerify(SimToolLayer):
            """不抓帧的桩：dry/--no-verify 时用，让图用完通路但不假装看过现场。"""

            def observe(self, target):
                from depot_vln.agent.tools import Observation
                if self.dry:
                    return Observation(ok=True, reason_code=None, occupied=None, cls=None,
                                       confidence=0.0,
                                       detail="[dry] 未抓帧（只验通路，不验内容）")
                return Observation(ok=False, reason_code="VERIFY_SKIPPED",
                                   detail="--no-verify：跳过现场复核")
        tools = _NoVerify(world, dry=args.dry, image_topic=args.image_topic,
                          speed=args.speed, poller=poller,
                          lane_dy={} if args.no_lane_offset else None,
                          verify=args.verify, pose_source=args.pose_source,
                          act_backend=args.act_backend, rtf=args.rtf)

    nlu = RuleNLU() if args.backend == "rule" else LLMNLU()
    graph = build_graph(tools, nlu, world=world, checkpointer=memory_checkpointer())
    thread_id = f"p6-{datetime.now().strftime('%H%M%S')}"
    cfg = {"configurable": {"thread_id": thread_id}, "recursion_limit": 30}
    init = {"thread_id": thread_id, "utterance": args.utterance, "history": [],
            "clarify_count": 0, "recover_count": 0, "alt_index": 0, "seq": 0,
            "allow_substitute": bool(args.substitute),
            "trajectory": [], "node_trace": [], "notes": []}

    print(f"== P6-2 demo  dry={args.dry}  utterance={args.utterance!r} ==")
    t0 = time.perf_counter()
    from langgraph.types import Command

    state = graph.invoke(init, cfg)
    questions: list[str] = []
    rounds = 0
    while "__interrupt__" in state and args.answer and rounds < 2:
        intr = state["__interrupt__"]
        try:
            payload = intr[0].value if isinstance(intr, (list, tuple)) else intr.value
            q = payload.get("question") if isinstance(payload, dict) else str(intr)
        except Exception:
            q = str(intr)
        questions.append(str(q))
        print(f"[clarify] {q}")
        rounds += 1
        state = graph.invoke(Command(resume=args.answer), cfg)
    wall = time.perf_counter() - t0

    if "__interrupt__" in state:
        print("[clarify] 图仍在等待回答；用 --answer \"…\" 再跑一次即可继续这条指令")

    target = state.get("target") or {}
    rc = state.get("reason_code")
    obs = state.get("observation") or {}
    plan = state.get("plan") or {}
    trace = state.get("node_trace") or []
    print()
    print(f"节点轨迹   {'>'.join(trace)}")
    print(f"目标       {target.get('address') or target.get('aisle') or target.get('kind')}")
    print(f"计划       {plan.get('path_length', '?')} m / ETA {plan.get('eta', '?')} s "
          f"/ 进入动作 {(plan.get('entry') or {}).get('type')}")
    print(f"观测       ok={obs.get('ok')} occupied={obs.get('occupied')} "
          f"cls={obs.get('cls')} conf={obs.get('confidence')} "
          f"({obs.get('reason_code') or ''})")
    if obs.get("detail"):
        print(f"观测详情   {obs['detail']}")
    act_step = next((e for e in (state.get("trajectory") or [])
                     if e.get("tool") == "navigate"), {}) or {}
    if act_step.get("pose"):
        pz = act_step["pose"]
        err = act_step.get("endpoint_err_m")
        print(f"到达位姿   ({pz['x']:+.2f}, {pz['y']:+.2f}) yaw {pz['yaw']:+.3f}"
              + (f"  末段误差 {err:.2f} m" if isinstance(err, (int, float)) else ""))
    print(f"原因码     {rc or 'OK'}")
    print(f"回答       {state.get('answer')}")
    print(f"墙钟       {wall:.1f} s")

    # 打分（只打印，不回传给图）
    score = None
    if target:
        from depot_vln.scoring import score_observation

        score = score_observation(target, obs or None)
        print(f"打分(真值) {json.dumps(score, ensure_ascii=False)[:400]}")
        if score.get("occupied_match") is False:
            print("⚠️ 占用判定与真值不符（verify 判错了）")
        elif score.get("class_match") is False:
            print(f"⚠️ 类别与真值不符：观测 {score.get('obs_class')} vs 真值 "
                  f"{score.get('truth_class')}（320×240 下 S/M 仅差 ~9 px，见 D25）")

    stamp = datetime.now().strftime("%Y%m%dT%H%M%S")
    out = ROOT / "results" / f"p6_demo_{stamp}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({
        "utterance": args.utterance, "dry": args.dry, "answer_used": args.answer,
        "clarify_questions": questions, "node_trace": trace,
        "target": target, "plan": plan, "observation": obs,
        "verify": args.verify, "pose_source": args.pose_source, "verify_rule": _rule_id(args),
        # 实验口径：>0/<=0 表示**故意**注入的相机俯仰误差（0 = 正常工况）
        "cam_pitch_deg": float(os.environ.get("CAM_PITCH_DEG", "0") or 0),
        "pose_final": act_step.get("pose"), "endpoint_err_m": act_step.get("endpoint_err_m"),
        "substituted_from": state.get("substituted_from"), "suggest": state.get("suggest"),
        "reason_code": rc, "answer": state.get("answer"),
        "trajectory": state.get("trajectory"), "notes": state.get("notes"),
        "score": score, "wall_s": round(wall, 2), "elapsed_sim_s": round(time.time() - t_sim0, 1),
    }, ensure_ascii=False, indent=2), encoding="utf-8")

    tr = ROOT / "transitions" / f"p6_{stamp}.jsonl"
    tr.parent.mkdir(exist_ok=True)
    with open(tr, "w", encoding="utf-8") as fh:
        for i, node in enumerate(trace):
            fh.write(json.dumps({"run": thread_id, "i": i, "node": node,
                                 "utterance": args.utterance,
                                 "reason_code": rc or "", "target": target.get("address") or "",
                                 "wall_s": round(wall, 2)}, ensure_ascii=False) + "\n")

    # 判据：真跑 = 到达且复核过，**或者**给出合法的拒绝原因码。开不过去（TIMEOUT）算失败。
    # 三态：0=PASS；3=PASS_WITH_WARN（到达且占用对，但类别与真值不符）；1=FAIL。
    REJECT_OK = {"BLOCKED", "NO_SLOT", "EMPTY_BAY"}
    if args.dry:
        ok = bool(state.get("answer")) and (bool(rc) or (bool(target) and bool(plan)))
        code = 0 if ok else 1
    else:
        ok = bool(state.get("answer")) and (bool(obs.get("ok")) or rc in REJECT_OK)
        code = 0 if ok else 1
        if ok and isinstance(score, dict):
            # 3 = PASS_WITH_WARN：执行/回答都对，但**观测与真值不符**（占用或类别）。
            # 这一类必须单独标出来：占用判错意味着 verify 的输出不可信，
            # 不能混进 PASS（2026-11-02 曾把"空位报成有货"记成 PASS）。
            if score.get("occupied_match") is False:
                code = 3
            elif score.get("class_match") is False:
                code = 3
    print()
    print(f"产物   {out.relative_to(ROOT)}   {tr.relative_to(ROOT)}")
    verdict = ("PASS" if code == 0
               else ("PASS_WITH_WARN（观测与真值不符）" if code == 3 else "FAIL"))
    print("RESULT:", verdict,
          "" if code in (0, 3) else "（未到达且无原因码，或回答为空）")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
