"""图节点实现（SPEC §8.2 / §8.3）。

模型参与的只有三类节点：understand / clarify / verify / respond（前两者文本，verify 图像）。
其余（resolve / plan / act / recover）全是代码 —— 这条边界在本文件里就是"谁调了 nlu"。
"""
from __future__ import annotations

import math
from typing import Any, Callable

from langgraph.types import interrupt

from depot_vln.agent.nlu.compile import matches_constraints
from depot_vln.catalog import get_catalog

from .state import AgentState

MAX_RECOVER = 2
MAX_CLARIFY = 2


def _t(name: str) -> list[str]:
    return [name]


class Runtime:
    """节点共享的依赖：工具层 + NLU + 世界（用于地图写回）。"""

    def __init__(self, tools, nlu, world=None, context: dict[str, Any] | None = None):
        self.tools = tools
        self.nlu = nlu
        self.world = world or getattr(tools, "world", None)
        self.context = context or {}

    def nlu_context(self) -> dict[str, Any]:
        """给理解层的世界上下文。**全部来自 catalog**，不再有 list("ABCDEF")/24。"""
        ctx = get_catalog().context()
        ctx.update(self.context)
        return ctx


# ---------------------------------------------------------------- understand
def make_understand(rt: Runtime) -> Callable[[AgentState], dict]:
    def understand(state: AgentState) -> dict:
        out = rt.nlu.understand(state.get("utterance", ""), rt.nlu_context())
        return {
            "intent": out.get("intent", "unknown"),
            "query": out.get("query", {"kind": "unknown"}),
            "slots": out.get("slots") or {},
            "backend": out.get("backend", getattr(rt.nlu, "name", "?")),
            "dropped": list(out.get("dropped") or []),
            "node_trace": _t("understand"),
        }

    return understand


def _endpoint_err(pose: dict | None, obs_pose: dict | None) -> float | None:
    """末段停位误差：到达位姿与观测位的平面距离（没有任一侧数据就返回 None，不编）。"""
    if not pose or not obs_pose:
        return None
    try:
        return round(math.hypot(float(pose["x"]) - float(obs_pose["x"]),
                                float(pose["y"]) - float(obs_pose["y"])), 3)
    except (KeyError, TypeError, ValueError):
        return None


# ---------------------------------------------------------------- resolve
def make_resolve(rt: Runtime) -> Callable[[AgentState], dict]:
    def resolve(state: AgentState) -> dict:
        q = dict(state.get("query") or {})
        constraints = dict(state.get("constraints") or {})
        kind = q.get("kind")

        if kind == "unknown" or not kind:
            return {"reason_code": "UNKNOWN_INTENT", "node_trace": _t("resolve")}

        # "封住的那条巷子"：不是去不了，是被明确封堵 —— 给 BLOCKED 而不是绕路
        if kind == "blocked_aisle":
            aisle = q.get("aisle")
            blocked_aisles = {seg["aisle"] for seg in getattr(rt.world, "blocked", []) or []}
            if aisle and aisle in blocked_aisles:
                return {
                    "reason_code": "BLOCKED",
                    "question": f"{aisle} 巷被路障封堵（路障在 bay {min(s['bay_from'] for s in rt.world.blocked if s['aisle'] == aisle)} 之前）",
                    "node_trace": _t("resolve"),
                }
            if aisle:
                q = {"kind": "aisle", "aisle": aisle}
            else:
                blocked = sorted({s["aisle"] for s in (getattr(rt.world, "blocked", None) or [])})
                if blocked:
                    return {
                        "reason_code": "BLOCKED",
                        "question": f"被封的是 {'、'.join(blocked)} 巷",
                        "node_trace": _t("resolve"),
                    }
                return {"reason_code": "NO_SLOT", "node_trace": _t("resolve")}

        # 闭集之外的类别（"冰箱"）：理解层认出来了，但地图里没有这个概念
        classes = set(q.get("classes") or [])
        if "__unknown__" in classes:
            return {"reason_code": "NO_SLOT", "node_trace": _t("resolve")}

        cands, reason = rt.tools.resolve(q)
        if cands and constraints:
            cands = [c for c in cands if matches_constraints(c["target"], constraints)]
            reason = None if cands else "NO_SLOT"
        for key in ("side", "level"):
            v = q.get(key)
            if v and cands:
                filtered = [c for c in cands if c["target"].get(key) in (None, v)]
                if filtered:
                    cands = filtered
        if not cands:
            return {"candidates": [], "reason_code": reason or "NO_SLOT", "node_trace": _t("resolve")}

        # 可达性：路径被挡 = BLOCKED（不是"没货位"）
        reachable = [c for c in cands if c.get("reachable", True)]
        if not reachable:
            return {
                "candidates": cands,
                "reason_code": "BLOCKED",
                "question": "目标在路障后面，图上没有通路",
                "node_trace": _t("resolve"),
            }
        cands = reachable

        if len(cands) > 1:
            return {"candidates": cands, "reason_code": "AMBIGUOUS", "node_trace": _t("resolve")}

        return {
            "candidates": cands,
            "target": cands[0]["target"],
            "reason_code": None,
            "node_trace": _t("resolve"),
        }

    return resolve


# ---------------------------------------------------------------- clarify
def make_clarify(rt: Runtime) -> Callable[[AgentState], dict]:
    def clarify(state: AgentState) -> dict:
        n = state.get("clarify_count", 0)
        if n >= MAX_CLARIFY:
            return {"reason_code": "CLARIFY_LIMIT", "node_trace": _t("clarify")}
        question = rt.nlu.ask(state.get("candidates") or [], state.get("utterance", ""))
        # interrupt 只用于"等一句话"（SPEC §8.3 ②）：机器人动起来期间绝不挂起图
        answer = interrupt({"question": question, "candidates": [c["target"] for c in (state.get("candidates") or [])]})
        from depot_vln.agent.nlu.compile import parse_clarification

        c = parse_clarification(str(answer))
        merged = dict(state.get("constraints") or {})
        merged.update(c)
        notes = [f"clarify#{n + 1}: 「{answer}」 -> {c}"]
        return {
            "clarify_count": n + 1,
            "constraints": merged,
            "clarify_answer": str(answer),
            "question": question,
            "notes": notes,
            "node_trace": _t("clarify"),
        }

    return clarify


# ---------------------------------------------------------------- plan
def make_plan(rt: Runtime) -> Callable[[AgentState], dict]:
    def plan(state: AgentState) -> dict:
        target = state.get("target") or {}
        p = rt.tools.plan(target)
        if p is None:
            return {"plan": None, "reason_code": "UNREACHABLE", "node_trace": _t("plan")}
        return {"plan": p, "target": p.get("goal", target), "reason_code": None, "node_trace": _t("plan")}

    return plan


# ---------------------------------------------------------------- act
def make_act(rt: Runtime) -> Callable[[AgentState], dict]:
    def act(state: AgentState) -> dict:
        seq = int(state.get("seq", 0)) + 1
        res = rt.tools.navigate(state.get("target") or {}, state.get("plan"), seq)
        step = {
            "seq": seq,
            "tool": "navigate",
            "args": {"target": state.get("target"), "entry": (state.get("plan") or {}).get("entry")},
            "ok": res.ok,
            "reason_code": res.reason_code,
            "detail": res.detail,
            "elapsed": res.elapsed,
            # 到达位姿（真值，运动层闭环用）+ 末段误差，回放时要能看到"停在哪"
            "pose": res.pose,
            "endpoint_err_m": _endpoint_err(res.pose, (state.get("plan") or {}).get("obs_pose")),
        }
        traj = list(state.get("trajectory") or []) + [step]
        return {
            "seq": seq,
            "trajectory": traj,
            "pose": res.pose or state.get("pose"),
            "reason_code": None if res.ok else (res.reason_code or "TIMEOUT"),
            "node_trace": _t("act"),
        }

    return act


# ---------------------------------------------------------------- verify
def make_verify(rt: Runtime) -> Callable[[AgentState], dict]:
    def verify(state: AgentState) -> dict:
        target = state.get("target") or {}
        obs = rt.tools.observe(target)
        reason = None
        if not obs.ok:
            reason = obs.reason_code or "VERIFY_MISMATCH"
        # 请求里带了类别（"纸箱"）：现场类别必须落在请求集合里
        want = set((state.get("query") or {}).get("classes") or []) - {"__unknown__"}
        if reason is None and want and target.get("kind") == "slot" and obs.cls and obs.cls not in want:
            reason = "VERIFY_MISMATCH"
        if reason is None and target.get("kind") == "slot":
            cand = (state.get("candidates") or [{}])[0]
            if cand.get("observed_occupied") is not None and obs.occupied is not None:
                if cand["observed_occupied"] != obs.occupied:
                    reason = "VERIFY_MISMATCH"
        # 写回地图：这就是"再次观测刷新置信度"的那一步
        if rt.world is not None and target.get("kind") == "slot" and obs.ok:
            try:
                rt.world.observe(target["address"], obs.occupied, obs.cls,
                                 float(obs.confidence), source="vision")
            except Exception:
                pass
        step = {
            "seq": None,
            "tool": "observe",
            "args": {"target": target},
            "ok": obs.ok and reason is None,
            "reason_code": reason,
            "detail": obs.detail,
            "elapsed": obs.elapsed,
        }
        traj = list(state.get("trajectory") or []) + [step]
        return {
            "observation": obs.model_dump(),
            "trajectory": traj,
            "reason_code": reason,
            "node_trace": _t("verify"),
        }

    return verify


# ---------------------------------------------------------------- recover
def make_recover(rt: Runtime) -> Callable[[AgentState], dict]:
    def recover(state: AgentState) -> dict:
        n = int(state.get("recover_count", 0))
        rc = state.get("reason_code")
        notes = [f"recover#{n + 1} from {rc}"]
        if n >= MAX_RECOVER:
            return {"reason_code": "RECOVER_LIMIT", "notes": notes, "node_trace": _t("recover")}

        alt = int(state.get("alt_index", 0))
        tried = list(state.get("tried") or [])

        # 1) 还有别的候选：换一个（语义级恢复，而不是原地重试）
        cands = list(state.get("candidates") or [])
        if len(cands) > alt + 1 and rc in ("VERIFY_MISMATCH", "TIMEOUT", "AMBIGUOUS"):
            nxt = cands[alt + 1]
            addr = nxt["target"].get("address")
            return {
                "recover_count": n + 1,
                "alt_index": alt + 1,
                "tried": tried + ([addr] if addr else []),
                "target": nxt["target"],
                "reason_code": None,
                "plan": None,
                "notes": notes + [f"换候选 {addr}"],
                "node_trace": _t("recover"),
            }

        # 2) 被挡/不可达：先试**语义换位**（同巷同面同层最近的 bay），换不了才把原因码交回。
        #    这是 P6 里"模型/规则都能超过纯 if-else"的地方：不是原地重试，是拿地图+语义图
        #    找一个语义等价、且图上可达的替代目标，并在回答里**明说改道**。
        if rc in ("BLOCKED", "UNREACHABLE"):
            tgt = dict(state.get("target") or {})
            if not tgt:            # resolve 判 BLOCKED 时只给 candidates、不给 target
                for c in (state.get("candidates") or []):
                    if (c or {}).get("target"):
                        tgt = dict(c["target"])
                        break
            cur = tgt.get("address")
            if cur:
                tried = tried + [cur]
            sub = None
            fn = getattr(rt.world, "substitute", None)
            if fn is not None and n < MAX_RECOVER:
                try:
                    sub = fn(tgt, exclude=tried)
                except TypeError:      # 旧签名兜底
                    sub = fn(tgt)
            # **默认不改道**：用户点名了地址，被挡就该老实说"去不了"并**提议**替代，
            # 由用户决定。只有 allow_substitute（演示/显式授权）才真的开过去。
            # 理由：不请自来的"换位"与静默猜是同一类错误 —— 用户要的是 21 号位，
            # 你开到 20 号位还说"到了"，比报 BLOCKED 更糟。
            if sub is not None and sub.get("target") and state.get("allow_substitute"):
                naddr = sub["target"].get("address")
                return {
                    "recover_count": n + 1,
                    "alt_index": alt + 1,
                    "tried": tried + ([naddr] if naddr else []),
                    "target": sub["target"],
                    "substituted_from": cur,
                    "reason_code": None,
                    "plan": None,
                    "notes": notes + [f"换位 {cur} → {naddr}（{sub.get('why', '')}）"],
                    "node_trace": _t("recover"),
                }
            # 不改道：把建议带上（回答里问用户），原因码原样交回
            sug = (sub or {}).get("target", {}).get("address") if sub else None
            return {"recover_count": n + 1, "reason_code": rc, "suggest": sug,
                    "notes": notes + ([f"建议改去 {sug}"] if sug else []),
                    "node_trace": _t("recover")}

        return {"recover_count": n + 1, "reason_code": rc or "RECOVER_LIMIT", "notes": notes, "node_trace": _t("recover")}

    return recover


# ---------------------------------------------------------------- respond
def make_respond(rt: Runtime) -> Callable[[AgentState], dict]:
    def respond(state: AgentState) -> dict:
        rc = state.get("reason_code")
        s = dict(state)
        s["reason_code"] = "OK" if rc is None and state.get("observation") else rc
        answer = rt.nlu.respond(s)
        return {"answer": answer, "node_trace": _t("respond")}

    return respond


# ---------------------------------------------------------------- 路由
def route_after_resolve(state: AgentState) -> str:
    rc = state.get("reason_code")
    if rc is None:
        return "plan"
    if rc == "AMBIGUOUS":
        return "clarify"
    if rc == "BLOCKED":
        return "recover"
    return "respond"


def route_after_clarify(state: AgentState) -> str:
    return "respond" if state.get("reason_code") == "CLARIFY_LIMIT" else "resolve"


def route_after_plan(state: AgentState) -> str:
    return "respond" if state.get("plan") is None else "act"


def route_after_act(state: AgentState) -> str:
    return "verify" if state.get("reason_code") is None else "recover"


def route_after_verify(state: AgentState) -> str:
    return "respond" if state.get("reason_code") is None else "recover"


def route_after_recover(state: AgentState) -> str:
    rc = state.get("reason_code")
    if rc is None and state.get("target"):
        return "plan"
    return "respond"
