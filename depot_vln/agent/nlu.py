"""understand / clarify / respond 三个"需要模型"的节点（SPEC §8.2）。

两种后端：
  RuleNLU    —— 确定性规则，离线、零延迟。基准默认用它，这样"语言层"的分数不会被
                7B 模型的采样噪声污染；换模型时对比才有意义。
  OllamaNLU  —— qwen2.5:7b 抽槽位（文本），qwen3-vl:8b 做 verify（图像）。

两者输出同一份 schema，图不关心是哪个后端。
"""
from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass
from typing import Any, Protocol

CARTON_CLASSES = {"carton_s", "carton_m", "carton_l"}

CLASS_WORDS: list[tuple[str, set[str]]] = [
    ("纸箱", CARTON_CLASSES),
    ("纸板箱", CARTON_CLASSES),
    ("carton", CARTON_CLASSES),
    ("箱子", CARTON_CLASSES),
    ("托盘", {"pallet"}),
    ("pallet", {"pallet"}),
    ("周转箱", {"tote"}),
    ("蓝色箱", {"tote"}),
    ("tote", {"tote"}),
    ("油桶", {"drum"}),
    ("桶", {"drum"}),
    ("drum", {"drum"}),
]

# 闭集之外的词：理解层要认出来"这是我没见过的类别"，而不是猜最近的
UNKNOWN_CLASS_WORDS = ["冰箱", "冰柜", "电视", "沙发", "fridge"]

_SIDE_WORD = {"左": "L", "左边": "L", "左侧": "L", "left": "L", "右": "R", "右边": "R", "右侧": "R", "right": "R"}


def parse_clarification(text: str) -> dict[str, Any]:
    """把用户对澄清问题的回答变成约束条件。"""
    t = (text or "").strip().lower()
    c: dict[str, Any] = {}
    m = re.search(r"\b([a-z])\b\s*(?:巷|通道|aisle|row)?", t)
    if m:
        c["aisle"] = m.group(1).upper()
    for w, s in _SIDE_WORD.items():
        if w in t:
            c["side"] = s
            break
    # 注意：`第?二?层` 这种写法会把"第一层"里的"层"也匹配成第 2 层（踩过）
    if re.search(r"(第二层|二层|上层|顶层|second|level\s*2|第?2\s*层)", t):
        c["level"] = 2
    elif re.search(r"(第一层|一层|底层|下层|地面|ground|level\s*1|第?1\s*层)", t):
        c["level"] = 1
    m = re.search(r"(\d{1,2})\s*(?:号|号位|bay)?", t)
    if m and "aisle" in c:
        bay = int(m.group(1))
        if 1 <= bay <= 99:
            c["bay"] = bay
    m = re.search(r"([a-z])\s*[-_]?\s*(l|r|左|右)\s*[-_]?\s*(\d{1,2})", t)
    if m:
        c["aisle"] = m.group(1).upper()
        c["side"] = "L" if m.group(2) in ("l", "左") else "R"
        c["bay"] = int(m.group(3))
    return c


def matches_constraints(target: dict[str, Any], c: dict[str, Any]) -> bool:
    if not c:
        return True
    if "aisle" in c and target.get("aisle") != c["aisle"]:
        return False
    if "side" in c and target.get("side") not in (None, c["side"]):
        return False
    if "level" in c and target.get("level") not in (None, c["level"]):
        return False
    if "bay" in c and target.get("bay") not in (None, c["bay"]):
        return False
    return True


class NLU(Protocol):
    name: str

    def understand(self, utterance: str, context: dict[str, Any]) -> dict[str, Any]: ...

    def ask(self, candidates: list[dict[str, Any]], utterance: str) -> str: ...

    def respond(self, state: dict[str, Any]) -> str: ...


# ---------------------------------------------------------------- 规则后端
@dataclass
class RuleNLU:
    name: str = "rule"

    def understand(self, utterance: str, context: dict[str, Any]) -> dict[str, Any]:
        u = (utterance or "").strip()
        low = u.lower()
        aisles = context.get("aisles") or list("ABCDEF")
        n_bays = int(context.get("bays_per_aisle") or 24)

        # 0) 明确提到"封住/封堵"的巷子
        if re.search(r"(封住|封堵|封了|封锁|blocked|sealed)", low):
            m = re.search(r"\b([a-z])\b", low)
            return {
                "intent": "goto",
                "query": {"kind": "blocked_aisle", "aisle": m.group(1).upper() if m else None},
            }

        # 1) 完整地址 A-L-07-2
        m = re.search(r"\b([a-z])\s*[-_\s]\s*(l|r|左|右|left|right)\s*[-_\s]\s*(\d{1,2})(?:\s*[-_\s]\s*(\d{1,2}))?", low)
        if m:
            aisle = m.group(1).upper()
            side = {"l": "L", "左": "L", "left": "L", "r": "R", "右": "R", "right": "R"}[m.group(2)]
            bay = int(m.group(3))
            level = int(m.group(4)) if m.group(4) else 1
            if aisle not in aisles or not (1 <= bay <= n_bays) or level not in (1, 2):
                return {"intent": "goto", "query": {"kind": "address", "address": f"{aisle}-{side}-{bay:02d}-{level}"},
                        "candidates_hint": "invalid"}
            return {
                "intent": "goto",
                "query": {"kind": "address", "address": f"{aisle}-{side}-{bay:02d}-{level}"},
            }

        aisle_m = re.search(r"\b([a-z])\b\s*(?:巷|通道|aisle|row)", low)
        classes = self._classes(low)
        side = next((s for w, s in _SIDE_WORD.items() if w in low), None)
        empty = bool(re.search(r"(空位|空格|空的|没有货|empty)", low))
        farthest = bool(re.search(r"(最里面|最里头|最远|最后一个|尽头|farthest|deepest|last)", low))
        most = bool(re.search(r"(最多|most)", low))
        lvl = self._level(low)

        # 闭集之外的类别（"冰箱"）：先认出来，别落到"只说巷"的分支去
        if self._unknown_class(low) and not classes:
            return {"intent": "goto", "query": {"kind": "max_class", "classes": ["__unknown__"]}}

        bay_m = re.search(r"(\d{1,2})\s*号", low)
        # 2a) 精确定位到 bay + 类别（"A 巷 5 号位放油桶的那格"）
        if aisle_m and bay_m and classes:
            return {
                "intent": "goto",
                "query": {
                    "kind": "class_in_bay",
                    "aisle": aisle_m.group(1).upper(),
                    "bay": int(bay_m.group(1)),
                    "classes": sorted(classes),
                    "side": side,
                    "level": lvl,
                },
            }
        # 2) 属性/关系：某巷里放某类货的那面货架
        if aisle_m and classes and not farthest:
            return {
                "intent": "goto",
                "query": {
                    "kind": "class_in_aisle",
                    "aisle": aisle_m.group(1).upper(),
                    "classes": sorted(classes),
                    "side": side,
                    "level": lvl,
                },
            }
        # 3) 全仓"纸箱最多的那排货架"
        if classes and most and not aisle_m:
            return {"intent": "goto", "query": {"kind": "max_class", "classes": sorted(classes)}}
        # 4) 某巷最里面的位
        if aisle_m and farthest:
            return {
                "intent": "goto",
                "query": {"kind": "farthest", "aisle": aisle_m.group(1).upper(), "empty": empty,
                          "side": side, "level": lvl},
            }
        # 5) 某巷空位
        if aisle_m and empty:
            m2 = re.search(r"(\d{1,2})\s*号", low)
            if m2:
                return {
                    "intent": "goto",
                    "query": {"kind": "empty_slot_in_bay", "aisle": aisle_m.group(1).upper(),
                              "bay": int(m2.group(1)), "level": lvl},
                }
            return {"intent": "goto", "query": {"kind": "farthest", "aisle": aisle_m.group(1).upper(),
                                                "empty": True, "side": side, "level": lvl}}
        # 6) 只说巷
        if aisle_m:
            return {"intent": "goto", "query": {"kind": "aisle", "aisle": aisle_m.group(1).upper(), "level": lvl}}
        # 7) 只说 N 号位（歧义来源）
        m = re.search(r"(\d{1,2})\s*号", low) or re.search(r"\bbay\s*(\d{1,2})\b", low)
        if m:
            bay = int(m.group(1))
            return {"intent": "goto", "query": {"kind": "bay", "bay": bay, "level": lvl, "side": side}}
        if self._unknown_class(low):
            return {"intent": "goto", "query": {"kind": "max_class", "classes": ["__unknown__"]}}
        return {"intent": "unknown", "query": {"kind": "unknown"}}

    def _classes(self, low: str) -> set[str]:
        out: set[str] = set()
        for word, classes in CLASS_WORDS:
            if word in low:
                out |= classes
        return out

    @staticmethod
    def _level(low: str) -> int | None:
        if re.search(r"(第二层|上层|二层|top|level\s*2)", low):
            return 2
        if re.search(r"(第一层|底层|下层|地面层|ground|level\s*1)", low):
            return 1
        return None

    def _unknown_class(self, low: str) -> bool:
        return any(w in low for w in UNKNOWN_CLASS_WORDS)

    def ask(self, candidates: list[dict[str, Any]], utterance: str) -> str:
        names = []
        for c in candidates[:6]:
            t = c["target"]
            if t["kind"] == "slot":
                names.append(t["address"])
            elif t["kind"] == "rack_face":
                names.append(f"{t['aisle']}-{t['side']} 面")
            else:
                names.append(str(t.get("aisle")))
        listed = "、".join(names)
        return f"「{utterance}」有几个可能的货位：{listed}。你指的是哪一个？（可以直接说 A 巷左边 7 号位）"

    def respond(self, state: dict[str, Any]) -> str:
        rc = state.get("reason_code")
        target = state.get("target") or {}
        plan = state.get("plan") or {}
        obs = state.get("observation") or {}
        if rc == "OK":
            if target.get("kind") == "slot":
                tail = ""
                if obs.get("ok"):
                    # occupied 未知（dry 或 VLM 只给低置信）时**不许**编"空位"
                    if obs.get("occupied") is None:
                        tail = "现场核验：未取到占用判定"
                    else:
                        tail = f"现场核验：{'有货' if obs.get('occupied') else '空位'}"
                    cls = obs.get("class") or obs.get("cls")   # Observation 字段名是 cls
                    if cls:
                        tail += f"，类别 {cls}"
                # 时间要分清：**行驶用时**取 act 的 elapsed，**核验用时**是 VLM 的耗时。
                # 踩过：原来把 `obs.elapsed`（VLM 9.6 s）写成"走 … 9.6 s"，与墙钟 114 s 矛盾。
                move_s = None
                for e in (state.get("trajectory") or []):
                    if e.get("tool") == "navigate" and isinstance(e.get("elapsed"), (int, float)):
                        move_s = float(e["elapsed"])
                if not move_s:      # dry/未开：退回计划 ETA（不许把 0.0 当"行驶 0 秒"）
                    move_s = float(plan.get("eta", 0) or 0)
                vt = obs.get("elapsed")
                vit = f"，核验 {float(vt):.1f} s" if isinstance(vt, (int, float)) else ""
                txt = (f"已到达 {target.get('address')}（走 {plan.get('path_length', 0):.1f} m，"
                       f"行驶 {move_s:.1f} s{vit}）。{tail}")
                sf = state.get("substituted_from")
                # 换位必须**明说**：不许把"改去的位"说成"你要的位"（那才是静默猜）
                return f"【改道】原目标 {sf} 走不通，" + txt if sf else txt
            if target.get("kind") == "rack_face":
                return f"已到达 {target.get('aisle')}-{target.get('side')} 面货架（走 {plan.get('path_length', 0):.1f} m）。"
            return f"已到达 {target.get('aisle')} 巷。"
        if rc == "AMBIGUOUS":
            return "我不确定你指的是哪一个，" + (state.get("question") or "能说得更具体些吗？")
        if rc == "CLARIFY_LIMIT":
            return "已经问了两轮还是没法确定，请直接给完整地址（例如 B-L-07-2）。"
        if rc == "BLOCKED":
            why = state.get("question") or "该巷道被路障封堵"
            sug = state.get("suggest")
            tail = f"最近的同类可达位是 {sug}，要我改去那里吗？" if sug else ""
            return f"去不了：{why}（原因码 BLOCKED）。{tail}"
        if rc == "NO_SLOT":
            return "没有这个货位（原因码 NO_SLOT）。请确认巷道/bay/层号。"
        if rc == "EMPTY_BAY":
            return "那个 bay 是空的，没有你说的货（原因码 EMPTY_BAY）。"
        if rc == "VERIFY_MISMATCH":
            return "到了位置，但现场看到的和地图记录不一致（原因码 VERIFY_MISMATCH），已标记为待复核。"
        if rc == "UNREACHABLE":
            return "图上找不到能到那里的路径（原因码 UNREACHABLE）。"
        if rc == "RECOVER_LIMIT":
            return "试了两次恢复都失败，交还给你处理（原因码 RECOVER_LIMIT）。"
        if rc == "UNKNOWN_INTENT":
            return "没听懂，能换种说法吗？比如「去 B-L-07-2」。"
        return f"失败：{rc}"


# ---------------------------------------------------------------- Ollama 后端
@dataclass
class OllamaNLU:
    """qwen2.5:7b 抽槽位；给出严格 JSON schema，解析失败就退回规则后端。"""

    model: str = "qwen2.5:7b"
    host: str = "http://127.0.0.1:11434"
    name: str = "ollama"
    _fallback: RuleNLU = None  # type: ignore[assignment]
    fallbacks: int = 0  # 输出没解析成 JSON、退回规则后端的次数（静默回退会伪造好成绩）

    def __post_init__(self) -> None:
        self._fallback = RuleNLU()

    def _chat(self, system: str, user: str) -> str:
        from langchain_ollama import ChatOllama

        # Qwen3 系列默认带"思考"，一句指令要 29 s（qwen2.5:7b 是 0.6 s）——机器人等不起。
        # 默认给它关掉；要对照"思考有没有用"就用 LLM_REASONING=high 打开。
        # qwen2.5 不带思考，保持原样（基线可复现）。
        mode = os.environ.get("LLM_REASONING", "auto")
        kw: dict[str, Any] = {}
        if mode == "off" or (mode == "auto" and self.model.startswith("qwen3")):
            kw["reasoning"] = False
        elif mode not in ("auto", "", "none"):
            kw["reasoning"] = mode
        llm = ChatOllama(model=self.model, base_url=self.host, temperature=0, **kw)
        msg = llm.invoke([("system", system), ("human", user)])
        return str(getattr(msg, "content", msg))

    def understand(self, utterance: str, context: dict[str, Any]) -> dict[str, Any]:
        system = (
            "你是仓库指令解析器。只输出 JSON，不要解释。可用的 query.kind："
            "address(address), bay(bay), aisle(aisle), class_in_aisle(aisle,classes), "
            "max_class(classes), farthest(aisle,empty), empty_slot_in_bay(aisle,bay)。"
            f"巷道 A-{context.get('aisles_last', 'F')}，bay 1-{context.get('bays_per_aisle', 24)}，"
            "层 1-2。类别闭集：carton_s,carton_m,carton_l,tote,pallet,drum。"
            '示例输出：{"intent":"goto","query":{"kind":"address","address":"B-L-07-2"}}'
        )
        try:
            raw = self._chat(system, utterance)
            m = re.search(r"\{.*\}", raw, re.S)
            obj = json.loads(m.group(0)) if m else {}
            if "query" in obj and "kind" in obj["query"]:
                return obj
        except Exception:
            pass
        self.fallbacks += 1
        return self._fallback.understand(utterance, context)

    def ask(self, candidates: list[dict[str, Any]], utterance: str) -> str:
        return self._fallback.ask(candidates, utterance)

    def respond(self, state: dict[str, Any]) -> str:
        return self._fallback.respond(state)
