"""确定性解析器（离线基线 / LLM 不可用时的兜底）。

它是**通用解析器**，不是"给 69 条基准写的正则集"：
所有词（巷道名、side/level 词、货类别名、关系词、单位词）都来自
`config/world.yaml` + `config/semantics.yaml`（经 `depot_vln/catalog.py`）。
换一个仓库（多两条巷道、三个层、新的货类）只改配置，本文件一行不用动。

它只做抽取，不做决策：`kind` 由 `compile.py` 统一决定。
"""
from __future__ import annotations

from typing import Any

from depot_vln.agent.nlu.base import NLU, Slots
from depot_vln.agent.nlu.compile import compile_query
from depot_vln.catalog import get_catalog


class RuleNLU:
    name = "rule"

    # ------------------------------------------------------------ understand
    def extract(self, utterance: str) -> Slots:
        cat = get_catalog()
        u = (utterance or "").strip()
        s = Slots(relations=cat.relations_in_text(u))

        addr = cat.parse_address(u)
        if addr is not None:
            s.address = addr
            s.evidence["address"] = str(addr)
            return s

        raw = cat.loose_address(u)
        if raw:
            s.raw_address = raw
            s.evidence["raw_address"] = raw
            return s

        s.aisle = cat.parse_aisle(u)
        if s.aisle:
            s.evidence["aisle"] = s.aisle
        else:
            s.raw_aisle = cat.loose_aisle(u)
        s.bay = cat.parse_bay(u)
        if s.bay is not None:
            s.evidence["bay"] = str(s.bay)
        s.side = cat.parse_side(u)
        if s.side:
            s.evidence["side"] = s.side
        s.level = cat.parse_level(u)
        if s.level:
            s.evidence["level"] = str(s.level)
        s.classes = cat.classes_in_text(u)
        if s.classes:
            s.evidence["classes"] = ",".join(sorted(s.classes))
        return s

    def understand(self, utterance: str, context: dict[str, Any]) -> dict[str, Any]:
        s = self.extract(utterance)
        q = compile_query(s)
        return {
            "intent": "goto" if q.get("kind") != "unknown" else "unknown",
            "query": q,
            "slots": s.as_dict(),
            "backend": self.name,
            "dropped": [],
        }

    # ------------------------------------------------------------ ask / respond
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
        n = len(candidates)
        tail = f"（共 {n} 个候选）" if n > len(names) else ""
        return f"「{utterance}」有几个可能的货位：{listed}{tail}。你指的是哪一个？"

    def respond(self, state: dict[str, Any]) -> str:
        rc = state.get("reason_code")
        target = state.get("target") or {}
        plan = state.get("plan") or {}
        obs = state.get("observation") or {}
        if rc == "OK":
            if target.get("kind") == "slot":
                tail = ""
                if obs.get("ok"):
                    # 未知（dry 或只拿到低置信）时**不许**编"空位"
                    if obs.get("occupied") is None:
                        tail = "现场核验：未取到占用判定"
                    else:
                        tail = f"现场核验：{'有货' if obs.get('occupied') else '空位'}"
                    cls = obs.get("cls")
                    if cls:
                        tail += f"，类别 {cls}"
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
                return f"【改道】原目标 {sf} 走不通，" + txt if sf else txt
            if target.get("kind") == "rack_face":
                return (f"已到达 {target.get('aisle')}-{target.get('side')} 面货架"
                        f"（走 {plan.get('path_length', 0):.1f} m）。")
            return f"已到达 {target.get('aisle')} 巷。"

        table = {
            "AMBIGUOUS": "我不确定你指的是哪一个，",
            "CLARIFY_LIMIT": "已经问了两轮还是没法确定，请直接给完整地址（例如 B-L-07-2）。",
            "BLOCKED": "去不了：{why}（原因码 BLOCKED）。{tail}",
            "UNOBSERVED": "这个信息我还没观测过（原因码 UNOBSERVED）—— 不能替你猜，"
                          "要我开过去现场看一眼吗？",
            "NO_SLOT": "没有这个货位（原因码 NO_SLOT）。请确认巷道/bay/层号。",
            "EMPTY_BAY": "那个 bay 是空的，没有你说的货（原因码 EMPTY_BAY）。",
            "VERIFY_MISMATCH": "到了位置，但现场看到的和地图记录不一致（原因码 VERIFY_MISMATCH），已标记为待复核。",
            "UNREACHABLE": "图上找不到能到那里的路径（原因码 UNREACHABLE）。",
            "RECOVER_LIMIT": "试了两次恢复都失败，交还给你处理（原因码 RECOVER_LIMIT）。",
            "UNKNOWN_INTENT": "没听懂，能换种说法吗？比如「去 B-L-07-2」。",
        }
        if rc == "AMBIGUOUS":
            return table[rc] + (state.get("question") or "能说得更具体些吗？")
        if rc in table:
            msg = table[rc]
            if rc == "BLOCKED":
                sug = state.get("suggest")
                tail = f"最近的同类可达位是 {sug}，要我改去那里吗？" if sug else ""
                return msg.format(why=state.get("question") or "该巷道被路障封堵", tail=tail)
            return msg
        return f"失败：{rc}"
