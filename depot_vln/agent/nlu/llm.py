"""LLM 抽槽位 + **落地校验**（可选后端，本地 ollama）。

为什么不是"让模型直接给 query"
------------------------------
裸提示词下，模型会把「去 10 号位」补成 `C-R-10-1` —— 图上于是只剩一个候选，
歧义检测永远不触发。系统**不知道自己被猜了**，这才是"静默猜"的来源
（重构前的实测：qwen2.5:7b 72.5%、qwen3-vl 75.4%，都有静默猜）。

三条约束（缺一不可）
  1. 模型只填槽位，不产 kind：`Slots` 结构里根本没有 kind 字段；
  2. 每个值必须在原话里找得到文字依据（grounding），否则丢弃并**记账**；
  3. 信息不足 → `compile_query` 产出欠定 query → 图上多候选 → 发问。

模型不可用 / 输出不可解析时退回规则解析器，并计入 `fallbacks`（静默回退会伪造好成绩）。
"""
from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass, field
from typing import Any

from depot_vln.agent.nlu.base import NLU, Slots
from depot_vln.agent.nlu.compile import compile_query
from depot_vln.agent.nlu.rule import RuleNLU
from depot_vln.catalog import get_catalog


@dataclass
class LLMNLU:
    model: str = "qwen2.5:7b"
    host: str = "http://127.0.0.1:11434"
    name: str = "llm"
    temperature: float = 0.0
    _fallback: RuleNLU = field(default_factory=RuleNLU)
    fallbacks: int = 0
    dropped_total: int = 0

    # ------------------------------------------------------------ 模型
    def _chat(self, system: str, user: str) -> str:
        from langchain_ollama import ChatOllama

        mode = os.environ.get("LLM_REASONING", "auto")
        kw: dict[str, Any] = {}
        if mode == "off" or (mode == "auto" and self.model.startswith("qwen3")):
            kw["reasoning"] = False
        elif mode not in ("auto", "", "none"):
            kw["reasoning"] = mode
        llm = ChatOllama(model=self.model, base_url=self.host, temperature=self.temperature, **kw)
        msg = llm.invoke([("system", system), ("human", user)])
        return str(getattr(msg, "content", msg))

    def _system(self) -> str:
        card = get_catalog().schema_card()
        return (
            "你是仓库指令的**槽位抽取器**。只输出 JSON，不要解释。\n"
            f"世界 schema：{json.dumps(card, ensure_ascii=False)}\n"
            "输出格式：{\"slots\": {\"address\": str|null, \"aisle\": str|null, "
            "\"bay\": int|null, \"side\": \"L\"|\"R\"|null, \"level\": int|null, "
            "\"classes\": [str], \"relations\": [str]}}\n"
            "关系词只能是：goto, blocked, empty, farthest, most, unknown_objects。\n"
            "规则：原话里没说的槽位一律填 null / []，**绝对不要补全**。"
            "例如「去 10 号位」只能给 bay=10，不给 aisle/side/level。"
        )

    # ------------------------------------------------------------ 落地校验
    def _ground(self, raw: dict[str, Any], utterance: str) -> tuple[Slots, list[str]]:
        cat = get_catalog()
        low = (utterance or "").lower()
        s = Slots(relations=cat.relations_in_text(utterance))
        dropped: list[str] = []

        def has(*needles: str) -> bool:
            return any(n and n.lower() in low for n in needles)

        # address：必须原话里真的有
        if raw.get("address"):
            addr = cat.parse_address(str(raw["address"]))
            if addr is not None and str(addr).lower().replace("-", "") in low.replace("-", ""):
                s.address = addr
                s.evidence["address"] = str(addr)
            else:
                dropped.append(f"address={raw['address']!r}（原话无依据或非法）")

        if s.address is None:
            if raw.get("aisle") and has(str(raw["aisle"])):
                a = str(raw["aisle"]).upper()
                if cat.is_aisle(a):
                    s.aisle, s.evidence["aisle"] = a, a
                else:
                    dropped.append(f"aisle={raw['aisle']!r}（不在世界 schema）")
            elif raw.get("aisle"):
                dropped.append(f"aisle={raw['aisle']!r}（原话无依据）")

            if raw.get("bay") is not None:
                try:
                    bay = int(raw["bay"])
                except (TypeError, ValueError):
                    bay = None
                if bay is not None and has(str(bay)) and cat.valid_bay(bay):
                    s.bay, s.evidence["bay"] = bay, str(bay)
                else:
                    dropped.append(f"bay={raw.get('bay')!r}（原话无依据或越界）")

            if raw.get("side"):
                side = cat.side_from_word(str(raw["side"]))
                words = cat.side_words.get(side, ()) if side else ()
                if side and has(*words):
                    s.side, s.evidence["side"] = side, side
                else:
                    dropped.append(f"side={raw['side']!r}（原话无依据）")

            if raw.get("level") is not None:
                try:
                    lv = int(raw["level"])
                except (TypeError, ValueError):
                    lv = None
                words = cat.level_words.get(lv, ()) if lv else ()
                if lv in cat.level_numbers() and has(*words, str(lv)):
                    s.level, s.evidence["level"] = lv, str(lv)
                else:
                    dropped.append(f"level={raw.get('level')!r}（原话无依据或越界）")

            # classes：模型给的类别同样要有依据；模型漏掉时用词表补（词表在配置里，可信）
            model_classes = {str(c) for c in (raw.get("classes") or []) if c}
            grounded = {c for c in model_classes if c in cat.class_names and has(*cat.classes[c].words)}
            for c in sorted(model_classes - grounded):
                dropped.append(f"class={c!r}（原话无依据或不在 schema）")
            s.classes = grounded or cat.classes_in_text(utterance)
            if s.classes:
                s.evidence["classes"] = ",".join(sorted(s.classes))

        rels = {str(r) for r in (raw.get("relations") or []) if r}
        s.relations |= rels & set(cat.lexicon)
        return s, dropped

    # ------------------------------------------------------------ 入口
    def understand(self, utterance: str, context: dict[str, Any]) -> dict[str, Any]:
        raw: dict[str, Any] | None = None
        try:
            text = self._chat(self._system(), utterance)
            m = re.search(r"\{.*\}", text, re.S)
            obj = json.loads(m.group(0)) if m else None
            if isinstance(obj, dict):
                raw = obj.get("slots") if isinstance(obj.get("slots"), dict) else obj
        except Exception:
            raw = None

        if not isinstance(raw, dict):
            self.fallbacks += 1
            out = self._fallback.understand(utterance, context)
            out["backend"] = f"{self.name}->rule(fallback)"
            return out

        slots, dropped = self._ground(raw, utterance)
        # 结构化地址/关系这类"文本里有唯一写法"的东西，永远以确定性抽取为准（可复现）
        rule_slots = self._fallback.extract(utterance)
        if rule_slots.address is not None:
            slots.address = rule_slots.address
        if rule_slots.raw_address:
            slots.raw_address = rule_slots.raw_address
        slots.relations |= rule_slots.relations

        self.dropped_total += len(dropped)
        q = compile_query(slots)
        return {
            "intent": "goto" if q.get("kind") != "unknown" else "unknown",
            "query": q,
            "slots": slots.as_dict(),
            "backend": self.name,
            "dropped": dropped,
        }

    def ask(self, candidates: list[dict[str, Any]], utterance: str) -> str:
        return self._fallback.ask(candidates, utterance)

    def respond(self, state: dict[str, Any]) -> str:
        return self._fallback.respond(state)

    def stats(self) -> dict[str, int]:
        return {"fallbacks": self.fallbacks, "dropped": self.dropped_total}
