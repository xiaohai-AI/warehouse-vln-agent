"""语言层的数据契约。"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Protocol

from depot_vln.catalog import Address


@dataclass
class Slots:
    """从一句话里抽出来的**事实**（不含任何决策）。

    注意这里没有 `kind`：决定"这是哪种查询"是 `compile.py` 的职责。
    模型即使想越权直接给 kind，也没有字段可以放 —— 这是类型层面的约束。
    """

    address: Address | None = None
    #: 外形像地址、但不合法的原始串（如 Z-L-01-1 / A-L-01-3）—— 必须走 NO_SLOT，
    #: 不许降级成"某条巷子"然后给个别的答案。
    raw_address: str | None = None
    #: 外形像巷道、但世界里没有（"Q 巷"）—— 同样必须走 NO_SLOT
    raw_aisle: str | None = None
    aisle: str | None = None
    bay: int | None = None
    side: str | None = None
    level: int | None = None
    classes: set[str] = field(default_factory=set)
    relations: set[str] = field(default_factory=set)
    #: 每个槽位值在原话里的依据（grounding）。空 = 没有依据，值不可信。
    evidence: dict[str, str] = field(default_factory=dict)

    # ---- 关系词快捷判定（词表来自 config/semantics.yaml，不是 Python 常量）
    def has(self, relation: str) -> bool:
        return relation in self.relations

    @property
    def empty(self) -> bool:
        return self.has("empty")

    @property
    def farthest(self) -> bool:
        return self.has("farthest")

    @property
    def most(self) -> bool:
        return self.has("most")

    @property
    def blocked(self) -> bool:
        return self.has("blocked")

    @property
    def unknown_object(self) -> bool:
        return self.has("unknown_objects")

    def as_dict(self) -> dict[str, Any]:
        return {
            "address": str(self.address) if self.address else self.raw_address,
            "aisle": self.aisle,
            "bay": self.bay,
            "side": self.side,
            "level": self.level,
            "classes": sorted(self.classes),
            "relations": sorted(self.relations),
            "evidence": dict(self.evidence),
        }


@dataclass
class Understanding:
    """理解层的输出：意图 + 查询 + 溯源。"""

    intent: str
    query: dict[str, Any]
    slots: Slots
    backend: str = "rule"
    #: 被丢弃的不可采信槽位（模型幻觉、越界值…），必须记账，不能静默
    dropped: list[str] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        return {
            "intent": self.intent,
            "query": dict(self.query),
            "slots": self.slots.as_dict(),
            "backend": self.backend,
            "dropped": list(self.dropped),
        }


class NLU(Protocol):
    """后端协议。三个方法都与具体世界无关（世界事实从 catalog 来）。"""

    name: str

    def understand(self, utterance: str, context: dict[str, Any]) -> dict[str, Any]: ...

    def ask(self, candidates: list[dict[str, Any]], utterance: str) -> str: ...

    def respond(self, state: dict[str, Any]) -> str: ...
