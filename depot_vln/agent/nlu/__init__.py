"""语言理解层。

    base.py      槽位（Slots）与后端协议
    compile.py   槽位 → query**（唯一决定"要不要发问"的地方）
    rule.py      确定性解析器（catalog 驱动，零延迟，离线基线）
    llm.py       LLM 抽槽位 + 落地校验（可选后端）

铁律（SPEC §8.4 的"权威分离"在代码里的落点）：

    * 模型只产出**槽位**，不产出 `query.kind`；
    * 每个槽位值必须能在原话里找到文字依据（grounding），否则丢弃并记账；
    * 信息不足时 `compile_query` **故意**产出欠定的 query → 图上多候选 → 发问。
      所以"不静默猜"是结构性质，不是提示词写得好。
"""
from __future__ import annotations

from depot_vln.agent.nlu.base import NLU, Slots, Understanding
from depot_vln.agent.nlu.compile import compile_query, parse_clarification
from depot_vln.agent.nlu.llm import LLMNLU
from depot_vln.agent.nlu.rule import RuleNLU

BACKENDS = ("rule", "llm")

__all__ = [
    "NLU", "Slots", "Understanding", "RuleNLU", "LLMNLU",
    "compile_query", "parse_clarification", "build_nlu",
]


def build_nlu(backend: str = "rule", *, model: str | None = None, host: str | None = None) -> NLU:
    """按名字造一个后端。离线默认 rule；`llm` 需要本地 ollama。"""
    b = (backend or "rule").strip().lower()
    if b in ("rule", "offline", "deterministic"):
        return RuleNLU()
    if b in ("llm", "ollama"):
        return LLMNLU(**(dict(model=model) if model else {}), **(dict(host=host) if host else {}))
    raise ValueError(f"未知后端 {backend!r}（可选：{', '.join(BACKENDS)}）")
