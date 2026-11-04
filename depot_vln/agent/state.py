"""LangGraph 状态定义（SPEC §8.2）。

State 是显式的：任何节点能读什么、能写什么都在这里，图拓扑即代码。
`node_trace` 用追加 reducer，因为逐货位扫描会用 `Send` 扇出，多分支要能汇聚。
"""
from __future__ import annotations

import operator
from typing import Annotated, Any, Literal, TypedDict

ReasonCode = Literal[
    "OK",
    "AMBIGUOUS",
    "NO_SLOT",
    "EMPTY_BAY",
    "BLOCKED",
    "UNREACHABLE",
    "UNOBSERVED",
    "VERIFY_MISMATCH",
    "TIMEOUT",
    "CLARIFY_LIMIT",
    "RECOVER_LIMIT",
    "UNKNOWN_INTENT",
]


class Candidate(TypedDict, total=False):
    target: dict[str, Any]
    score: float
    why: str
    reachable: bool
    observed_occupied: bool | None
    observed_class: str | None


class TrajectoryStep(TypedDict, total=False):
    seq: int
    tool: str
    args: dict[str, Any]
    ok: bool
    reason_code: str | None
    detail: str
    elapsed: float


class AgentState(TypedDict, total=False):
    # --- 输入
    thread_id: str
    utterance: str
    history: list[dict[str, str]]
    pose: dict[str, float]

    # --- understand
    intent: str            # goto | query | reject
    query: dict[str, Any]  # 结构化槽位（不含坐标）
    constraints: dict[str, Any]
    slots: dict[str, Any]  # 抽取到的原始槽位 + 依据（审计用）
    backend: str           # 这一句是哪个后端理解的（rule / llm）
    dropped: list[str]     # 被落地校验丢弃的槽位（模型幻觉）

    # --- resolve
    candidates: list[Candidate]
    target: dict[str, Any]
    reason_code: str | None

    # --- plan / act
    plan: dict[str, Any] | None
    seq: int
    trajectory: list[TrajectoryStep]

    # --- verify / recover / respond
    observation: dict[str, Any] | None
    answer: str
    question: str
    clarify_answer: str
    clarify_count: int
    recover_count: int
    # **必须声明**：LangGraph 只保留 AgentState 里声明过的通道，未声明字段写入会被静默丢弃。
    # 踩过：`make_recover` 一直在写 `alt_index`，但没声明 → 每轮读回 0 → "换候选"永远挑同一个。
    alt_index: int
    tried: list[str]              # 已试过的地址（防止 A→B→A 来回换）
    substituted_from: str | None  # 语义换位：原目标地址（回答里要明说改道）
    allow_substitute: bool        # 是否允许**擅自**改道（默认关：只提议，不代替用户决定）
    suggest: str | None           # 被挡时建议的替代货位（回答里问用户要不要改去）

    # --- 审计
    node_trace: Annotated[list[str], operator.add]
    notes: Annotated[list[str], operator.add]
