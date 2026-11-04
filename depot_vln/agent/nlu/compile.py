"""槽位 → query 的编译器（**唯一**决定"要不要发问"的地方）。

为什么单独一个函数
------------------
旧实现的每个分支里都直接返回 `{"kind": ...}`，于是"哪些信息缺失"散落在 20 个
if 分支里，想加一种说法就要改一处、并祈祷别的地方不冲突。

现在抽取与决策分离：
    抽取器（rule / llm）  只回答"原话里出现了什么"
    编译器（本文件）      只回答"这些事实够不够决定一个目标"

够不够的判据很朴素：**信息不足时照样产出 query，但产出一个欠定的 query**，
让图上的 `resolve` 必然得到多个候选 → `AMBIGUOUS` → `clarify`。
所以"歧义必须发问"不依赖任何 if-else 的完备性。
"""
from __future__ import annotations

import re
from typing import Any

from depot_vln.agent.nlu.base import Slots
from depot_vln.catalog import get_catalog


def compile_query(s: Slots) -> dict[str, Any]:
    cat = get_catalog()

    # 1) 完整地址（含"看起来像地址但非法"的情况）
    if s.address is not None:
        return {"kind": "address", "address": str(s.address)}
    if s.raw_address:
        return {"kind": "address", "address": s.raw_address}

    # 2) 明确提到封堵
    if s.blocked:
        return {"kind": "blocked_aisle", "aisle": s.aisle}

    # 3) 闭集之外的物体（"冰箱"）：认出是未知类别，而不是猜最近的
    if s.unknown_object and not s.classes:
        return {"kind": "max_class", "classes": ["__unknown__"]}

    # 4) 类别 × 位置
    if s.classes:
        if s.bay is not None and s.aisle:
            return _with_common({
                "kind": "class_in_bay", "aisle": s.aisle, "bay": s.bay,
                "classes": sorted(s.classes),
            }, s)
        if s.most or not s.aisle:
            q: dict[str, Any] = {"kind": "max_class", "classes": sorted(s.classes)}
            if s.aisle:
                q["aisle"] = s.aisle
            return q
        return _with_common({
            "kind": "class_in_aisle", "aisle": s.aisle, "classes": sorted(s.classes),
        }, s)

    # 5) bay 号（可能带巷/侧/层约束）
    if s.bay is not None:
        if s.aisle and s.empty:
            return _with_common({
                "kind": "empty_slot_in_bay", "aisle": s.aisle, "bay": s.bay}, s)
        return _with_common({"kind": "bay", "bay": s.bay}, s)

    # 6) 巷 × 关系
    if s.aisle and s.farthest:
        return _with_common({"kind": "farthest", "aisle": s.aisle, "empty": s.empty}, s)
    if s.aisle and s.empty:
        # "A 巷的空位"：没说哪一格 -> 交给 farthest(empty) 的多候选路径去发问
        return _with_common({"kind": "farthest", "aisle": s.aisle, "empty": True}, s)
    if s.aisle:
        return _with_common({"kind": "aisle", "aisle": s.aisle}, s)
    if s.raw_aisle:
        return {"kind": "aisle", "aisle": s.raw_aisle}   # 不存在 -> NO_SLOT

    return {"kind": "unknown"}


def _with_common(q: dict[str, Any], s: Slots) -> dict[str, Any]:
    """把 side / level 这类"可选约束"挂上去（None 不挂，表示没指定）。"""
    if s.side:
        q["side"] = s.side
    if s.level:
        q["level"] = s.level
    if s.aisle and "aisle" not in q:
        q["aisle"] = s.aisle
    return q


def parse_clarification(text: str) -> dict[str, Any]:
    """把用户对澄清问题的回答变成约束条件（catalog 驱动，不写死巷道表）。"""
    cat = get_catalog()
    low = (text or "").strip().lower()
    c: dict[str, Any] = {}

    addr = cat.parse_address(low)
    if addr is not None:
        return {"aisle": addr.aisle, "side": addr.side, "bay": addr.bay,
                **({"level": addr.level} if addr.level else {})}

    aisle = cat.parse_aisle(low)
    if aisle:
        c["aisle"] = aisle
    side = cat.parse_side(low)
    if side:
        c["side"] = side
    level = cat.parse_level(low)
    if level:
        c["level"] = level

    # bay：必须带单位词，否则"左边第一层"里的数字会被误读成 bay
    m = re.search(r"(?<!\d)(\d{1,3})\s*(?:号位|号|位|bay|slot)", low)
    if m is None:
        m = re.search(r"\bbay\s*(\d{1,3})\b", low)
    if m:
        bay = int(m.group(1))
        if cat.valid_bay(bay):
            c["bay"] = bay
    return c


def matches_constraints(target: dict[str, Any], c: dict[str, Any]) -> bool:
    """澄清回答产生的约束是否命中某个候选目标。None 视为"未指定"，不算失配。"""
    if not c:
        return True
    for key in ("aisle", "side", "level", "bay"):
        if key in c and target.get(key) not in (None, c[key]):
            return False
    return True
