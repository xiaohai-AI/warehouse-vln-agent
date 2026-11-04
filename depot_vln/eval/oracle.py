"""基准的独立复核器（oracle）。

职责：拿 **ground_truth** 检查 `cases.yaml` 里每条人工声明是否成立。
它不解析自然语言、不跑 agent、也不知道解析器怎么写 —— 这正是它与被测系统的独立性来源。

一条声明与真值矛盾时（比如声称 A-R 有油桶、真值里油桶其实在 A-L），
`validate` 会返回问题清单，**基准拒绝运行**，而不是悄悄扣分。
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[2]
CASES_PATH = Path(__file__).resolve().parent / "cases.yaml"

CARTON = {"carton_s", "carton_m", "carton_l"}


def _truth() -> dict[str, Any]:
    return json.loads((ROOT / "ground_truth" / "slots.json").read_text(encoding="utf-8"))["slots"]


def _aisles() -> dict[str, Any]:
    return json.loads((ROOT / "ground_truth" / "aisles.json").read_text(encoding="utf-8"))


def load_cases(path: str | Path | None = None) -> list[dict[str, Any]]:
    obj = yaml.safe_load((Path(path) if path else CASES_PATH).read_text(encoding="utf-8"))
    return list(obj.get("cases") or [])


def validate(case: dict[str, Any], truth: dict[str, Any] | None = None,
             aisles: dict[str, Any] | None = None) -> list[str]:
    """返回该条声明的**问题清单**；空列表 = 声明与真值一致。"""
    truth = truth if truth is not None else _truth()
    aisles = aisles if aisles is not None else _aisles()
    exp = case.get("expect") or {}
    beh = exp.get("behavior")
    problems: list[str] = []

    def check_reachable(addr: str, label: str) -> None:
        rec = truth.get(addr)
        if rec is None:
            problems.append(f"{label}: {addr} 不在真值表里")
        elif not rec.get("reachable"):
            problems.append(f"{label}: {addr} 真值里不可达，不能当 reach 目标")

    def check_face(face: str) -> None:
        aisle, _, side = face.partition("-")
        if f"{aisle}-{side}" not in (aisles.get("rack_faces") or {}):
            problems.append(f"reach: 货架面 {face} 不存在")

    if beh == "reach":
        target = str(exp.get("target") or "")
        if not target:
            problems.append("reach: 缺 target")
        elif "-" in target and target.count("-") == 1:
            check_face(target)
        elif target.count("-") >= 3:
            check_reachable(target, "reach")
        elif target in {a["name"] for a in aisles.get("bays", [])}:
            pass
        else:
            problems.append(f"reach: 无法识别的 target {target!r}")

    elif beh == "clarify":
        after = str(exp.get("after_clarify") or "")
        if not exp.get("answer"):
            problems.append("clarify: 缺 answer（澄清回答）")
        if after.count("-") >= 3:
            check_reachable(after, "clarify")
        elif after.count("-") == 1:
            check_face(after)
        elif not after:
            problems.append("clarify: 缺 after_clarify")

    elif beh == "reject":
        reason = exp.get("reason")
        if reason not in ("BLOCKED", "NO_SLOT", "EMPTY_BAY"):
            problems.append(f"reject: 未知 reason {reason!r}")
        insp = exp.get("inspect") or {}
        if reason == "BLOCKED" and insp.get("address"):
            rec = truth.get(insp["address"])
            if rec is not None and rec.get("reachable"):
                problems.append(f"BLOCKED: {insp['address']} 真值里其实可达")
        if reason == "EMPTY_BAY" and insp.get("bay"):
            aisle, bay = insp["bay"]
            levels = insp.get("levels") or [1, 2]
            for side in ("L", "R"):
                for lv in levels:
                    rec = truth.get(f"{aisle}-{side}-{int(bay):02d}-{lv}")
                    if rec and rec["occupied"]:
                        problems.append(
                            f"EMPTY_BAY: {aisle}-{side}-{int(bay):02d}-{lv} 真值有货，不是空 bay")
    else:
        problems.append(f"未知 behavior {beh!r}")

    return problems


def validate_all(cases: list[dict[str, Any]] | None = None) -> dict[str, list[str]]:
    cases = cases if cases is not None else load_cases()
    truth, aisles = _truth(), _aisles()
    seen: set[str] = set()
    out: dict[str, list[str]] = {}
    for c in cases:
        cid = str(c.get("id"))
        probs = validate(c, truth, aisles)
        if cid in seen:
            probs.append("重复的 id")
        seen.add(cid)
        if probs:
            out[cid] = probs
    return out


if __name__ == "__main__":
    bad = validate_all()
    if bad:
        for cid, probs in bad.items():
            print(f"[BAD] {cid}")
            for p in probs:
                print(f"      - {p}")
        raise SystemExit(1)
    print(f"[oracle] {len(load_cases())} 条声明全部与真值一致")
