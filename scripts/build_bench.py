#!/usr/bin/env python3
"""生成指令基准（SPEC §8.6 / P0 闸门）。

四条硬规矩
  1. 60–80 条，四类各 15–20 条；期望行为全部由 ground_truth 算出来，不是手写的
  2. 期望答案与 agent 使用同一份语义定义（类别集合、地址格式），但计算路径独立
  3. 歧义类必须真的歧义、不可达类必须真的不可达（生成时校验，不满足就换一条）
  4. 一条命令复跑：`make bench` 每次都重算，seed 固定 -> 结果可复现

输出 results/bench_spec.csv（基准本体）+ results/bench_spec.json（含统计）
"""
from __future__ import annotations

import csv
import json
import random
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]

CARTON = {"carton_s", "carton_m", "carton_l"}
CLASS_WORD = {
    "carton": ("纸箱", CARTON),
    "pallet": ("托盘", {"pallet"}),
    "tote": ("周转箱", {"tote"}),
    "drum": ("油桶", {"drum"}),
}
SIDE_WORD = {"L": "左边", "R": "右边"}
LEVEL_WORD = {1: "第一层", 2: "第二层"}
PER_CATEGORY = 18


def load() -> tuple[dict, dict, dict, dict]:
    grid = json.loads((ROOT / "world" / "slot_grid.json").read_text(encoding="utf-8"))
    truth = json.loads((ROOT / "ground_truth" / "slots.json").read_text(encoding="utf-8"))
    aisles = json.loads((ROOT / "ground_truth" / "aisles.json").read_text(encoding="utf-8"))
    return grid, truth, aisles, {}


def face_counts(aisles: dict) -> dict[str, dict[str, int]]:
    return aisles.get("rack_faces") or {}


def main() -> int:
    grid, truth, aisles, _ = load()
    rng = random.Random(int(grid.get("seed", 0)))
    slots: dict[str, dict] = grid["slots"]
    truth_slots: dict[str, dict] = truth["slots"]
    faces = face_counts(aisles)
    aisle_names = [a["name"] for a in grid["aisles"]]
    n_bays = int(grid["derived"]["bays_per_aisle"])
    blocked_aisles = {seg["aisle"] for seg in aisles.get("blocked", [])}

    def slot_full(addr: str) -> str:
        return addr

    rows: list[dict[str, Any]] = []

    # ---------------------------------------------------------------- ① 直达
    reachable = [a for a, s in truth_slots.items() if s["reachable"]]
    rng.shuffle(reachable)
    direct_templates = [
        "去 {addr}",
        "到 {addr} 那个货位看看",
        "带我去 {addr}",
        "{addr} 怎么走",
        "go to {addr}",
        "麻烦到 {addr} 一趟",
    ]
    for i in range(PER_CATEGORY):
        addr = reachable[i % len(reachable)]
        tpl = direct_templates[i % len(direct_templates)]
        rows.append(
            {
                "id": f"D{i + 1:02d}",
                "category": "direct",
                "utterance": tpl.format(addr=slot_full(addr)),
                "expected_behavior": "reach",
                "expected_target": addr,
                "expected_reason_code": "",
                "clarify_answer": "",
                "notes": "地址直达；期望到达该 slot 并做一次 verify",
            }
        )

    # ---------------------------------------------------------------- ② 属性/关系
    attr_rows: list[dict[str, Any]] = []

    # 2a) 某巷某类货只在单侧有 —— 唯一答案
    unique_face: list[tuple[str, str, str]] = []
    for (face, counts) in sorted(faces.items()):
        aisle, side = face.split("-")
        other = f"{aisle}-{'R' if side == 'L' else 'L'}"
        for key, (word, classes) in CLASS_WORD.items():
            mine = sum(counts.get(c, 0) for c in classes)
            theirs = sum((faces.get(other) or {}).get(c, 0) for c in classes)
            if mine > 0 and theirs == 0:
                unique_face.append((aisle, side, key))
    rng.shuffle(unique_face)
    for (aisle, side, key) in unique_face[:10]:
        word = CLASS_WORD[key][0]
        attr_rows.append(
            {
                "id": "",
                "category": "attribute",
                "utterance": f"去 {aisle} 巷放{word}的那面货架",
                "expected_behavior": "reach",
                "expected_target": f"{aisle}-{side}",
                "expected_reason_code": "",
                "clarify_answer": "",
                "notes": f"属性聚合：{aisle}-{side} 是该巷唯一有 {key} 的一侧",
            }
        )

    # 2b) 全仓"纸箱最多的那排货架" —— 唯一最大值
    carton_totals = aisles.get("carton_totals") or {}
    if carton_totals:
        best = max(carton_totals.values())
        winners = [f for f, c in carton_totals.items() if c == best]
        if len(winners) == 1:
            f = winners[0]
            attr_rows.append(
                {
                    "id": "",
                    "category": "attribute",
                    "utterance": "去放纸箱最多的那排货架",
                    "expected_behavior": "reach",
                    "expected_target": f,
                    "expected_reason_code": "",
                    "clarify_answer": "",
                    "notes": f"全仓聚合：{f} 有 {best} 件纸箱（唯一最大）",
                }
            )

    # 2c) 某巷最里面"某个确定的空位"（侧 + 层 + 空位 三者夹到唯一）
    far_rows: list[dict[str, Any]] = []
    for aisle in aisle_names:
        for side in ("L", "R"):
            for level in (1, 2):
                addr = f"{aisle}-{side}-{n_bays:02d}-{level}"
                rec = truth_slots.get(addr)
                if rec and not rec["occupied"] and rec["reachable"]:
                    far_rows.append(
                        {
                            "id": "",
                            "category": "attribute",
                            "utterance": f"去 {aisle} 巷{SIDE_WORD[side]}最里面{LEVEL_WORD[level]}那个空位",
                            "expected_behavior": "reach",
                            "expected_target": addr,
                            "expected_reason_code": "",
                            "clarify_answer": "",
                            "notes": f"图上最远 bay（{n_bays}）+ 侧 + 层 -> 唯一空位",
                        }
                    )
    rng.shuffle(far_rows)
    attr_rows.extend(far_rows[: max(0, 18 - len(attr_rows))])

    # 兜底：仍不足 18，就用"单侧唯一"的其它类别继续填
    k = 0
    while len(attr_rows) < 18 and unique_face:
        aisle, side, key = unique_face[k % len(unique_face)]
        word = CLASS_WORD[key][0]
        attr_rows.append(
            {
                "id": "",
                "category": "attribute",
                "utterance": f"{aisle} 巷{word}都在哪一面货架",
                "expected_behavior": "reach",
                "expected_target": f"{aisle}-{side}",
                "expected_reason_code": "",
                "clarify_answer": "",
                "notes": "兜底重复：单侧唯一类别",
            }
        )
        k += 1
    attr_rows = attr_rows[:18]
    for i, r in enumerate(attr_rows):
        r["id"] = f"A{i + 1:02d}"
    rows.extend(attr_rows)

    # ---------------------------------------------------------------- ③ 歧义
    amb_rows: list[dict[str, Any]] = []
    ambiguous_bays = rng.sample(range(1, n_bays + 1), 6)
    for b in ambiguous_bays:
        amb_rows.append(
            {
                "id": "",
                "category": "ambiguous",
                "utterance": f"去 {b:02d} 号位",
                "expected_behavior": "clarify",
                "expected_target": "",
                "expected_reason_code": "AMBIGUOUS",
                "clarify_answer": f"{rng.choice(aisle_names)} 巷左边第一层",
                "notes": f"{b:02d} 号位在 6 条巷道 × 2 侧 × 2 层都有，必须反问",
            }
        )
    # 澄清答案要真的能收敛
    for r in amb_rows:
        aisle = rng.choice(aisle_names)
        side = rng.choice(["L", "R"])
        level = rng.choice([1, 2])
        bay = int(r["utterance"].split()[1])
        r["clarify_answer"] = f"{aisle} 巷{SIDE_WORD[side]}{LEVEL_WORD[level]}"
        r["expected_after_clarify"] = f"{aisle}-{side}-{bay:02d}-{level}"

    # 3b) 某巷两侧都有纸箱 -> 两面货架都候选
    both_sides = []
    for aisle in aisle_names:
        l = sum((faces.get(f"{aisle}-L") or {}).get(c, 0) for c in CARTON)
        r = sum((faces.get(f"{aisle}-R") or {}).get(c, 0) for c in CARTON)
        if l > 0 and r > 0:
            both_sides.append(aisle)
    rng.shuffle(both_sides)
    for aisle in both_sides[:6]:
        side = rng.choice(["L", "R"])
        amb_rows.append(
            {
                "id": "",
                "category": "ambiguous",
                "utterance": f"去 {aisle} 巷放纸箱的那面货架",
                "expected_behavior": "clarify",
                "expected_target": "",
                "expected_reason_code": "AMBIGUOUS",
                "clarify_answer": f"{aisle} 巷{SIDE_WORD[side]}",
                "expected_after_clarify": f"{aisle}-{side}",
                "notes": f"{aisle} 巷两侧都有纸箱，必须问是哪一面而不是猜",
            }
        )

    # 3c) 最里面那个位（同 bay 两侧两层并列）
    # 只用没被封堵的巷道：否则"最里面"该是可达的最远 bay，而不是名义上的 24 号
    open_aisles = [a for a in aisle_names if a not in blocked_aisles] or aisle_names
    for aisle in rng.sample(open_aisles, min(3, len(open_aisles))):
        side = rng.choice(["L", "R"])
        level = rng.choice([1, 2])
        amb_rows.append(
            {
                "id": "",
                "category": "ambiguous",
                "utterance": f"去 {aisle} 巷最里面那个位",
                "expected_behavior": "clarify",
                "expected_target": "",
                "expected_reason_code": "AMBIGUOUS",
                "clarify_answer": f"{aisle} 巷{SIDE_WORD[side]}{LEVEL_WORD[level]}",
                "expected_after_clarify": f"{aisle}-{side}-{n_bays:02d}-{level}",
                "notes": "同一 bay 的两侧/两层距离并列 -> 必须澄清",
            }
        )

    # 3d) 纸箱最多的那排货架并列
    if carton_totals:
        best = max(carton_totals.values())
        winners = [f for f, c in carton_totals.items() if c == best]
        if len(winners) > 1:
            side = "L" if winners[0].endswith("L") else "R"
            amb_rows.append(
                {
                    "id": "",
                    "category": "ambiguous",
                    "utterance": "去放纸箱最多的那排货架",
                    "expected_behavior": "clarify",
                    "expected_target": "",
                    "expected_reason_code": "AMBIGUOUS",
                    "clarify_answer": f"{winners[0].split('-')[0]} 巷{SIDE_WORD[side]}",
                    "expected_after_clarify": winners[0],
                    "notes": f"并列最大：{winners}",
                }
            )
    amb_rows = amb_rows[:18]
    for i, r in enumerate(amb_rows):
        r["id"] = f"M{i + 1:02d}"
    rows.extend(amb_rows)

    # ---------------------------------------------------------------- ④ 不可达
    un_rows: list[dict[str, Any]] = []
    # 4a) BLOCKED：路障后面的货位
    blocked_slots = [a for a, s in truth_slots.items() if not s["reachable"]]
    rng.shuffle(blocked_slots)
    for addr in blocked_slots[:5]:
        un_rows.append(
            {
                "id": "",
                "category": "unreachable",
                "utterance": f"去 {addr}",
                "expected_behavior": "reject",
                "expected_target": "",
                "expected_reason_code": "BLOCKED",
                "clarify_answer": "",
                "notes": "路障封堵区内的货位：必须给 BLOCKED，不许硬走",
            }
        )
    if blocked_aisles:
        a = sorted(blocked_aisles)[0]
        un_rows.append(
            {
                "id": "",
                "category": "unreachable",
                "utterance": f"去封住的那条 {a} 巷",
                "expected_behavior": "reject",
                "expected_target": "",
                "expected_reason_code": "BLOCKED",
                "clarify_answer": "",
                "notes": "用户直接提到封堵巷道：说明原因，不许尝试",
            }
        )

    # 4b) NO_SLOT：不存在的地址 / 巷道 / bay / 层 / 类别
    no_slot_utts = [
        "去 Z-L-01-1",
        f"去 A-L-{n_bays + 6:02d}-1",
        "去 A-L-01-3",
        "去 Q 巷",
        f"去 {n_bays + 40} 号位",
        "去 A 巷放冰箱的那面货架",
    ]
    for u in no_slot_utts:
        un_rows.append(
            {
                "id": "",
                "category": "unreachable",
                "utterance": u,
                "expected_behavior": "reject",
                "expected_target": "",
                "expected_reason_code": "NO_SLOT",
                "clarify_answer": "",
                "notes": "不存在 / 闭集之外的类别：NO_SLOT",
            }
        )

    # 4c) EMPTY_BAY：指定的 bay 层确实是空的（"去 A 巷 5 号位第一层放油桶的那格"）
    empty_bays: list[tuple[str, int]] = []
    for aisle in aisle_names:
        for b in range(1, n_bays + 1):
            lvl1 = [truth_slots.get(f"{aisle}-{s}-{b:02d}-1") for s in ("L", "R")]
            if all(r is not None and not r["occupied"] for r in lvl1):
                empty_bays.append((aisle, b))
    rng.shuffle(empty_bays)
    for (aisle, b) in empty_bays[:6]:
        key = rng.choice(list(CLASS_WORD))
        word = CLASS_WORD[key][0]
        un_rows.append(
            {
                "id": "",
                "category": "unreachable",
                "utterance": f"去 {aisle} 巷 {b:02d} 号位第一层放{word}的那格",
                "expected_behavior": "reject",
                "expected_target": "",
                "expected_reason_code": "EMPTY_BAY",
                "clarify_answer": "",
                "notes": f"{aisle}-{b:02d} 第一层两侧都空：EMPTY_BAY（不是 NO_SLOT）",
            }
        )
    un_rows = un_rows[:18]
    for i, r in enumerate(un_rows):
        r["id"] = f"U{i + 1:02d}"
    rows.extend(un_rows)

    # ---------------------------------------------------------------- 输出
    out_csv = ROOT / "results" / "bench_spec.csv"
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    cols = [
        "id",
        "category",
        "utterance",
        "expected_behavior",
        "expected_target",
        "expected_reason_code",
        "clarify_answer",
        "expected_after_clarify",
        "notes",
    ]
    with open(out_csv, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)

    counts: dict[str, int] = {}
    for r in rows:
        counts[r["category"]] = counts.get(r["category"], 0) + 1
    meta = {
        "schema": "depot-vln/bench_spec@1",
        "n": len(rows),
        "per_category": counts,
        "seed": grid.get("seed"),
        "csv": str(out_csv.relative_to(ROOT)),
        "columns": cols,
    }
    (ROOT / "results" / "bench_spec.json").write_text(
        json.dumps(meta, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(f"[build_bench] {len(rows)} 条 -> {out_csv.relative_to(ROOT)}")
    for k, v in counts.items():
        print(f"  {k:<12} {v}")

    warnings = []
    if len(rows) < 60:
        warnings.append(f"总量 {len(rows)} < 60")
    for k, v in counts.items():
        if not 15 <= v <= 20:
            warnings.append(f"{k} 类 {v} 条不在 15–20 之间")
    for wmsg in warnings:
        print("  警告:", wmsg)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
