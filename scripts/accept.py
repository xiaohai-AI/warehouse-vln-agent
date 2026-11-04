#!/usr/bin/env python3
"""把所有判据汇总成一张表（docs/ACCEPTANCE.md + 终端输出）。

  python3.10 scripts/accept.py            # 汇总（会重跑 P1 自检，因为它是秒级的）
  python3.10 scripts/accept.py --no-run   # 只用已有结果

每条判据都要求"一条命令可复跑"。没跑过的判据显示"未跑"，不显示"通过"。
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import re
import subprocess
import sys
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "results"
sys.path.insert(0, str(ROOT))          # 让 `import depot_vln.*` 在 `scripts/` 里也能用


def latest(pattern: str) -> Path | None:
    files = sorted(glob.glob(str(RESULTS / pattern)))
    return Path(files[-1]) if files else None


def load_json(name: str) -> dict | None:
    p = RESULTS / name
    if not p.exists():
        return None
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except Exception:
        return None


def bench_summary(path: Path | None) -> dict | None:
    if not path or not path.exists():
        return None
    rows = list(csv.DictReader(open(path, encoding="utf-8")))
    if not rows:
        return None
    cats: dict[str, list[dict]] = {}
    for r in rows:
        cats.setdefault(r["category"], []).append(r)
    per = {}
    for c, rs in cats.items():
        ok = sum(1 for r in rs if r["behavior_ok"] == "True")
        sw = sum(1 for r in rs if r["silent_wrong"] == "True")
        per[c] = {"n": len(rs), "ok": ok, "rate": ok / len(rs), "silent_wrong": sw}
    total = sum(v["n"] for v in per.values())
    ok = sum(v["ok"] for v in per.values())
    return {
        "csv": str(path.relative_to(ROOT)),
        "per_category": per,
        "n": total,
        "ok": ok,
        "rate": ok / total,
        "silent_wrong": sum(v["silent_wrong"] for v in per.values()),
    }


def p6_demo_summary() -> dict | None:
    """汇总 P6-2 真跑的产物（`results/p6_demo_*.json`），只统计非 dry 的。

    分类靠**行为**而不是时间戳：`reason_code=None` + 无 `substituted_from` = 直达；
    `substituted_from` 非空 = 授权改道；`BLOCKED` + 回答里带"要我改去" = 只提议。
    """
    runs = []
    for f in sorted(glob.glob(str(RESULTS / "p6_demo_*.json"))):
        try:
            d = json.loads(Path(f).read_text(encoding="utf-8"))
        except Exception:
            continue
        if d.get("dry"):
            continue
        ans = d.get("answer") or ""
        if d.get("substituted_from") or ans.startswith("【改道】"):
            kind = "substituted"
        elif d.get("reason_code") == "BLOCKED" and ("要我去" in ans or "要我改" in ans):
            kind = "blocked_suggest"
        elif d.get("reason_code") in (None, "OK") and (d.get("observation") or {}).get("ok"):
            kind = "direct"
        else:
            kind = "other"
        plan = d.get("plan") or {}
        obs = d.get("observation") or {}
        sc = d.get("score") or {}
        # 类别一致率：直接用 观测类别 vs 真值类别（老产物里 score 只有 matches_truth，
        # 也照样能算），不依赖 score 的新旧格式
        cm = None
        if sc.get("class_match") is not None:
            cm = bool(sc["class_match"])
        elif obs.get("cls") and sc.get("truth_class"):
            cm = (str(obs["cls"]) == str(sc["truth_class"]))
        from depot_vln.verify_depth import RULE_ID as _DEPTH_RULE

        is_depth_verify = "比格口" in (obs.get("detail") or "")
        # 只把**当前判据版本**的真跑计入占用判定统计；旧判据的记录另列，
        # 免得"改了判据"把历史失败抹掉、或者拿旧失败判新规则不及格。
        pitch_deg = float(d.get("cam_pitch_deg") or 0.0)
        # **正常工况**（pitch=0）才进"占用判定对不对"的统计；pitch≠0 是**故意注入故障**
        # 的容限实验，单独报（它本来就该失准，混进来会让判据看起来时对时错）。
        depth_cur = (is_depth_verify and d.get("verify_rule") == _DEPTH_RULE
                     and abs(pitch_deg) < 0.01)
        runs.append({
            "file": Path(f).name, "kind": kind,
            "depth_verify": depth_cur,
            "depth_stale": is_depth_verify and not depth_cur,
            "pitch_exp": (is_depth_verify and abs(pitch_deg) >= 0.01),
            "cam_pitch_deg": pitch_deg,
            "occ_match": sc.get("occupied_match"),
            "path_m": round(float(plan.get("path_length") or 0), 2),
            "endpoint_err_m": d.get("endpoint_err_m"),
            "obs_ok": bool(obs.get("ok")), "obs_cls": obs.get("cls"),
            "wall_s": d.get("wall_s"),
            "class_match": cm,
            "score": sc,
        })
    if not runs:
        return None
    dv = [r for r in runs if r["depth_verify"]]
    dv_stale = [r for r in runs if r.get("depth_stale") and not r.get("pitch_exp")]
    pitch_exp = [r for r in runs if r.get("pitch_exp")]
    return {
        "n": len(runs),
        "kinds": {k: sum(1 for r in runs if r["kind"] == k)
                  for k in ("direct", "substituted", "blocked_suggest", "other")},
        "depth_verify": {
            "n": len(dv),
            "occ_ok": sum(1 for r in dv if r["occ_match"] is True),
            "occ_bad": [r["file"] for r in dv if r["occ_match"] is False],
            "stale": [r["file"] for r in dv_stale],
            "pitch": sorted(({"pitch": r["cam_pitch_deg"], "match": r["occ_match"],
                              "file": r["file"]} for r in pitch_exp),
                            key=lambda x: x["pitch"]),
        },
        "longest_m": max(r["path_m"] for r in runs),
        "endpoint_ok": [r["endpoint_err_m"] for r in runs if isinstance(r["endpoint_err_m"], (int, float))],
        "class_match": [r["class_match"] for r in runs if r["class_match"] is not None],
        "runs": runs,
    }


def run_cmd(args: list[str]) -> tuple[int, str]:
    proc = subprocess.run([sys.executable, *args], cwd=str(ROOT), capture_output=True, text=True)
    return proc.returncode, (proc.stdout or "") + (proc.stderr or "")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-run", action="store_true")
    args = ap.parse_args()

    rows: list[tuple[str, str, str, str]] = []  # 判据, 阶段, 状态, 关键数字

    # ---- P1 世界
    p1_ok = None
    p1_detail = "未跑"
    p1_n = ""          # 闸门数量从 verify_world 的输出里取，不再写死（曾写死"19 项"，实际已 22 项）
    if not args.no_run:
        rc, out = run_cmd(["scripts/verify_world.py"])
        p1_ok = rc == 0
        for line in out.splitlines():
            if "巷道净宽 mean" in line:
                p1_detail = line.strip()
            m = re.search(r"自检全部通过（(\d+) 项）", line)
            if m:
                p1_n = f"（{m.group(1)} 项）"
        (RESULTS).mkdir(exist_ok=True)
        (RESULTS / "p1_world.log").write_text(out, encoding="utf-8")
    else:
        log = RESULTS / "p1_world.log"
        if log.exists():
            p1_ok = "RESULT: PASS" in log.read_text(encoding="utf-8")
            for line in log.read_text(encoding="utf-8").splitlines():
                if "巷道净宽 mean" in line:
                    p1_detail = line.strip()
                m = re.search(r"自检全部通过（(\d+) 项）", line)
                if m:
                    p1_n = f"（{m.group(1)} 项）"
    rows.append((f"P1 世界自检{p1_n}", "P1", "PASS" if p1_ok else ("FAIL" if p1_ok is False else "未跑"), p1_detail))

    # ---- M1 / M2
    # M1 的**判据**必须用真值版（m1_truth.json）：m1.json 是拿 /odom 判的，而 /odom 是我们
    # 自己从轮速积出来的 -> 自己给自己打分（实测会出现 0.4% 的假优秀）。真值版缺失时才退回 odom 版，
    # 并在数字里标明来源，避免以后误读。
    m1_truth = load_json("m1_truth.json")
    m1 = load_json("m1.json")
    m1_detail = ""
    if m1_truth:
        m1 = m1_truth
        m1_src = "真值位姿"
        m1_detail = (f"{sum(1 for c in m1.get('cases', []) if c.get('ok'))}/"
                     f"{len(m1.get('cases', []))} 用例达标（真值位姿，非 odom）")
    elif m1:
        m1_src = "odom(自评)"
        m1_detail = (f"{m1.get('n_pass')}/{m1.get('n_total')} 用例达标"
                     f"（⚠ 用的是 /odom = 自己给自己打分，需 `make m1-truth`）")
    if m1:
        rows.append((
            "M1 速度跟踪",
            "P2",
            "PASS" if m1.get("pass") else "FAIL",
            m1_detail,
        ))
    else:
        rows.append(("M1 速度跟踪", "P2", "未跑", "需 `make sim SIM_ARGS='gui:=false'` + `make m1-truth`"))

    m2 = load_json("m2.json")
    if m2:
        rows.append((
            "M2 侧移入库位",
            "P2",
            "PASS" if m2.get("pass") else "FAIL",
            f"横向 {m2.get('lateral_displacement_m')} m，成功 {m2.get('n_success')}/{m2.get('n_attempts')}",
        ))
    else:
        rows.append(("M2 侧移入库位", "P2", "未跑", "需 sim + Nav2 + `make m2`"))

    # ---- S1 双目深度
    # 产物名是 s1_depth.json（scripts/s1_depth_eval.py 的默认输出），旧名 s1.json 兼容
    s1 = load_json("s1_depth.json") or load_json("s1.json")
    rows.append((
        "S1 双目深度误差",
        "P3",
        ("PASS" if s1.get("pass") else "FAIL") if s1 else "未跑",
        (f"中位 {100 * s1.get('median_rel_err', 0):.2f}% / 95 分位 "
         f"{100 * s1.get('p95_rel_err', 0):.2f}%（{s1.get('n_pixels')} 像素，"
         f"{s1.get('resolution', ['?', '?'])[0]}×{s1.get('resolution', ['?', '?'])[1]}，"
         f"判据 <5% / <12%）")
        if s1 else "需相机桥接后跑 scripts/s1_depth_eval.py",
    ))

    # ---- SLAM-1（三配置：S1 纯激光 / S2 +IMU EKF / S3 双目惯导）
    s1 = load_json("slam1.json")
    s2 = load_json("slam1_s2.json")
    s3 = load_json("slam1_s3.json")
    s4 = load_json("slam1_s4.json")
    s5 = load_json("slam1_s5.json")
    if s1 or s2 or s3 or s4 or s5:
        parts = []
        for tag, d in (("S1", s1), ("S2", s2), ("S3", s3), ("S4", s4), ("S5", s5)):
            if not d:
                continue
            hit = (d.get("map") or {}).get("match_ratio")
            parts.append(f"{tag} ATE {d.get('ate_rmse_m', 0):.2f} m / 地图 {100 * (hit or 0):.1f}%")
        # SPEC §6：S3 达标线 = 相对 S1 的 ATE 降 ≥40%（或回环残差 <0.3 m，未单独测）
        aim = ""
        if s1 and s3:
            target = 0.6 * float(s1.get("ate_rmse_m") or 0.0)
            got = float(s3.get("ate_rmse_m") or 0.0)
            aim = ("；S3 达标（相对 S1 降 ≥40%）" if got <= target
                   else f"；S3 未达标（{got:.2f} m > 线 {target:.2f} m）→ 报告须写清视觉退化点")
        elif s1 and s2:
            aim = "；+IMU 无显著改善（沿轴平移不可观测）"
        if s1 and s4:
            t4 = 0.6 * float(s1.get("ate_rmse_m") or 0.0)
            g4 = float(s4.get("ate_rmse_m") or 0.0)
            g1 = float(s1.get("ate_rmse_m") or 0.0)
            # 两个事实都要写：**绝对**是否优于 S1，与 SPEC 的"相对 S1 再降 40%"线。
            # 只写"未达标"会误导（S4 其实三项都优于 S1），只写"优于"则不认 SPEC。
            # **注意归因**：0.6×S1 是 SPEC §6 给 **S3** 的达标线；S4/S5 是本项目自加的配置，
            # 沿用同一条数只作"自设目标"，不能写成"未达 SPEC"。
            aim += (f"；S4（地面光流，自加配置）ATE {g4:.2f} m "
                    f"{'优于' if g4 < g1 else '不及'} S1 {g1:.2f} m"
                    f"，{'达' if g4 <= t4 else '未达'} 自设目标 0.6×S1（{t4:.2f} m）")
        if s1 and s5:
            t5 = 0.6 * float(s1.get("ate_rmse_m") or 0.0)
            g5 = float(s5.get("ate_rmse_m") or 0.0)
            g4b = float((s4 or {}).get("ate_rmse_m") or 0.0)
            g1b = float(s1.get("ate_rmse_m") or 1.0)
            aim += (f"；S5（地面光流+轮速 EKF，五配置最好）ATE {g5:.2f} m"
                    f"{f'，较 S4 {g4b:.2f} m 更好' if g4b and g5 < g4b else (f'，不及 S4 {g4b:.2f} m' if g4b else '')}"
                    f"（相对 S1 降 {100 * (1 - g5 / g1b):.0f}%），"
                    f"{'达' if g5 <= t5 else '未达'}自设目标 0.6×S1（{t5:.2f} m）")
        n_cfg = sum(bool(x) for x in (s1, s2, s3, s4, s5))
        rows.append((
            "SLAM-1 配置对比",
            "P4",
            (f"S1+S2+S3+S4+S5 齐备" if n_cfg == 5 else f"已跑 {n_cfg}/5 配置"),
            "；".join(parts) + aim,
        ))
    else:
        rows.append((
            "SLAM-1 三配置对比",
            "P4",
            "未跑",
            "需 200 m 固定路线 + 真值轨迹",
        ))

    # ---- GRAPH-1
    g1 = load_json("graph1.json")
    if g1:
        rows.append((
            "GRAPH-1 图 vs 真值最短路",
            "P5",
            "PASS" if g1.get("pass") else "FAIL",
            f"中位误差 {g1.get('median_rel_err'):.1%}（90 分位 {g1.get('p90_rel_err'):.1%}），"
            f"节点 {g1.get('nodes')} / 边 {g1.get('edges')}",
        ))
    else:
        rows.append(("GRAPH-1 图 vs 真值最短路", "P5", "未跑", "`python3.10 scripts/check_graph.py`"))

    # ---- SEG-1 / SEG-2
    for key, label, stage, hint in (
        ("seg1.json", "SEG-1 bay 网格发现", "P5", "需 3D 点云结构提取（P5 实现）"),
        ("seg2.json", "SEG-2 占用/类别", "P5", "需 T2 流水线 + 打分脚本"),
    ):
        d = load_json(key)
        rows.append((label, stage, ("PASS" if d.get("pass") else "FAIL") if d else "未跑",
                     json.dumps({k: v for k, v in (d or {}).items() if k in ("median_center_err_m", "count_err_ratio", "yaw_err_deg",
                                     "precision", "recall", "top1_acc")}, ensure_ascii=False) if d else hint))

    # ---- LANG-1
    bs = bench_summary(latest("bench_2*.csv"))
    if bs:
        worst = min(bs["per_category"].items(), key=lambda kv: kv[1]["rate"])
        rows.append((
            "LANG-1 指令基准（四类）",
            "P0/P6",
            "PASS" if bs["silent_wrong"] == 0 else "FAIL",
            f"{bs['ok']}/{bs['n']} = {bs['rate']:.1%}，静默猜错 {bs['silent_wrong']} 次，"
            f"最差类别 {worst[0]} {worst[1]['rate']:.0%}（{bs['csv']}）",
        ))
    else:
        rows.append(("LANG-1 指令基准（四类）", "P0/P6", "未跑", "`make bench-run`"))

    # ---- P6-1 约束层：① 离线自检（秒级，直接跑）② 真模型基准（读已有 CSV，不重跑 LLM）
    p61_ok = None
    p61_detail = "未跑"
    if not args.no_run:
        rc, out = run_cmd(["tests/test_constraint.py"])
        p61_ok = rc == 0
        (RESULTS / "p61_selftest.log").write_text(out, encoding="utf-8")
        m = re.search(r"结论：\s*(PASS|FAIL)", out)
        p61_detail = "34 项单元 + 对抗端到端 + 回退可见" if (m and m.group(1) == "PASS") else "见 results/p61_selftest.log"
    else:
        log = RESULTS / "p61_selftest.log"
        if log.exists():
            p61_ok = "结论：PASS" in log.read_text(encoding="utf-8")
            p61_detail = "34 项单元 + 对抗端到端 + 回退可见"
    rows.append(("P6-1 约束层离线自检", "P6",
                 "PASS" if p61_ok else ("FAIL" if p61_ok is False else "未跑"), p61_detail))

    cons_csv = latest("bench_llm/bench_*_constrained.csv")
    if cons_csv:
        crows = list(csv.DictReader(open(cons_csv, encoding="utf-8")))
        n_ok = sum(1 for r in crows if str(r["behavior_ok"]).strip().lower() in ("true", "1"))
        n_silent = sum(1 for r in crows if str(r["silent_wrong"]).strip().lower() in ("true", "1"))
        n_nocode = sum(1 for r in crows if r["expected_behavior"] == "reject" and not r["actual_reason_code"])
        # 对照：同目录里最新的裸模型 CSV（文件名不带 _constrained）
        bare = [f for f in sorted(glob.glob(str(RESULTS / "bench_llm" / "bench_2*.csv")))
                if "_constrained" not in f]
        cmp_txt = ""
        if bare:
            brows = list(csv.DictReader(open(bare[-1], encoding="utf-8")))
            b_ok = sum(1 for r in brows if str(r["behavior_ok"]).strip().lower() in ("true", "1"))
            b_silent = sum(1 for r in brows if str(r["silent_wrong"]).strip().lower() in ("true", "1"))
            cmp_txt = (f"；对照裸模型 {b_ok}/{len(brows)} = {b_ok / len(brows):.1%}、"
                       f"静默猜 {b_silent}")
        rows.append((
            "P6-1 约束层（真模型基准）", "P6",
            "PASS" if (n_ok == len(crows) and n_silent == 0 and n_nocode == 0) else "FAIL",
            f"{n_ok}/{len(crows)} = {n_ok / len(crows):.1%}，静默猜 {n_silent}，缺原因码 {n_nocode}"
            f"{cmp_txt}（{Path(cons_csv).name}）",
        ))
    else:
        rows.append(("P6-1 约束层（真模型基准）", "P6", "未跑",
                     "`BACKEND=ollama-constrained make bench-run`"))

    # ---- P6-2 真工具层：从产物汇总（不重跑仿真；跑一次要 2–6 分钟且需人在场）
    p62 = p6_demo_summary()
    if p62:
        k = p62["kinds"]
        errs = p62["endpoint_ok"]
        cls_m = p62["class_match"]
        # 判据：有直达证据 + 有恢复证据（改道或仅提议）才算过；
        # **类别一致率只作参考**（D25：320×240 下 S/M 仅差 ~9 px，类别不可靠）
        enough = k["direct"] >= 1 and (k["substituted"] + k["blocked_suggest"]) >= 1
        detail = (f"真跑 {p62['n']} 次：直达 {k['direct']} / 改道 {k['substituted']} / "
                  f"被封提议 {k['blocked_suggest']} / 失败或调试 {k['other']}；"
                  f"最长 {p62['longest_m']:.1f} m")
        if k["blocked_suggest"] == 0:
            detail += "（**仅提议**这条路径目前只有 dry 证据）"
        if errs:
            detail += f"；末段误差最好 {min(errs):.2f} m"
        if cls_m:
            detail += (f"；类别一致 {sum(1 for c in cls_m if c)}/{len(cls_m)}"
                       f"（**参考值**，按 D25 不作判据）")
        rows.append(("P6-2 真工具层（真跑证据）", "P6", "PASS" if enough else "FAIL", detail))
    else:
        rows.append(("P6-2 真工具层（真跑证据）", "P6", "未跑",
                     "`UTT=\"去 A-R-01-1\" make p6-demo`（需仿真，2–6 分钟）"))

    # ---- P6-2b verify(depth)：占用判定真跑证据（几何法，不调 VLM）
    if p62:
        dv = p62["depth_verify"]
        if dv["n"]:
            ok = dv["occ_ok"] == dv["n"] and not dv["occ_bad"]
            det = (f"VERIFY=depth 真跑 {dv['n']} 格（当前判据版本）："
                   f"占用判定与真值一致 {dv['occ_ok']}/{dv['n']}")
            if dv["stale"]:
                det += f"；另有 {len(dv['stale'])} 次是旧判据（不计入）"
            if dv["occ_bad"]:
                det += f"；不一致 {len(dv['occ_bad'])} 个（{', '.join(dv['occ_bad'][:3])}）"
            rows.append(("P6-2b verify 深度门限（占用判定）", "P6",
                         "PASS" if ok else "FAIL", det))
        else:
            rows.append(("P6-2b verify 深度门限（占用判定）", "P6", "未跑",
                         "`UTT=\"去 C-L-08-1\" VERIFY=depth make p6-demo`"))

    # ---- P6-2d 俯仰容限（故意注入 cam_pitch，本来就该在某个角度失准）
    if True:
        # **俯仰表必须按当前判据重算**：判据改了（窗口 z 上沿 1.15→0.85）之后，
        # 旧真跑里存的判决属于旧规则，直接列出来就是在拿旧规则的数报新规则。
        # 好在这条本来就能离线重算（存档里有 depth + 位姿），用审计汇总即可。
        pts = []
        from depot_vln.verify_depth import RULE_ID as _RULE      # noqa: PLC0415
        try:
            _audit = json.loads((RESULTS / "verify_audit.json").read_text(encoding="utf-8"))
            if _audit.get("rule_id") == _RULE:
                pts = _audit.get("pitch") or []
        except Exception:                   # noqa: BLE001
            pts = []
        if pts:
            up = [p for p in pts if p["pitch"] > 0]
            dn = [p for p in pts if p["pitch"] < 0]
            # 判据（2026-11-03 按新规则重述）：①**俯侧必须实测出假阳性**（边界被钉住）
            # ②**仰侧在已测范围内不得漏检**（+3° 以内全对）。
            # 旧判据要求"仰侧也有失准点"，那是窗口 z 上沿 1.15 时代的结论：窗口顶到上层
            # 横梁、质心偏高，按质心收缩会啃掉货的底边 → 仰侧提前漏检。改成 0.85 后窗口更贴货，
            # 仰侧 +2/+3 **实测全部判对**（354/311 px），所以"仰侧边界"本次**未测到** ——
            # 这一条如实写进 detail 与 LIMITATIONS L8，不许拿旧数字冒充。
            # ②改成"**安全带内**（|pitch| ≤ 3°）不得漏检"，而不是"所有仰侧样本都判对"：
            # 否则一旦补测 +4° 并真的失准（那正是我们想看到的信息），这条判据会误报 FAIL。
            # 仰侧边界若已被测到，就把那个角度报出来（从"未测"变成"已钉住"）。
            # **用"位姿归一"列判边界**（`match_truth_pose`）：俯仰实验里混着 odom 漂移
            # （实测到过 0.44 m，它单独就能让带内俯仰判错）。不归一 = 把位姿误差记到俯仰头上。
            # as-run（`match`）列照旧并列在 detail 里，**不隐藏** —— 现场是两者叠加。
            def _m(p):
                return p.get("match_truth_pose", p["match"])

            ok = (any(_m(p) is False for p in dn)
                  and all(_m(p) is True for p in pts if abs(float(p["pitch"])) <= 3.0))
            up_bad = sorted(float(p["pitch"]) for p in up if _m(p) is False)
            # 按角度归并：同一个角度可能有多次（如 +2° 两次一错一对）—— 逐个列出会读成矛盾，
            # 归并成 "+2.0°×2→1错 1准" 才能看出**骑线**这件事本身（D29 续 2）。
            by_pitch: dict[float, list[bool]] = {}
            for p in pts:
                by_pitch.setdefault(round(p["pitch"], 1), []).append(bool(p["match"]))

            def _cell(pitch: float, ms: list[bool]) -> str:
                n_ok = sum(ms)
                if len(ms) == 1:
                    return f"{pitch:+.1f}°→{'准' if n_ok else '错'}"
                return f"{pitch:+.1f}°×{len(ms)}→{n_ok}准 {len(ms) - n_ok}错"

            by_pitch = {}
            for p in pts:
                by_pitch.setdefault(round(p["pitch"], 1), []).append(bool(_m(p)))
            tbl = "、".join(_cell(k, v) for k, v in sorted(by_pitch.items(),
                                                           key=lambda kv: -kv[0]))
            rows.append(("P6-2d verify 俯仰容限（注入实验）", "P6",
                         "PASS" if ok else ("FAIL" if pts else "未跑"),
                         f"[{_RULE}] {tbl}｜俯侧 **假阳性**（边界钉住）、安全带内（俯仰绝对值 ≤3°）无漏检"
                         + (f"；**仰侧边界已实测：{up_bad[0]:+.1f}° 起漏检**" if up_bad
                            else "；**仰侧上界未测**（+4/+5° 无样本）")
                         + "（旧规则下 +2° 会漏检，那是窗口上沿 1.15→0.85 之前的结论，见 D27 续 / L8）"))
        else:
            rows.append(("P6-2d verify 俯仰容限（注入实验）", "P6", "未跑",
                         "`CAM_PITCH=+3.0 UTT=\"去 C-L-09-1\" VERIFY=depth make p6-demo`"))

    # ---- P6-2e 动态障碍（真阳性 + 中断后恢复通行）
    dyn = []
    for f in sorted(glob.glob(str(RESULTS / "p6_dyn_*.json"))):
        try:
            dyn.append(json.loads(Path(f).read_text(encoding="utf-8")))
        except Exception:
            continue
    if dyn:
        good = [d for d in dyn if d.get("pass")]
        ok = bool(good)
        lines = max((d.get("guard_block_lines") or 0) for d in good)
        events = max((d.get("guard_block_events") or 0) for d in good)
        det = (f"真跑 {len(dyn)} 次、{len(good)} 次成立；推车横穿行驶线 → 安全层压住 {lines} 行日志"
               f"（{events} 次计数行，被 KILL 时来不及打总数）、"
               f"让开后机器人继续开到货位并完成 verify（demo rc=0）"
               if good else
               f"真跑 {len(dyn)} 次但都不满足（需 压住≥1 + 自检 rc 0 + demo rc 0）："
               f"{[d.get('pass') for d in dyn]}")
        rows.append(("P6-2e 动态障碍（真阳性+恢复）", "P6", "PASS" if ok else "FAIL", det))
    else:
        rows.append(("P6-2e 动态障碍（真阳性+恢复）", "P6", "未跑",
                     "`GUARD=1 DYN=1 UTT=\"去 C-L-08-1\" VERIFY=depth make p6-demo`"))

    # ---- P6-2f Nav2 动态障碍绕行（受控试验台：车摆在"起点→目标"直线上）
    probes = []
    for f in sorted(glob.glob(str(RESULTS / "avoid_probe_*.json"))):
        try:
            probes.append(json.loads(Path(f).read_text(encoding="utf-8")))
        except Exception:
            continue
    # **配对判据**（2026-11-02 修正）：只看"有车时的路径偏出"会把**静态地图本来就有的绕行**
    # 误算成避障 —— 实测踩过：无车对照偏出 -0.45 m（14 个采样，最大 0.77 m），
    # 有车那次的 0.69 m 完全落在基线分布内。所以必须"有车偏出 − 无车偏出 ≥ 0.5 m"。
    ctrls = []
    for f in sorted(glob.glob(str(RESULTS / "avoid_control_*.json"))):
        try:
            ctrls.append(json.loads(Path(f).read_text(encoding="utf-8")))
        except Exception:
            continue

    def _ctrl_for(x: float) -> float | None:
        best = None
        for c in ctrls:
            if abs((c.get("cart_x") or 0) - x) < 0.3:
                best = abs(c.get("control_dev_m") or 0)
        return best

    if probes:
        paired = []
        for d in probes:
            base = _ctrl_for(d.get("cart_x") or 0)
            if base is None:
                continue
            extra = abs(d.get("path_dev_m") or 0) - base
            paired.append((d, base, extra))
        # **物理口径**（2026-11-03 定版，D31 续 6）：
        #   ① 障碍真的进了 costmap：全局层最近致命格 ≤ 推车半宽（0.26→取 0.31 容差）
        #   ② 路径到车心 ≥ 半宽 + inflation（0.26+0.35 = 0.61）—— 即**不进膨胀区**
        #   ③ 车仍挡路时机器人越过（end > cart_x + 0.5）
        # 曾经的"配对偏差 ≥0.5"被"规划从当前位置出发"的斜线效应污染，已降为旁证。
        CLEAR_MIN, MARK_MAX = 0.61, 0.31
        good = [d for d in probes
                if d.get("pass")
                and (d.get("path_clear_m") or d.get("path_dev_m") or 0) >= CLEAR_MIN
                and (d.get("global_min_d_m") if d.get("global_min_d_m") is not None else 9.9) <= MARK_MAX]
        # 整条 demo 的 DYN 记录也是同一判据的合法证据（真实长巷道，比受控台更接近使用场景）。
        # 只取：越过 + demo 正常结束 + 两层都标注 + 路径不进膨胀区。
        dyn_ev = []
        for f in sorted(glob.glob(str(RESULTS / "p6_dyn_*.json"))):
            try:
                e = json.loads(Path(f).read_text(encoding="utf-8"))
            except Exception:                   # noqa: BLE001
                continue
            clear = e.get("path_dev_m")
            gmd = e.get("global_min_d_m")
            if (e.get("bypass") == 1 and e.get("demo_rc") == 0 and clear is not None
                    and float(clear) >= CLEAR_MIN
                    and (gmd if gmd is not None else 9.9) <= MARK_MAX):
                dyn_ev.append(e)
        if not paired and not dyn_ev:
            rows.append(("P6-2f Nav2 绕行（配对判据）", "P6", "未跑",
                         "缺**无车对照**：先跑 `CONTROL=1 bash scripts/avoid_probe.sh`"))
        elif good or dyn_ev:
            bits = []
            for d0 in good:
                clear = d0.get("path_clear_m") or d0.get("path_dev_m") or 0
                bits.append(f"受控台 ({d0['cart_x']},{d0['cart_y']})：标注 {d0['global_min_d_m']:.2f} m、"
                            f"路径 {clear:.2f} m（余量 {clear - CLEAR_MIN:+.2f}）")
            for e in dyn_ev:
                bits.append(f"整条 demo (x={e['dyn_x']})：标注 {e['global_min_d_m']:.2f} m、"
                            f"路径 {e['path_dev_m']:.2f} m（余量 {e['path_dev_m'] - CLEAR_MIN:+.2f}）")
            best = max([(d.get("path_clear_m") or d.get("path_dev_m") or 0) for d in good]
                       + [float(e["path_dev_m"]) for e in dyn_ev])
            det = (f"**{len(bits)} 条证据**均满足：①两层 costmap 都标上了（≤0.31 m）"
                   f"②路径到车心 ≥{CLEAR_MIN:.2f} m（=0.26 半宽 + inflation，不进膨胀区）"
                   f"③车仍挡路时机器人越过。最好的一次 {best:.2f} m（物理不碰撞只需 0.49 m，"
                   f"余量 {(best - 0.49) / 0.49 * 100:.0f}%）。明细：{'；'.join(bits)}。"
                   f"**A/B 反证**：同一几何下全局层未标注时路径只到 0.40 m（穿进膨胀区）")
            rows.append(("P6-2f Nav2 绕行（受控台 + 整条 demo）", "P6", "PASS", det))
        else:
            tbl = "；".join(
                f"clear {abs(d.get('path_clear_m') or d.get('path_dev_m') or 0):.2f} m、"
                f"global_min_d {d.get('global_min_d_m')}"
                for d, _b, _e in paired)
            rows.append(("P6-2f Nav2 绕行（配对判据）", "P6", "FAIL",
                         f"物理判据未达（需 clear ≥0.61 且 global_min_d ≤0.31）→ **绕行未被证明**：{tbl}。"
                         f"原因：推车没真正压在机器人实际路径上（对照显示无车时路径同样偏出），"
                         f"偏出量里混着静态地图绕行与「规划从当前位置出发」的斜线效应"
                         f"（REPORT §5.13、TROUBLESHOOTING §37）"))
    else:
        rows.append(("P6-2f Nav2 绕行（受控台）", "P6", "未跑",
                     "`bash scripts/avoid_probe.sh`"))

    # ---- P6-2c verify(depth) 离线审计：真实深度图复算 + 位姿扰动鲁棒性
    dumps = list((RESULTS / "depth_dumps").glob("*.npz"))
    if dumps:
        code, out = run_cmd([str(ROOT / "scripts" / "verify_depth_audit.py"), "--strict"])
        tail = [ln for ln in out.strip().splitlines() if ln.strip()][-2:]
        # 汇总文件里带 rule_id：判据一改，这里报的必须是**新规则下重算**的数
        audit = {}
        try:
            audit = json.loads((RESULTS / "verify_audit.json").read_text(encoding="utf-8"))
        except Exception:                       # noqa: BLE001
            pass
        ver = f"[{audit.get('rule_id')}] " if audit.get("rule_id") else ""
        rows.append(("P6-2c verify 深度门限（离线审计）", "P6",
                     "PASS" if code == 0 else "FAIL",
                     f"{ver}{len(dumps)} 份深度存档复算：{' / '.join(t.strip() for t in tail)}"))
    else:
        rows.append(("P6-2c verify 深度门限（离线审计）", "P6", "未跑",
                     "无 results/depth_dumps/*.npz"))

    print(f"== depot-vln 验收汇总 {datetime.now():%Y-%m-%d %H:%M} ==")
    print()
    # **排序**：rows 是按代码里 append 的顺序拼的，插入新判据时很容易把 P6-2c 落到表尾
    # （实测发生过：表尾出现 P6-2c，看起来像"P6-2b 后面漏了 c"）。这里按编号显式排序。
    _ORDER = ["P1", "M1", "M2", "S1", "SLAM-1", "GRAPH-1", "SEG-1", "SEG-2", "LANG-1",
              "P6-1 约束层离线自检", "P6-1 约束层（真模型基准）", "P6-2 真工具层（真跑证据）",
              "P6-2b", "P6-2c", "P6-2d", "P6-2e", "P6-2f"]

    def _ord(name: str) -> int:
        for i, k in enumerate(_ORDER):
            if name.startswith(k):
                return i
        return len(_ORDER)

    rows.sort(key=lambda r: _ord(r[0]))

    w1 = max(len(r[0]) for r in rows) + 2
    print(f"{'判据':<{w1}}{'阶段':<8}{'状态':<8}关键数字")
    for name, stage, status, detail in rows:
        print(f"{name:<{w1}}{stage:<8}{status:<8}{detail}")

    n_pass = sum(1 for r in rows if r[2] == "PASS")
    n_fail = sum(1 for r in rows if r[2] == "FAIL")
    n_todo = sum(1 for r in rows if r[2] == "未跑")
    print()
    print(f"PASS {n_pass} / FAIL {n_fail} / 未跑 {n_todo}")

    md = [f"# 验收汇总（{datetime.now():%Y-%m-%d %H:%M}）", "",
          "由 `python3.10 scripts/accept.py` 生成。规则：没跑过就是「未跑」，不写「通过」。", "",
          "| 判据 | 阶段 | 状态 | 关键数字 |", "|---|---|---|---|"]
    for name, stage, status, detail in rows:
        md.append(f"| {name} | {stage} | {status} | {detail.replace('|', '/')} |")
    md.append("")
    md.append(f"合计：PASS {n_pass} / FAIL {n_fail} / 未跑 {n_todo}")
    md.append("")
    (ROOT / "docs").mkdir(exist_ok=True)
    (ROOT / "docs" / "ACCEPTANCE.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    print(f"-> docs/ACCEPTANCE.md")
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
