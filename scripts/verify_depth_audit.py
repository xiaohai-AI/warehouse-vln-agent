#!/usr/bin/env python3
"""verify(depth) 离线审计：用 `results/depth_dumps/*.npz` 的真实深度图复算判决。

**为什么要这个工具**
  调阈值如果靠"改代码 → 开仿真 3 分钟 → 看一个数字"，一轮一轮烧的是人的时间。
  把左右图与深度落盘之后，判据与鲁棒性都能**离线**复算：
    · 基准判决对不对（对照 `ground_truth/slots.json`）—— **这条是判据**
    · 抗位姿误差的**包络**（把四边形整体平移，等效位姿/外参偏差）—— 这条是**报告项**

**为什么扰动只作报告、不作判据**（实测定量）
  横向平移对判决**完全没有影响**（±14 px 下块大小一模一样：窗口比货宽）。
  真正敏感的是**竖直**方向：窗口上下偏 8 px（≈2.9° 俯仰 / 0.07 m 高度）会让 `carton_s`
  的块从 554 掉到 187 px（低于 250 px 门槛）→ 翻成"空位"。
  这个轴向 odom 位姿**不提供**（odom 没有 z / 俯仰），所以本轮没触发；
  但它是真实薄弱点，必须写成**包络**，不能假装免疫。
  → 基准判决进判据；包络只报数（`--strict` 仅在基准判错时非零退出）。

口径说明（与 `score()` 同类）：本文件是**打分函数**，允许读 `ground_truth/`；
运行时链路（agent 的 verify 节点）**不读**真值，只用 `world/slot_grid.json` + 现场观测。

用法
  python3 scripts/verify_depth_audit.py            # 全部存档
  python3 scripts/verify_depth_audit.py --strict   # 基准判决有错就退出码 1
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402

from depot_vln.verify_depth import (LEVEL1_Z, RULE_ID,  # noqa: E402
                                    PROT_MIN_PX, PROT_NEAR, WIN_HALF_W,
                                    cam_world, depth_gate_verdict, slot_opening)


def shifted(depth: np.ndarray, du: int, dv: int) -> np.ndarray:
    out = np.full_like(depth, np.nan)
    h, w = depth.shape
    src = depth[max(0, dv):h + min(0, dv), max(0, du):w + min(0, du)]
    out[max(0, -dv):max(0, -dv) + src.shape[0], max(0, -du):max(0, -du) + src.shape[1]] = src
    return out


def stable_to(depth: np.ndarray, op: dict, verdict: str, axis: str,
              mags=(2, 4, 6, 8, 11, 14, 18, 22, 28, 35)) -> int:
    """判决在多大平移内保持不变（px）。axis='h' 只动 u，axis='v' 只动 v。"""
    best = 0
    for m in mags:
        offsets = ((-m, 0), (m, 0)) if axis == "h" else ((0, -m), (0, m))
        if all(depth_gate_verdict(shifted(depth, du, dv), op)["verdict"] == verdict
               for du, dv in offsets):
            best = m
        else:
            break
    return best


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dumps", default=str(ROOT / "results" / "depth_dumps"))
    ap.add_argument("--truth", default=str(ROOT / "ground_truth" / "slots.json"))
    ap.add_argument("--strict", action="store_true")
    ap.add_argument("--out", default=str(ROOT / "results" / "verify_audit.json"),
                    help="机器可读汇总（accept.py 的 P6-2c/P6-2d 读它）")
    args = ap.parse_args()

    dumps = sorted(Path(args.dumps).glob("*.npz"))
    truth = json.loads(Path(args.truth).read_text(encoding="utf-8"))["slots"]
    if not dumps:
        print("没有存档（先跑 VERIFY=depth 的 p6-demo）")
        return 2

    print(f"存档 {len(dumps)} 份；判据：窗口 ±{WIN_HALF_W} m，比格口近 {PROT_NEAR} m 的"
          f"最大连通块 ≥{PROT_MIN_PX} px\n")
    bad = 0
    emp: list[int] = []
    occ: list[int] = []
    inj: list[tuple[float, str, str, int]] = []      # (pitch, 真值, 判决, 块)
    inj_rows: list[dict] = []                        # 俯仰表的机器可读版（带 addr）
    base_rows: list[dict] = []
    for p in dumps:
        z = np.load(p, allow_pickle=False)
        meta = json.loads(p.with_suffix(".json").read_text(encoding="utf-8"))
        addr = meta["addr"]
        if addr not in truth:
            print(f"[skip] {addr} 不在真值表")
            continue
        robot = {"x": meta["pose"]["x"], "y": meta["pose"]["y"], "yaw": meta["pose"]["yaw"]}
        cam = cam_world(robot, "L" if meta["slot"]["side"] == "L" else "R")
        op = slot_opening(meta["slot"], cam)
        if op is None:
            print(f"[skip] {addr} 投影失败")
            continue
        res = depth_gate_verdict(z["depth"], op)
        want = "occupied" if truth[addr]["occupied"] else "empty"
        ok = res["verdict"] == want
        src = meta.get("pose_source", "truth")
        err = meta.get("pose_err") or {}
        errs = (f"位姿误差 {err['d_m']:.2f} m/{err['dyaw_deg']:+.1f}°" if err else "位姿=真值")
        pitch = float(meta.get("cam_pitch_deg") or 0.0)
        # 故意注入俯仰的实验**不进基准判据**（它本来就该在某个角度失准），
        # 单独列成容限表；否则一条设计好的故障会让"判据"看起来时对时错。
        if abs(pitch) >= 0.01:
            inj.append((pitch, want, res["verdict"], res["blob"]))
            # **必须同时给"位姿归一"列**：俯仰实验里混着 odom 漂移（实测到过 0.44 m），
            # 不归一就会把位姿误差的锅记在俯仰头上（2026-11-03 就发生过：-3° 空位 994 px 假阳性，
            # 用真值位姿复算只有 1 px）。判"俯仰容限"用归一列，as-run 列如实并列。
            v_norm, b_norm = res["verdict"], res["blob"]
            p_t = meta.get("pose_truth")
            if p_t:
                op_t = slot_opening(meta["slot"], cam_world(p_t, meta["slot"]["side"]))
                if op_t is not None:
                    r2 = depth_gate_verdict(z["depth"], op_t)
                    v_norm, b_norm = r2["verdict"], r2["blob"]
            inj_rows.append({"addr": addr, "pitch": pitch, "truth": want,
                             "verdict": res["verdict"], "blob": res["blob"],
                             "match": res["verdict"] == want,
                             "verdict_truth_pose": v_norm, "blob_truth_pose": b_norm,
                             "match_truth_pose": v_norm == want,
                             "pose_err_m": (meta.get("pose_err") or {}).get("d_m")})
            print(f"[inject] {addr:<10} cam_pitch={pitch:+.1f}° 真值 {want:<8} → 判 "
                  f"{res['verdict']:<8} 块 {res['blob']:5d} px（预期：超出容限即失准）")
            continue
        bad += 0 if ok else 1
        (occ if want == "occupied" else emp).append(res["blob"])
        base_rows.append({"dump": p.name, "addr": addr, "truth": want,
                          "verdict": res["verdict"], "blob": res["blob"], "match": ok})
        print(f"[{'PASS' if ok else 'FAIL'}] {addr:<10} 真值 {want:<8} → 判 {res['verdict']:<8} "
              f"块 {res['blob']:5d} px（共 {res['near_px']} px）| {src:<5} {errs} | "
              f"包络 横±{stable_to(z['depth'], op, res['verdict'], 'h')} px / "
              f"竖±{stable_to(z['depth'], op, res['verdict'], 'v')} px")

    n = len(emp) + len(occ)
    print()
    if emp and occ:
        print(f"分离度：空位块 {min(emp)}~{max(emp)} px  vs  有货 {min(occ)}~{max(occ)} px"
              f"（门槛 {PROT_MIN_PX} px）")
    print(f"结论：{n - bad}/{n} 判对（基准判据，正常工况）；扰动包络见上 —— "
          f"横向近乎无关，**竖直是薄弱轴**")
    if inj:
        inj.sort()
        print("俯仰注入实验（**不计入基准判据**）："
              + "、".join(f"{p:+.1f}°→{v}({b} px)" for p, _, v, b in inj))
    # 落一份机器可读汇总：**判据改了以后，账本必须重算**，不能继续用旧规则下的判决。
    # accept.py 的 P6-2c（基准）与 P6-2d（俯仰容限）都以这份的 rule_id 为准。
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({
        "rule_id": RULE_ID, "z_band": list(LEVEL1_Z),
        "n_dumps": len(dumps), "base_ok": n - bad, "base_n": n,
        "empty_blobs": emp, "occupied_blobs": occ,
        "base": base_rows, "pitch": inj_rows,
    }, ensure_ascii=False, indent=2), encoding="utf-8")
    try:
        print(f"汇总 → results/{out.name}")     # 打相对路径：绝对路径会漏进生成的 ACCEPTANCE
    except Exception:                          # noqa: BLE001
        print("汇总 → results/verify_audit.json")
    if args.strict and bad:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
