#!/usr/bin/env python3
"""生成整个世界：world/warehouse.sdf + slot_grid.json + ground_truth/*。

用法
  python3 scripts/build_world.py [--config config/world.yaml] [--verify]
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from depot_vln.layout import ROOT as _ROOT  # noqa: E402
from depot_vln.layout import compute_layout, load_config  # noqa: E402
from depot_vln.sdf import build_sdf  # noqa: E402
from depot_vln.truth import build_truth  # noqa: E402


def write_json(path: Path, obj) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(obj, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(ROOT / "config" / "world.yaml"))
    ap.add_argument("--out", default=str(ROOT / "world"))
    ap.add_argument("--gt", default=str(ROOT / "ground_truth"))
    ap.add_argument("--verify", action="store_true", help="生成后立刻跑自检")
    args = ap.parse_args()

    cfg = load_config(args.config)
    layout = compute_layout(cfg)
    truth = build_truth(cfg, layout)

    out = Path(args.out)
    gt = Path(args.gt)
    out.mkdir(parents=True, exist_ok=True)
    gt.mkdir(parents=True, exist_ok=True)

    # 1) 世界本体
    sdf = build_sdf(cfg, layout, truth)
    (out / "warehouse.sdf").write_text(sdf, encoding="utf-8")

    # 2) 货位几何（运行时唯一允许读的几何来源）
    slot_grid = {
        "schema": "depot-vln/slot_grid@1",
        "config": str(Path(args.config).name),
        "seed": layout["seed"],
        "footprint": layout["footprint"],
        "derived": layout["derived"],
        "aisles": layout["aisles"],
        "blocks": layout["blocks"],
        "bays": layout["bays"],
        "slots": {s["address"]: s for s in layout["slots"]},
        "address_format": "<Aisle>-<Side>-<Bay:02d>-<Level>",
        "side_convention": "面朝巷道前进方向(+X)：L=+Y，R=-Y",
    }
    write_json(out / "slot_grid.json", slot_grid)
    write_json(out / "layout.json", layout)

    # 3) 地面真值（只用于打分）
    write_json(gt / "slots.json", {
        "schema": "depot-vln/ground_truth_slots@1",
        "seed": truth["seed"],
        "stats": truth["stats"],
        "slots": truth["slots"],
    })
    write_json(gt / "aisles.json", {
        "schema": "depot-vln/ground_truth_aisles@1",
        "blocked": truth["blocked"],
        "rack_faces": truth["rack_faces"],
        "carton_totals": truth["carton_totals"],
        "bays": truth["bays"],
    })

    # 4) 真值轨迹模板（P4 由 scripts/record_gt_poses.py 填数）
    poses_csv = gt / "poses.csv"
    with open(poses_csv, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "x", "y", "z", "qx", "qy", "qz", "qw", "vx", "vy", "wz"])
    (gt / "poses_README.md").write_text(
        "# ground_truth/poses.csv\n\n"
        "真值轨迹。由 `python3 scripts/record_gt_poses.py`（订阅 /world/depot/pose/info 里的\n"
        "depot_bot 位姿）在仿真运行时写入，20 Hz。\n\n"
        "列：t(秒, sim time) x y z qx qy qz qw vx vy wz\n"
        "本文件只用于 ATE/RPE 打分，**运行时禁止被 agent 读取**。\n",
        encoding="utf-8",
    )

    d = layout["derived"]
    print(f"[build_world] config      {cfg['_path']}")
    print(f"[build_world] footprint   {d['footprint_x']:.2f} x {d['footprint_y']:.2f} m")
    print(f"[build_world] aisles      {d['n_aisles']}  净宽 {cfg['racks']['aisle_clear_width']:.2f} m")
    print(f"[build_world] bays        {d['n_bays']}  slots {d['n_slots']}")
    print(f"[build_world] occupied    {truth['stats']['occupied']}  ({truth['stats']['occupancy_rate']:.1%})")
    print(f"[build_world] sdf         {out / 'warehouse.sdf'}  ({len(sdf) / 1024:.0f} KiB)")
    print(f"[build_world] slot_grid   {out / 'slot_grid.json'}")
    print(f"[build_world] truth       {gt / 'slots.json'}")

    if args.verify:
        import subprocess

        rc = subprocess.call([sys.executable, str(ROOT / "scripts" / "verify_world.py")])
        return rc
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
