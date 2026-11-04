#!/usr/bin/env python3
"""世界二分：把世界按子系统逐步缩小，实测哪一项让 Sensors 渲染线程起不来。

每组都走"改配置 -> 生成到 /tmp -> 起 server 跑固定时长 -> 看有没有步进"，不靠猜。
每个变体限时 60 s，跑完就杀，不碰你正在跑的仿真（独立 transport partition）。

  python3.10 tests/bisect_world.py
  python3.10 tests/bisect_world.py --only bays4
"""
from __future__ import annotations

import argparse
import copy
import json
import os
import re
import subprocess
import sys
import tempfile
import time
import uuid
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
PY = sys.executable

# name, cfg 覆盖, 说明
VARIANTS: list[tuple[str, dict, str]] = [
    ("full", {}, "当前世界（基准）"),
    ("no_shadows", {"shadows": False}, "关阴影"),
    ("low_detail", {"visual_detail": "low"}, "视觉件降到最低（只留碰撞体外形）"),
    ("no_cargo", {"semantics.occupancy_prob": 0.0}, "没有货物"),
    ("no_markings", {"markings.bay_barcode": False}, "没有地面条纹标线"),
    ("med", {"visual_detail": "medium"}, "medium（当前默认）"),
    ("med_noshadow", {"visual_detail": "medium", "shadows": False}, "medium + 关阴影"),
    ("med_nolamp", {"visual_detail": "medium", "shadows": False,
                    "lights.aisle_lamp.per_aisle": 0}, "medium + 关阴影 + 无 spot 灯"),
    ("med_low_cargo", {"visual_detail": "low", "shadows": False}, "low + 关阴影（最快档）"),
]


def apply_override(cfg: dict, dotted: str, value) -> None:
    keys = dotted.split(".")
    node = cfg
    for k in keys[:-1]:
        node = node[k]
    node[keys[-1]] = value


def build(cfg: dict, out_dir: Path) -> Path:
    cfg_path = out_dir / "world.yaml"
    cfg_path.write_text(yaml.safe_dump(cfg, allow_unicode=True), encoding="utf-8")
    sdf = out_dir / "warehouse.sdf"
    r = subprocess.run(
        [PY, str(ROOT / "scripts" / "build_world.py"), "--config", str(cfg_path),
         "--out", str(out_dir), "--gt", str(out_dir / "gt")],
        capture_output=True, text=True, cwd=str(ROOT),
    )
    if r.returncode != 0 or not sdf.exists():
        raise RuntimeError((r.stdout or "") + (r.stderr or ""))
    return sdf


def probe(sdf: Path, limit: float, shadows_ok: bool = True) -> dict:
    part = "depot_bisect_" + uuid.uuid4().hex[:8]
    env = dict(os.environ)
    env["IGN_PARTITION"] = part
    log = Path(tempfile.mkdtemp(prefix="bisect_")) / "server.log"
    fh = open(log, "w", encoding="utf-8")
    t0 = time.time()
    proc = subprocess.Popen(["ign", "gazebo", "-s", "-r", "-v", "3", str(sdf)],
                            stdout=fh, stderr=subprocess.STDOUT, text=True, env=env)
    t_render = t_stats = None
    rtf = iters = None
    try:
        while time.time() - t0 < limit:
            time.sleep(2.0)
            txt = log.read_text(encoding="utf-8", errors="ignore") if log.exists() else ""
            if t_render is None and "Rendering Thread initialized" in txt:
                t_render = round(time.time() - t0, 1)
            try:
                r = subprocess.run(["ign", "topic", "-e", "-t", "/world/depot/stats", "-n", "1"],
                                   capture_output=True, text=True, timeout=3, env=env)
                if r.stdout.strip():
                    if t_stats is None:
                        t_stats = round(time.time() - t0, 1)
                    m = re.search(r"real_time_factor:\s*([\d.eE+-]+)", r.stdout)
                    n = re.search(r"iterations:\s*(\d+)", r.stdout)
                    rtf = float(m.group(1)) if m else rtf
                    iters = int(n.group(1)) if n else iters
                    if iters and iters > 200:
                        break
            except subprocess.TimeoutExpired:
                pass
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            proc.kill()
        fh.close()
    txt = sdf.read_text(encoding="utf-8", errors="ignore")
    return {
        "render_s": t_render,
        "stats_s": t_stats,
        "rtf": rtf,
        "iters": iters,
        "visuals": len(re.findall(r"<visual", txt)),
        "collisions": len(re.findall(r"<collision", txt)),
        "lights": len(re.findall(r"<light ", txt)),
        "ok": t_stats is not None,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--limit", type=float, default=60.0)
    ap.add_argument("--only", default=None)
    args = ap.parse_args()

    base = yaml.safe_load((ROOT / "config" / "world.yaml").read_text(encoding="utf-8"))
    rows = []
    for name, override, note in VARIANTS:
        if args.only and args.only != name:
            continue
        cfg = copy.deepcopy(base)
        for k, v in override.items():
            apply_override(cfg, k, v)
        tmp = Path(tempfile.mkdtemp(prefix=f"bisect_{name}_"))
        print(f"[bisect] {name:<12} {note} ...", flush=True)
        try:
            sdf = build(cfg, tmp)
        except Exception as e:
            print(f"          生成失败：{str(e)[:200]}")
            rows.append({"name": name, "note": note, "build_fail": True})
            continue
        r = probe(sdf, args.limit)
        r.update({"name": name, "note": note})
        rows.append(r)
        print(f"          visual={r['visuals']:<5} light={r['lights']:<3} "
              f"渲染就绪={r['render_s']} 步进={r['stats_s']} RTF={r['rtf']} "
              f"-> {'OK' if r['ok'] else 'HANG'}", flush=True)

    print()
    print(f"{'variant':<12}{'visual':>7}{'light':>6}{'渲染(s)':>9}{'步进(s)':>9}{'RTF':>7}  结论")
    for r in rows:
        if r.get("build_fail"):
            print(f"{r['name']:<12}{'--':>7}{'--':>6}{'--':>9}{'--':>9}{'--':>7}  生成失败")
            continue
        print(f"{r['name']:<12}{r['visuals']:>7}{r['lights']:>6}{str(r['render_s']):>9}"
              f"{str(r['stats_s']):>9}{str(r['rtf']):>7}  {'OK' if r['ok'] else 'HANG'}")
    ok = [r["name"] for r in rows if r.get("ok")]
    print()
    print("能起来的变体：", ", ".join(ok) if ok else "（全部 HANG）")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
