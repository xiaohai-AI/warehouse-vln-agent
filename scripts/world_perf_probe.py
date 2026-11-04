#!/usr/bin/env python3
"""量化一个 SDF 世界从启动到"真正开始在步进"要多久（不依赖 ROS）。

判据三条：
  1) 多久出现 "Rendering Thread initialized"
  2) 多久 /world/<name>/stats 开始有数据（= 仿真在步进）
  3) 稳定后的 RTF

  python3.10 scripts/world_perf_probe.py                      # 测 world/warehouse.sdf
  python3.10 scripts/world_perf_probe.py --sdf /tmp/light.sdf --limit 120
"""
from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import time
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def read(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="ignore")
    except Exception:
        return ""


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sdf", default=str(ROOT / "world" / "warehouse.sdf"))
    ap.add_argument("--limit", type=float, default=90.0, help="最多等多少秒")
    ap.add_argument("--world", default="depot")
    args = ap.parse_args()

    sdf = Path(args.sdf)
    if not sdf.exists():
        print(f"找不到 {sdf}")
        return 2

    part = "depot_perf_" + uuid.uuid4().hex[:8]
    env = dict(os.environ)
    env["IGN_PARTITION"] = part
    out = Path("/tmp") / f"perf_{part}.log"
    fh = open(out, "w", encoding="utf-8")
    t0 = time.time()
    proc = subprocess.Popen(
        ["ign", "gazebo", "-s", "-r", "-v", "3", str(sdf)],
        stdout=fh, stderr=subprocess.STDOUT, text=True, env=env,
    )
    print(f"[perf] 起 server（partition={part}，日志 {out}）")

    t_render = None
    t_stats = None
    rtf = None
    iters = None
    try:
        while time.time() - t0 < args.limit:
            time.sleep(2.0)
            txt = read(out)
            if t_render is None and "Rendering Thread initialized" in txt:
                t_render = time.time() - t0
            try:
                r = subprocess.run(
                    ["ign", "topic", "-e", "-t", f"/world/{args.world}/stats", "-n", "1"],
                    capture_output=True, text=True, timeout=3, env=env,
                )
                if r.stdout.strip():
                    if t_stats is None:
                        t_stats = time.time() - t0
                    m = re.search(r"real_time_factor:\s*([\d.eE+-]+)", r.stdout)
                    n = re.search(r"iterations:\s*(\d+)", r.stdout)
                    if m:
                        rtf = float(m.group(1))
                    if n:
                        iters = int(n.group(1))
                    if t_stats is not None and t_render is not None and iters and iters > 100:
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

    txt = read(out)
    n_vis = len(re.findall(r"<visual", read(sdf)))
    n_col = len(re.findall(r"<collision", read(sdf)))
    n_light = len(re.findall(r"<light ", read(sdf)))

    print()
    print(f"世界规模       visual {n_vis} / collision {n_col} / light {n_light}")
    print(f"渲染线程就绪   {t_render:.1f} s" if t_render is not None else "渲染线程就绪   未出现（超时）")
    print(f"开始步进       {t_stats:.1f} s" if t_stats is not None else "开始步进       未出现（仿真被卡住）")
    print(f"RTF            {rtf}" + (f"  iterations={iters}" if iters else "") if rtf else "RTF            拿不到")
    if "failed to create dri2" in txt:
        print("EGL           有 dri2 警告（本机常见；小世界实验证明它不影响出数据）")
    print()
    ok = t_stats is not None and (rtf or 0) > 0.1
    print("RESULT:", "PASS" if ok else "FAIL")
    if not ok:
        print("结论：这个规模的场景初始不起来（不是 EGL 通不通的问题）。下一步按收益排序减负：")
        print("  a) config/world.yaml: visual_detail: low            （视觉件 2832 -> ~570）")
        print("  b) 关阴影：sdf 里 <shadows>false</shadows>")
        print("  c) 少灯：18 个 spot -> 每巷 1 个（照明均匀度主要靠 ambient）")
        print("  d) 降雷达采样：1024x32 -> 512x32")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
