#!/usr/bin/env python3
"""gz 侧健康探测（不需要 ROS 环境，不会挂住）。

解决的问题：`ign topic -e -t /lidar3d -n 1` 在没有发布者时会**永久阻塞**，
看起来像"卡住了"。这里每个探测都带超时，并且直接读 gz 的 server 日志判断
"Sensors 渲染线程有没有卡住仿真"。

用法
  python3.10 scripts/gz_probe.py
  python3.10 scripts/gz_probe.py --timeout 2.0
"""
from __future__ import annotations

import argparse
import glob
import os
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
IGN = "ign"


def run(cmd: list[str], timeout: float) -> tuple[int, str, bool]:
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return p.returncode, (p.stdout or "") + (p.stderr or ""), False
    except subprocess.TimeoutExpired:
        return -1, "", True


def newest_server_log() -> Path | None:
    logs = sorted(glob.glob(os.path.expanduser("~/.ignition/gazebo/log/*/server_console.log")),
                  key=os.path.getmtime, reverse=True)
    return Path(logs[0]) if logs else None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--timeout", type=float, default=3.0)
    args = ap.parse_args()

    rows: list[tuple[str, bool, str]] = []

    def add(name: str, ok: bool, detail: str) -> None:
        rows.append((name, ok, detail))

    # 1) 进程
    rc, out, _ = run(["pgrep", "-af", "ign gazebo server"], 3.0)
    procs = [l for l in out.splitlines() if "ign gazebo server" in l]
    add("gz server 进程", bool(procs), f"{len(procs)} 个" if procs else "没有（先起 launch）")

    # 2) 话题清单
    rc, out, timed = run([IGN, "topic", "-l"], 6.0)
    topics = set(l.strip() for l in out.splitlines() if l.strip().startswith("/"))
    need = {
        "/clock": "仿真时钟",
        "/world/depot/stats": "世界统计（有它说明仿真在步进）",
        "/model/depot_bot/odom": "里程计",
        "/model/depot_bot/cmd_vel": "速度指令",
        "/joint_states": "轮关节状态",
        "/lidar3d": "32 线点云",
        "/imu": "IMU",
    }
    missing = [f"{t}({why})" for t, why in need.items() if t not in topics]
    add("gz 话题清单", not missing,
        f"{len(topics)} 个话题，全部关键话题在" if not missing else f"缺 {missing}")

    # 3) 关键话题有没有数据（带超时，不会挂）
    data_topics = ["/clock", "/world/depot/stats", "/model/depot_bot/odom", "/joint_states", "/lidar3d", "/imu"]
    no_data = []
    for t in data_topics:
        rc2, out2, timed2 = run([IGN, "topic", "-e", "-t", t, "-n", "1"], args.timeout)
        if timed2 or not out2.strip():
            no_data.append(t)
    add("话题有数据", not no_data, "全部有数据" if not no_data else f"无数据: {no_data}")

    # 4) RTF / 迭代数
    rtf = None
    iters = None
    rc3, out3, timed3 = run([IGN, "topic", "-e", "-t", "/world/depot/stats", "-n", "1"], args.timeout + 1)
    if not timed3 and out3:
        m = re.search(r"real_time_factor:\s*([\d.eE+-]+)", out3)
        n = re.search(r"iterations:\s*(\d+)", out3)
        rtf = float(m.group(1)) if m else None
        iters = int(n.group(1)) if n else None
    add("RTF", bool(rtf and rtf > 0.1), f"RTF={rtf}" + (f" iterations={iters}" if iters else "")
        if rtf else "拿不到 stats（仿真没在跑）")

    # 5) 机器人实体在不在
    rc4, out4, timed4 = run([IGN, "topic", "-e", "-t", "/world/depot/pose/info", "-n", "1"], args.timeout + 1)
    has_robot = "depot_bot" in out4
    add("depot_bot 实体", has_robot, "位姿里有 depot_bot" if has_robot else "位姿里找不到 depot_bot")

    # 6) 读 server 日志：Sensors 渲染线程是否卡死
    log = newest_server_log()
    blocked = None
    log_detail = "没有 server 日志"
    if log is not None:
        txt = log.read_text(encoding="utf-8", errors="ignore")
        wait = re.findall(r"\((\S+?)\).*Waiting for init", txt)
        init = re.findall(r"\((\S+?)\).*Rendering Thread initialized", txt)
        log_detail = f"{log.parent.name}"
        if wait and init:
            # 只比较最后一组
            from datetime import datetime

            def ts(s: str) -> float:
                try:
                    return datetime.fromisoformat(s[:26]).timestamp()
                except Exception:
                    return 0.0

            gap = ts(init[-1]) - ts(wait[-1])
            blocked = gap > 5.0 and "Received signal" in txt
            log_detail += f"；渲染线程等了 {gap:.1f} s 才初始化"
            if blocked:
                log_detail += "（等到了 Ctrl-C 才起来 → 仿真被卡住）"
        elif wait and not init:
            blocked = True
            log_detail += "；渲染线程一直没初始化 → 仿真被卡住"
    add("Sensors 渲染线程", not blocked, log_detail)

    # ---- 输出
    print()
    w = max(len(r[0]) for r in rows) + 2
    for name, ok, detail in rows:
        print(f"[{'PASS' if ok else 'FAIL'}] {name:<{w}} {detail}")
    bad = [r for r in rows if not r[1]]
    print()
    if bad:
        print("RESULT: FAIL")
        names = [b[0] for b in bad]
        if "Sensors 渲染线程" in names or ("话题有数据" in names and "话题清单" not in names):
            print()
            print("诊断：话题都在但没数据，且渲染线程卡住 —— 这是 EGL 初始化失败，不是模型问题。按顺序试：")
            print("  ① ros2 launch launch/depot_sim.launch.py gui:=true egl_vendor:=nvidia")
            print("  ② sed -i 's/^render_engine: ogre2/render_engine: ogre/' config/world.yaml && make world")
            print("     ros2 launch launch/depot_sim.launch.py gui:=true egl_vendor:=none")
            print("  ③ ros2 launch launch/depot_sim.launch.py gui:=true software_render:=true")
        if "gz 话题清单" in names:
            print()
            print("诊断：关键话题都没建起来 —— 世界/机器人没加载成功，或起了两个 gz 实例互相打架。")
            print("  pkill -f 'ign gazebo'  然后重新 launch，确认只有一个 [ign gazebo-1] 进程块")
        return 1
    print("RESULT: PASS —— 仿真在跑、传感器出数、机器人实体在位")
    return 0


if __name__ == "__main__":
    sys.exit(main())
