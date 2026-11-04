#!/usr/bin/env python3
"""读一次 Nav2 的 costmap（OccupancyGrid），统计**目标点附近有没有致命格**。

## 为什么不用 `ros2 topic echo`

原来的探针（`move_obstacle.sample_costmap_at`）用 `ros2 topic echo --once` + 正则解析，
两条 costmap 都打印"读不到"。原因不是 topic 不存在，而是：

1. `/local_costmap/costmap` 是 160×160 = **25600 个 int**，`topic echo` 逐行打印成 YAML 列表，
   序列化+打印+管道加起来**经常超过几秒**，一超时就返回空（而调用方只看到 `None`）；
2. costmap 的 publisher 用的是 `transient_local` QoS，`ros2 topic echo` 默认不兼容这个 durability，
   可能根本收不到第一条；
3. 正则解析还要区分 `info.origin` / `position` 里那些数字，容易串。

**改成 rclpy 直接订阅**：`transient_local + reliable` 对齐 publisher 的 QoS，拿到的就是原始数组，
解析只做一次索引换算。**独立进程**，不与任何主流程的 spin 挤 context。

## 用法

    # 诊断：车附近有没有致命格（--x/--y 是世界坐标；local 的 odom 帧会自动 +31）
    python3 scripts/costmap_probe.py --topic /local_costmap/costmap --x -26.0 --y -1.25

    # 离线自检（不需要仿真）
    python3 scripts/costmap_probe.py --selftest

输出一行 JSON：`near`（半径内致命格数）、`min_d`（最近致命格到目标点的距离）、
`total`、`frame`、`res/width/height/origin`。
"""
from __future__ import annotations

import argparse
import json
import math
import sys

LEHTAL = 99          # nav2_costmap_2d 的 LETHAL_OBSTACLE
SPAWN_X = -31.0      # 世界原点与 odom 原点的差（见 static_map_pub 的 --spawn-x）


def stats(msg, tx: float, ty: float, radius: float) -> dict:
    """在 OccupancyGrid 里统计目标点附近（欧氏半径）的致命格。

    `msg` 只需要有 `.info`（width/height/resolution/origin）和 `.data`，自检也用它。
    索引约定：行主序，`index = row * width + col`，`col ↔ x`、`row ↔ y`。
    """
    info = msg.info
    w, h, res = int(info.width), int(info.height), float(info.resolution)
    ox = float(info.origin.position.x)
    oy = float(info.origin.position.y)

    near = 0
    min_d = math.inf
    near_list: list[list[float]] = []
    total = 0
    data = msg.data
    for idx in range(min(len(data), w * h)):
        if data[idx] < LEHTAL:
            continue
        total += 1
        gx = ox + (idx % w + 0.5) * res
        gy = oy + (idx // w + 0.5) * res
        d = math.hypot(gx - tx, gy - ty)
        if d <= radius:
            near += 1
            if d < min_d:
                min_d = d
            if len(near_list) < 8:
                near_list.append([round(gx, 2), round(gy, 2), int(data[idx])])
    return {
        "near": near,
        "min_d": None if min_d is math.inf else round(min_d, 3),
        "total": total,
        "res": res,
        "width": w,
        "height": h,
        "origin": [round(ox, 2), round(oy, 2)],
        "near_cells": near_list,
    }


def probe(topic: str, tx: float, ty: float, radius: float, timeout: float,
          offset: float | None) -> dict:
    """订阅一次 costmap 并统计。`offset` 为 None 时按 frame 自动决定（odom → +31）。"""
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)

    rclpy.init()
    node = Node("costmap_probe")
    # 对齐 nav2 costmap publisher：transient_local + reliable（`ros2 topic echo` 默认不兼容这一条）
    qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    got: list = []
    node.create_subscription(OccupancyGrid, topic, lambda m: got.append(m), qos)

    t_end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while not got and node.get_clock().now().nanoseconds / 1e9 < t_end:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not got:
        node.destroy_node()
        rclpy.try_shutdown()
        return {"ok": False, "error": f"{timeout:.0f}s 内没收到 {topic}"}

    msg = got[0]
    frame = msg.header.frame_id or "?"
    if offset is None:
        # **符号最容易错的地方**：机器人在世界 x=SPAWN_X(-31) 出发，而 odom 从 0 起算，
        # 所以 `odom = world - SPAWN_X = world + 31`。写成 `world + SPAWN_X` 会去查 x-62，
        # 那里什么也没有 → 统计出"0 个致命格"，看起来像"costmap 没标上"（实测踩过）。
        offset = -SPAWN_X if frame in ("odom", "local") else 0.0
    out = stats(msg, tx + offset, ty, radius)
    out.update({"ok": True, "topic": topic, "frame": frame, "offset": offset})
    node.destroy_node()
    rclpy.try_shutdown()
    return out


class _FakeInfo:                      # 自检用的最小替身
    def __init__(self, w, h, res, ox, oy):
        self.width, self.height, self.resolution = w, h, res
        self.origin = type("O", (), {"position": type("P", (), {"x": ox, "y": oy})})


class _FakeMsg:
    def __init__(self, w, h, res, ox, oy, data):
        self.info = _FakeInfo(w, h, res, ox, oy)
        self.data = data


def selftest() -> int:
    """离线验证索引换算：在已知位置放致命格，检查统计对不对（不需要 ROS/仿真）。

    行主序很容易写反（把 x/y 弄反），所以这里专门用**不对称**的尺寸和位置来验。
    """
    fails = 0

    def chk(cond: bool, name: str, extra: str = "") -> None:
        nonlocal fails
        print(f"  {'PASS' if cond else 'FAIL'}  {name}{('：' + extra) if extra else ''}")
        fails += 0 if cond else 1

    # 160×120 的栅格（宽≠高，防止把 w/h 写反），res 0.05，原点 (-4,-3)
    # → 覆盖 x∈[-4,4]、y∈[-3,3]，测试点必须落在这个框里（第一次自检就是原点选得离测试点太远，
    #   `idx()` 直接越界 —— 自检抓到的是测试自己的几何错，不是被测代码的错）。
    w, h, res, ox, oy = 160, 120, 0.05, -4.0, -3.0
    data = [0] * (w * h)

    def idx(gx: float, gy: float) -> int:
        cx = int((gx - ox) / res)
        cy = int((gy - oy) / res)
        return cy * w + cx

    # 1) 目标点 (0,0) 正上方放一格
    data[idx(0.0, 0.0)] = LEHTAL
    s = stats(_FakeMsg(w, h, res, ox, oy, data), 0.0, 0.0, 1.0)
    chk(s["near"] == 1 and s["total"] == 1, "单格命中", f"near={s['near']} total={s['total']}")
    chk(s["min_d"] is not None and s["min_d"] < 0.08, "最小距离≈格中心偏移",
        f"min_d={s['min_d']}")

    # 2) 半径外的一格不算
    data = [0] * (w * h)
    data[idx(3.0, 0.0)] = LEHTAL
    s = stats(_FakeMsg(w, h, res, ox, oy, data), 0.0, 0.0, 1.0)
    chk(s["near"] == 0 and s["total"] == 1, "半径外不计入 near",
        f"near={s['near']} total={s['total']}")

    # 3) x/y 不能写反：在 (2, 1) 放一格，目标 (2,1) 应命中、反着查 (1,2) 不应命中
    data = [0] * (w * h)
    data[idx(2.0, 1.0)] = LEHTAL
    s_ok = stats(_FakeMsg(w, h, res, ox, oy, data), 2.0, 1.0, 0.5)
    s_bad = stats(_FakeMsg(w, h, res, ox, oy, data), 1.0, 2.0, 0.5)
    chk(s_ok["near"] == 1 and s_bad["near"] == 0, "行主序 x/y 没写反",
        f"命中 {s_ok['near']} / 反着查 {s_bad['near']}")

    # 4) 未知值 (-1) 与低代价 (0~98) 都不算致命
    data = [0] * (w * h)
    data[idx(0.5, 0.5)] = -1
    data[idx(0.6, 0.6)] = 50
    s = stats(_FakeMsg(w, h, res, ox, oy, data), 0.0, 0.0, 2.0)
    chk(s["total"] == 0, "只认 ≥99 的致命格", f"total={s['total']}")

    # 5) 原点偏移生效：(0,0) 处的格子，若原点改成 (10,10) 则不再命中 (0,0)
    data = [0] * (w * h)
    data[0] = LEHTAL
    s_a = stats(_FakeMsg(w, h, res, 0.0, 0.0, data), 0.025, 0.025, 0.2)
    s_b = stats(_FakeMsg(w, h, res, 10.0, 10.0, data), 0.025, 0.025, 0.2)
    chk(s_a["near"] == 1 and s_b["near"] == 0, "origin 偏移被计入",
        f"origin0 → {s_a['near']} / origin10 → {s_b['near']}")

    # 6) 世界→odom 的偏移符号：(世界 -31 出发) 世界点 -26 → odom x = +5
    chk(abs((-26.0) - SPAWN_X - 5.0) < 1e-9, "odom = world − SPAWN_X（符号）",
        f"世界 -26 → odom {-26.0 - SPAWN_X:+.1f}")

    # 7) 空 data 不炸
    s = stats(_FakeMsg(0, 0, 0.05, 0.0, 0.0, []), 0.0, 0.0, 1.0)
    chk(s["total"] == 0 and s["width"] == 0, "空栅格不崩")

    total_chk = 8
    print(f"\n自检：{total_chk - fails}/{total_chk} 通过")
    return 1 if fails else 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/local_costmap/costmap")
    ap.add_argument("--x", type=float, default=0.0, help="目标点世界 x")
    ap.add_argument("--y", type=float, default=0.0, help="目标点世界 y")
    ap.add_argument("--radius", type=float, default=1.0, help="统计半径（m）")
    ap.add_argument("--timeout", type=float, default=10.0)
    ap.add_argument("--offset", type=float, default=None,
                    help="世界→costmap 帧的平移（默认：frame=odom 时 +31，map 时 0；只做加法）")
    ap.add_argument("--selftest", action="store_true", help="离线自检，不需要 ROS")
    args = ap.parse_args()

    if args.selftest:
        return selftest()
    try:
        out = probe(args.topic, args.x, args.y, args.radius, args.timeout, args.offset)
    except Exception as e:                        # noqa: BLE001
        out = {"ok": False, "error": f"{type(e).__name__}: {e}"}
    print(json.dumps(out, ensure_ascii=False))
    return 0 if out.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
