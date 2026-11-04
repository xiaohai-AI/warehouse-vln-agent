#!/usr/bin/env python3
"""动态障碍：在巷道里横穿一台推车，用来测**动态**避障（而不是静态贴边）。

为什么用"运行时注入"而不是写进世界 SDF
  世界生成链有硬闸门 **A5「全部实体 static」**，且静态地图只能从 static 碰撞体派生。
  把会动的推车塞进 SDF 会同时破坏这两件事（世界不再确定、地图可能被烘进去）。
  → 用 `/world/depot/create` 现造 + `/world/depot/set_pose` 移动，**世界零改动**。

行为（也是判据的输入）
  1. 轮询真值位姿，等机器人开到距横穿点 `--trigger` 米以内；
  2. 造推车（已存在则忽略），从巷道一侧滑到另一侧（默认 0.45 m/s 仿真速度）；
  3. 到中线停 `--hold` 秒（前向安全层应当把车压住）；
  4. 滑出巷道停住（安全层应放行，机器人继续开）；
  5. **自检**：读回推车位姿，位移与指令不符就报 FAIL —— 不许"看起来动了"。

位姿读取复用 `scripts/slam_route.py::model_pose()`（与机器人真值同一份实现，不另写）。

用法（仿真在跑；`DYN=1 make p6-demo` 会自动调起）
  python3 scripts/move_obstacle.py --x -16.0 --aisle C --trigger 6.0
  python3 scripts/move_obstacle.py --dry          # 只打印计划，不碰仿真
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

WORLD = "depot"
CART = "dyn_cart"
CART_SDF = ROOT / "world" / "fixtures" / "dyn_cart.sdf"
AISLE_Y = {"A": -10.75, "B": -6.45, "C": -2.15, "D": 2.15, "E": 6.45, "F": 10.75}
LANE_HALF = 1.60            # 巷道净宽 3.20 → 半宽；货架面就在 ±1.60
CART_SIZE = (0.62, 0.52, 0.90)
STEP = 0.45                 # 每次 set_pose 挪 0.45 m（每次都是一次 shell 调用，步子太小会拖时间）


def log(msg: str) -> None:
    print(f"[dyn] {msg}", flush=True)


def write_cart_sdf() -> Path:
    """推车 SDF（**脚本生成**，不手写）。

    **必须 dynamic（static=false）**：实测 static 实体的 `set_pose` 只改位姿组件，
    **物理碰撞体不跟着搬** → 激光雷达（射线检测走物理）看不见它，机器人直接从车身上
    开过去（实测：安全层 0 次压住、机器人开到距横穿点 0.58 m 却没碰到东西）。
    dynamic + `gravity=false`（无外力就不会漂）+ 高摩擦，teleport 才真正改碰撞位置。
    """
    w, d, h = CART_SIZE
    mass = 40.0
    ix = mass * (d * d + h * h) / 12.0
    iy = mass * (w * w + h * h) / 12.0
    iz = mass * (w * w + d * d) / 12.0
    CART_SDF.parent.mkdir(parents=True, exist_ok=True)
    CART_SDF.write_text(
        f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="{CART}">
    <static>false</static>
    <gravity>false</gravity>
    <link name="body">
      <inertial>
        <mass>{mass}</mass>
        <inertia><ixx>{ix:.4f}</ixx><iyy>{iy:.4f}</iyy><izz>{iz:.4f}</izz></inertia>
      </inertial>
      <collision name="col">
        <geometry><box><size>{w} {d} {h}</size></box></geometry>
        <surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction></surface>
      </collision>
      <visual name="vis">
        <geometry><box><size>{w} {d} {h}</size></box></geometry>
        <material><ambient>0.75 0.45 0.10 1</ambient><diffuse>0.85 0.52 0.12 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
""", encoding="utf-8")
    return CART_SDF


def sample_plan_points(timeout: float = 8.0) -> list[tuple[float, float]]:
    """读一次 Nav2 的全局规划 `/plan`，返回路径点的 y 范围。

    判"绕行"要看**规划器有没有改路线**：路径 y 范围若偏离行驶线 >0.3 m，说明规划器确实
    绕了（那机器人不走就是控制器的问题）；若始终贴着行驶线，说明规划器没绕（costmap 或
    planner 的问题）。不采这个点，只能在"停/等"和"绕行"之间猜。
    """
    import re
    import subprocess

    try:
        p = subprocess.run(["ros2", "topic", "echo", "--once", "/plan"],
                           capture_output=True, text=True, timeout=timeout)
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return []
    txt = (p.stdout or "")
    pts = [(float(m.group(1)), float(m.group(2))) for m in
           re.finditer(r"position:\s*x:\s*(-?[\d.eE+-]+)\s*y:\s*(-?[\d.eE+-]+)", txt)]
    return pts


def sample_plan_y(timeout: float = 6.0) -> tuple[float, float] | None:
    pts = sample_plan_points(timeout=timeout)
    return (min(y for _x, y in pts), max(y for _x, y in pts)) if pts else None


def plan_y_at(pts: list[tuple[float, float]], x0: float) -> float | None:
    """路径在 x=x0 处的 y（线性插值）。这是"绕了多少"的唯一硬指标。"""
    if not pts:
        return None
    pts = sorted(pts)
    if x0 <= pts[0][0]:
        return pts[0][1]
    if x0 >= pts[-1][0]:
        return pts[-1][1]
    for (xa, ya), (xb, yb) in zip(pts, pts[1:]):
        if xa <= x0 <= xb and abs(xb - xa) > 1e-9:
            return ya + (yb - ya) * (x0 - xa) / (xb - xa)
    return None


def sample_costmap_at(topic: str, wx: float, wy: float, radius: float = 1.0,
                      timeout: float = 10.0) -> tuple[int, int, float | None] | None:
    """读一次 OccupancyGrid，返回 (目标点附近致命格数, 全图致命格数)。

    **这是"激光看见了 ≠ costmap 知道"的分界探针**。上一版用 `ros2 topic echo --once` + 正则解析，
    两条 costmap 全部返回"读不到"，原因是：
      ① 160×160=25600 个 int 逐行打印，序列化+管道常超 8 s → 超时返回空；
      ② costmap publisher 是 `transient_local`，`topic echo` 默认不兼容该 durability，可能收不到消息；
      ③ 正则还要在 `info.origin`/`position` 的一堆数字里区分，容易串。
    现在改成调 `scripts/costmap_probe.py`（rclpy 直接订阅，独立进程，QoS 对齐 publisher）。
    """
    import json
    import subprocess

    argv = [sys.executable, str(Path(__file__).resolve().parent / "costmap_probe.py"),
            "--topic", topic, "--x", f"{wx:.3f}", "--y", f"{wy:.3f}",
            "--radius", f"{radius:.2f}", "--timeout", f"{timeout:.1f}"]
    try:
        p = subprocess.run(argv, capture_output=True, text=True, timeout=timeout + 8)
    except subprocess.TimeoutExpired:
        return None
    for line in (p.stdout or "").splitlines():
        line = line.strip()
        if not line.startswith("{"):
            continue
        try:
            d = json.loads(line)
        except Exception:
            continue
        if not d.get("ok"):
            return None
        # min_d 必须带出来：只报"附近有 N 个致命格"分不清是**推车**还是**隔壁货架**
        # （实际半径 1 m 内容易把货架算进来）。min_d 接近 0 → 就是推车本体。
        return int(d.get("near") or 0), int(d.get("total") or 0), d.get("min_d")
    return None


def sample_scan_min(timeout: float = 6.0) -> float | None:
    """读一次 `/scan` 的最近回波（m）。用来判"激光到底有没有看见这台车"。

    为什么必须查：`scan_relay --type ring` 是**挑看得最远的那一圈**；如果挑中的那圈略微
    上仰，就会从 0.9 m 高的推车**头顶掠过去** —— 那么 costmap 里永远没有它，
    规划器当然不会绕，而现场看起来像"Nav2 不会避障"。
    """
    import re
    import subprocess

    try:
        p = subprocess.run(["ros2", "topic", "echo", "--once", "/scan"],
                           capture_output=True, text=True, timeout=timeout)
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return None
    txt = p.stdout or ""
    i = txt.find("ranges:")
    if i < 0:
        return None
    seg = txt[i:i + 40000]
    vals = []
    for m in re.finditer(r"-?[\d.]+(?:e[-+]?\d+)?", seg):
        try:
            v = float(m.group(0))
        except ValueError:
            continue
        if 0.02 < v < 60.0:
            vals.append(v)
    return min(vals) if vals else None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--aisle", default="C")

    ap.add_argument("--x", type=float, default=-16.0, help="横穿点的世界 x")
    ap.add_argument("--trigger", type=float, default=5.0,
                    help="机器人距横穿点多近时**开始横穿**（m）。要留出横穿的 10 s，"
                         "太近会让车摆位时机器人已经贴上来（主兜底是后面的闭环等待）。"
                         "`0` = 不等机器人：直接把车摆到挡路位并停留（快速避障试验台用）")
    ap.add_argument("--near", type=float, default=1.6,
                    help="车摆到挡路位后，等机器人进到多近才开始计时停留（m）")
    ap.add_argument("--speed", type=float, default=1.0, help="推车速度（m/s，**仿真速度**）")
    ap.add_argument("--rtf", type=float, default=0.44, help="实时因子（墙钟 = 仿真 / RTF）")
    ap.add_argument("--hold", type=float, default=4.0, help="挡在路上停多久（仿真秒）")
    ap.add_argument("--block-y", type=float, default=None,
                    help="挡路位置的 y（默认=机器人自己那条行驶线：C 巷中线 +0.55）")
    ap.add_argument("--north", action="store_true", help="从 +y 侧滑向 −y 侧（默认相反）")
    ap.add_argument("--dry", action="store_true")
    args = ap.parse_args()

    y0 = AISLE_Y[args.aisle]
    side = 1.0 if args.north else -1.0
    y_start = y0 + side * (LANE_HALF - CART_SIZE[1] / 2 - 0.02)
    # **必须挡在机器人自己那条线上**，不能停在巷道中线：机器人沿 LANE_DY 偏移行驶
    # （C 巷 +0.55），推车停在中线时侧向差 0.55 m —— 那正是前向扇区的盲区
    # （要落进 ±25° 需前向 ≥1.18 m，届时斜距 1.30 m 已超 0.55 阈值，不会压住）。
    y_block = args.block_y if args.block_y is not None else y0 + (0.55 if args.aisle == "C" else 0.0)
    y_mid = y_block
    y_park = y_start                                 # 退回出发边（不把车塞进货架里）
    wall_per_step = (STEP / max(0.05, args.speed)) / max(0.1, args.rtf)

    log(f"计划：巷 {args.aisle}(中线 y={y0:+.2f})，横穿点 x={args.x:+.2f}；"
        f"从 {y_start:+.2f} 横穿到**挡路位 {y_mid:+.2f}**（停 {args.hold:.1f} 仿真秒"
        f"≈{args.hold / max(0.1, args.rtf):.1f} 墙钟秒）→ 退回 {y_park:+.2f}；"
        f"速度 {args.speed} m/s；每步 {wall_per_step:.2f} 墙钟秒")
    if args.dry:
        return 0

    from slam_route import _run, model_pose          # 复用真值读取与 shell 封装

    def svc(service: str, reqtype: str, req: str) -> bool:
        rc_out = _run(["ign", "service", "-s", f"/world/{WORLD}/{service}",
                       "--reqtype", reqtype, "--reptype", "ignition.msgs.Boolean",
                       "--timeout", "5000", "--req", req], timeout=15)
        ok = "data: true" in rc_out
        if not ok:
            log(f"{service} 未返回 true：{rc_out.strip()[:160]}")
        return ok

    def set_pose(y: float) -> bool:
        return svc("set_pose", "ignition.msgs.Pose",
                   f'name: "{CART}", position: {{x: {args.x:.3f}, y: {y:.3f}, z: 0.45}}')

    # 1) 等机器人接近（`--trigger 0` = 不等，给快速试验台用：车先到位，机器人自己开过来）
    #    **不能写成 `while cond: ... else:`** —— while 的条件一开始就为假时，else 分支照样会
    #    执行，于是 trigger=0 会立刻打出"超时放弃"就退出（踩过：试验台白跑一轮）。
    if args.trigger <= 0:
        log("trigger=0：不等机器人，直接把车摆到挡路位")
    else:
        log("等机器人开到横穿点附近 …")
        t0 = time.monotonic()
        near = False
        while time.monotonic() - t0 < 300:
            p = model_pose("depot_bot")
            if p is not None and abs(p[0] - args.x) <= args.trigger:
                log(f"触发：机器人 x={p[0]:+.2f}（距横穿点 {abs(p[0] - args.x):.2f} m）")
                near = True
                break
            time.sleep(0.5)
        if not near:
            log("超时：机器人始终没靠近，放弃（不算通过）")
            return 2

    # 2) 造车
    if not svc("create", "ignition.msgs.EntityFactory",
               f'sdf_filename: "{write_cart_sdf()}", name: "{CART}", '
               f'pose: {{position: {{x: {args.x:.3f}, y: {y_start:.3f}, z: 0.45}}}}'):
        log("create 失败（可能已存在）→ 直接尝试 set_pose")
    time.sleep(1.0)
    before = model_pose(CART)

    # 3) 横穿到挡路位；**早检**一次，确认 static 实体的 set_pose 真的生效
    #    （Fortress 的 set_pose 对 static 模型应当生效，但这条不能靠"应该"——
    #     失效时整轮实验作废，早发现只损失 3 s 而不是 60 s）
    n = max(1, int(abs(y_mid - y_start) / STEP))
    for i in range(1, n + 1):
        set_pose(y_start + (y_mid - y_start) * i / n)
        time.sleep(wall_per_step)
    log(f"已挡在行驶线上 y={y_mid:+.2f}；等机器人进到 {args.near:.1f} m 内再计时"
        "（避免\"车走完了机器人还没到\"）")
    t1 = time.monotonic()
    gap = None
    while time.monotonic() - t1 < 90:
        p = model_pose("depot_bot")
        if p is not None:
            gap = abs(p[0] - args.x)
            if gap <= args.near:
                break
        time.sleep(0.5)
    log(f"机器人距横穿点 {'' if gap is None else f'{gap:.2f} m'} → 停 {args.hold} 仿真秒"
        "（此时车**停在路上**）")
    # **关键测量**：挡路期间机器人还在往前推进吗？
    #   drive_to 后端：会被前向安全层压住 → x 停在横穿点之前，等车让开再走（"停/等"）
    #   nav2 后端：costmap 里出现障碍 → 重新规划绕行 → x 继续增大并越过横穿点（"绕行"）
    # 只判"到没到达"分不出这两者（两者都会到达），必须看挡路期间的 x 推进。
    hold_wall = args.hold / max(0.1, args.rtf)
    t2 = time.monotonic()
    xs: list[float] = []
    plans: list[tuple[float, float]] = []      # (y_min, y_max) of the global plan
    scans: list[float] = []                    # 挡路期间 /scan 的最近回波
    cms: list[tuple[str, int, int, float | None]] = []   # (topic, 附近致命格, 全图, 最近距离)
    plan_at: list[float] = []                  # 路径在 x=推车处 的 y
    cm_tried = False                           # costmap 探针只试一次
    while time.monotonic() - t2 < hold_wall:
        p = model_pose("depot_bot")
        if p is not None:
            xs.append(p[0])
            if len(xs) % 3 == 1:               # 约每 2.4 s 看一次规划
                pl = sample_plan_y()
                if pl is not None:
                    plans.append(pl)
                pp = sample_plan_points()
                if pp:
                    at = plan_y_at(pp, args.x)
                    if at is not None:
                        plan_at.append(at)
                smin = sample_scan_min()
                if smin is not None:
                    scans.append(smin)
                # costmap 探针当前解析不了（TROUBLESHOOTING §35.5）；只试一次且 3 s 超时，
                # 否则每轮吃掉十几秒，把姿态采样饿到 1 个点（实测：推进量失去分辨率）。
                if not cm_tried:
                    for topic in ("/local_costmap/costmap", "/global_costmap/costmap"):
                        cm = sample_costmap_at(topic, args.x, y_mid, timeout=6.0)
                        if cm is not None:
                            cms.append((topic, cm[0], cm[1], cm[2]))
                    cm_tried = True
        time.sleep(0.8)
    if plans:
        ymin = min(a for a, _b in plans)
        ymax = max(b for _a, b in plans)
        log(f"Nav2 全局规划（挡路期间各次采样）y 范围: {ymin:+.2f} ~ {ymax:+.2f}"
            f"（行驶线 y={args.block_y if args.block_y is not None else y0 + 0.55:+.2f}；"
            f"偏出 >0.3 m 才算真绕行）")
    else:
        log("Nav2 全局规划：挡路期间读不到 /plan（Nav2 没在跑？或没有活动目标）")
    if plan_at:
        y_at = sum(plan_at) / len(plan_at)
        # **真绕行的唯一硬指标**：路径在推车那个 x 上的 y，离推车的 y 有多远。
        # 只看 y 的 min/max 会被"斜线/正常转弯"骗过去（踩过：半径 0.8 m 的擦过被算成绕行）。
        dev = y_at - args.block_y if args.block_y is not None else y_at - y_mid
        log(f"路径在 x={args.x:+.2f} 处 y≈{y_at:+.2f}，推车在 y={y_mid:+.2f}"
            f" → 偏出 {abs(dev):.2f} m（<0.5 m 视为**没绕**）")
    if scans:
        log(f"/scan 最近回波（挡路期间）: {min(scans):.2f} ~ {max(scans):.2f} m —— "
            f"推车就在车前 ~1 m，若最近回波仍 >2 m 说明**激光没打到它**（挑中的那圈上仰）")
    else:
        log("/scan 读不到（scan_relay 没跑？）")
    for topic in ("/local_costmap/costmap", "/global_costmap/costmap"):
        rows = [(n, t, m) for tp, n, t, m in cms if tp == topic]
        if not rows:
            log(f"{topic}: 读不到（costmap_probe.py 没返回）")
            continue
        n_max = max(a for a, _b, _c in rows)
        t_max = max(b for _a, b, _c in rows)
        ds = [c for _a, _b, c in rows if c is not None]
        d_min = min(ds) if ds else None
        # 判读要看 **min_d**：半径 1 m 内很容易把隔壁货架算进来，只有 min_d≈0 才是推车本体
        if n_max == 0:
            verdict = "**没标上**"
        elif d_min is not None and d_min <= 0.45:
            verdict = "**标上了（就是推车本体）**"
        else:
            verdict = f"附近有致命格但最近的在 {d_min:.2f} m —— **可能是隔壁货架**"
        log(f"{topic}: 推车位附近致命格最多 {n_max} 个（全图 {t_max} 个），"
            f"最近距离 {('%.2f m' % d_min) if d_min is not None else '?'} —— {verdict}")
    if xs:
        log(f"挡路期间机器人 x: {xs[0]:+.2f} → {xs[-1]:+.2f}（推进 {xs[-1] - xs[0]:+.2f} m，"
            f"共 {len(xs)} 个采样）")
        passed = xs[-1] > args.x + 0.5
        log(f"结论：{'**绕过/通过**（车仍在路上时机器人已越过横穿点）' if passed else '**停车等待**（机器人被挡在横穿点之前）'}")

    p_mid = model_pose(CART)
    if p_mid is None or abs(p_mid[1] - y_mid) > 0.35:
        log(f"早检失败：推车没到挡路位（读到 {p_mid}）→ static set_pose 可能不生效，"
            "实验作废（可改用 create/remove 策略）")
        return 4
    log(f"早检通过：推车位姿 ({p_mid[0]:+.2f},{p_mid[1]:+.2f}) 与指令一致")

    # 4) 滑出巷道
    n = max(1, int(abs(y_park - y_mid) / STEP))
    for i in range(1, n + 1):
        set_pose(y_mid + (y_park - y_mid) * i / n)
        time.sleep(wall_per_step)
    log(f"已让开：y={y_park:+.2f}")

    # 5) 自检：挡路位与退回位都核对过（**别拿起点比起点**，那是恒 0 的假自检）
    time.sleep(0.5)
    after = model_pose(CART)
    if before is None or after is None:
        log("自检失败：读不到推车位姿（set_pose 可能没生效）→ 这次实验无效")
        return 3
    d_back = abs(after[1] - y_start)                   # 退回位偏差（挡路位误差已在早检里验）
    d_span = abs(y_mid - before[1])                    # 起点 → 挡路位（设计值 ≈1.87 m，只报不判）
    ok = d_back < 0.35
    log(f"自检：起点→挡路位 {d_span:.2f} m（设计跨度，仅记录）、退回位偏差 {d_back:.2f} m"
        f"（起点 {before[1]:+.2f} / 挡路 {y_mid:+.2f} / 退 {after[1]:+.2f}）"
        f" → {'PASS' if ok else 'FAIL'}")
    log("注意：以上读的是 set_pose 的位姿组件；**物理碰撞体是否跟着搬要靠安全层压住来判**")
    return 0 if ok else 3


if __name__ == "__main__":
    raise SystemExit(main())
