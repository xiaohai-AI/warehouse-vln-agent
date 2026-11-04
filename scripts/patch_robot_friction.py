#!/usr/bin/env python3
"""给机器人 SDF 的四个轮子注入各向异性摩擦（SPEC §4.2 补救①）。

为什么需要这一步（实测，不是猜）
  1. urdf 里的 `<gazebo reference="wheel_x_link"><mu1>..</mu1><mu2>..</mu2>` 这套写法，
     `gz sdf -p` **静默丢弃**：生成出来的 wheel collision 里一个 `<surface>` 都没有。
     （实验：最小 URDF 里 `<mu1>/<mu2>` 直接子元素 -> 丢失；`<friction><ode>` -> 被塞到
     link 层，位置非法，同样无效。见 docs/TROUBLESHOOTING.md）
  2. 没有摩擦 = 默认各向同性 = 圆柱轮只能沿滚动方向产生力 -> 横移指令下轮子空转、
     车不动。实测：cmd vy=0.3 m/s，真值横向位移 0.000 m（-100%）；前向也有 -22% 的损失。

做法：碰撞面用 ODE 的各向异性摩擦近似 45° 辊子
  mu  = 辊子方向的抓地（大）
  mu2 = 垂直于 fdir1 方向（0，让它自由侧滑）
  fdir1 在 base_footprint 里取 ±(1,∓1,0)（左前/右后一组，右前/左后一组反向），
  四轮合成后既能出力向前、也能出力横向 —— 这是 gz 里模拟麦克纳姆的标准近似。

用法
  python3 scripts/patch_robot_friction.py robot/depot_bot.sdf          # 就地注入
  python3 scripts/patch_robot_friction.py robot/depot_bot.sdf --check   # 只检查，不写
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

# 四个轮子 -> fdir1（与官方 mecanum_drive.sdf 的取法一致：对角同向）
WHEELS = {
    "fl": (1, -1),
    "fr": (1, 1),
    "rl": (1, 1),
    "rr": (1, -1),
}
SURFACE = """<surface>
          <friction>
            <ode>
              <mu>{mu}</mu>
              <mu2>{mu2}</mu2>
              <fdir1 ignition:expressed_in="base_footprint">{fx} {fy} 0</fdir1>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>{kp}</kp>
              <kd>{kd}</kd>
              <min_depth>{min_depth}</min_depth>
              <max_vel>0.0</max_vel>
            </ode>
          </contact>
        </surface>"""


def collision_name(prefix: str, txt: str) -> str | None:
    """找出这个轮子的轮胎 collision 名（gz sdf -p 会把 fixed joint 的子件合并，名字带前缀）。"""
    m = re.search(rf"<collision name='(wheel_{prefix}_link[^']*tire[^']*)'", txt)
    return m.group(1) if m else None


def has_surface(txt: str, name: str) -> bool:
    i = txt.index(f"<collision name='{name}'>")
    j = txt.index("</collision>", i)
    return "<surface>" in txt[i:j]


def inject(txt: str, name: str, mu: float, mu2: float, fx: int, fy: int,
           kp: float, kd: float, min_depth: float) -> str:
    i = txt.index(f"<collision name='{name}'>")
    j = txt.index("</collision>", i)
    block = txt[i:j]
    if "<surface>" in block:
        return txt
    surface = SURFACE.format(mu=mu, mu2=mu2, fx=fx, fy=fy, kp=kp, kd=kd, min_depth=min_depth)
    return txt[:j] + surface + "\n      " + txt[j:]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("sdf", nargs="?", default="robot/depot_bot.sdf")
    ap.add_argument("--check", action="store_true", help="只检查四个轮子有没有摩擦，不写文件")
    ap.add_argument("--mu", type=float, default=3.0,
                    help="辊子方向抓地。实测 1.0 与 3.0 在修正量法后区别不明显（不是瓶颈），取 3.0 更稳")
    # mu2 的 A/B 实测（都在 M2 上量真值落点误差）：
    #   mu2=0.0 -> 1.22~1.28 m（5 轮，最好）
    #   mu2=0.2 -> 3.63~3.73 m（更差：长距离段向北飘、顶在货架面上）
    # 单看短程混合运动 (0.1,-0.15) 时 mu2=0.2 反而更好（真值 dyaw +0.0° vs mu2=0 的 -9.8°），
    # 但长距离闭环下更糟 —— 所以保持 0.0，把"按行程加自适应侧向摩擦"留给后续。
    ap.add_argument("--mu2", type=float, default=0.0)
    ap.add_argument("--kp", type=float, default=1.0e6)
    ap.add_argument("--kd", type=float, default=100.0)
    ap.add_argument("--min-depth", type=float, default=0.001)
    ap.add_argument("--isotropic", action="store_true",
                    help="关掉各向异性（mu2=mu），用于 A/B：验证横移是否真的靠 fdir1")
    args = ap.parse_args()

    path = Path(args.sdf)
    txt = path.read_text(encoding="utf-8")
    mu2 = args.mu if args.isotropic else args.mu2

    names = {}
    for prefix in WHEELS:
        nm = collision_name(prefix, txt)
        if nm is None:
            print(f"FAIL 找不到 wheel_{prefix} 的轮胎 collision —— SDF 结构变了？")
            return 2
        names[prefix] = nm

    if args.check:
        missing = [p for p, nm in names.items() if not has_surface(txt, nm)]
        for p, nm in names.items():
            ok = has_surface(txt, nm)
            print(f"  [{'PASS' if ok else 'FAIL'}] {p}: {nm} {'有 <surface>' if ok else '没有摩擦设置'}")
        print("\nRESULT:", "PASS" if not missing else f"FAIL（{len(missing)} 个轮子没有摩擦）")
        return 0 if not missing else 1

    for prefix, (fx, fy) in WHEELS.items():
        txt = inject(txt, names[prefix], args.mu, mu2, fx, fy, args.kp, args.kd, args.min_depth)
    path.write_text(txt, encoding="utf-8")

    # 注入后立刻自检：四个轮子都必须带上 surface + fdir1
    bad = [p for p, nm in names.items() if not has_surface(txt, nm)]
    n_fdir = len(re.findall(r"<fdir1 ignition:expressed_in=", txt))
    print(f"[friction] 注入 mu={args.mu} mu2={mu2} kp={args.kp:.0e} kd={args.kd:.0f} "
          f"-> {len(WHEELS) - len(bad)}/4 轮，fdir1 共 {n_fdir} 处")
    if bad:
        print(f"FAIL 注入失败：{bad}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
