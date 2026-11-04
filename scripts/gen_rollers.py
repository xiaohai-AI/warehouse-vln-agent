#!/usr/bin/env python3
"""生成麦克纳姆轮的辊子布局（xacro 片段），`make robot` 会先跑它。

为什么用生成器：辊子的位姿要解 45° 倾角的旋转矩阵，手算 9×2 组容易写错，
而写错的后果在 GUI 里才看得出来（上一版就是：辊子轴线没有轴向分量、
外沿 0.088 超过碰撞半径 0.076、外面还套了一圈实心胎把辊子全盖住）。

真麦克纳姆轮的几何约束（这里全部满足）
  1. **没有轮胎筒**：辊子外沿包络 = 轮半径 R（碰撞体仍是整圆 R，见 xacro 注释）
  2. 辊子中心圆半径 Rc + 辊子半径 r = R
  3. n 个辊子沿圆周均布：相邻辊子刚好相切 → 2r ≈ 2·Rc·sin(pi/n)
     联立得 Rc = R / (1 + sin(pi/n))，r = R - Rc
  4. 辊子轴线 = cos45°·(轮轴 ŷ) + sin45°·(切向 t̂)  → 与轮轴正好 45°
  5. 辊子轴向长度 L = W / cos45°（两端正好落在轮的两个侧面）
  6. **左右轮镜像**：左轮 hand=+1，右轮 hand=-1

输出：robot/rollers.xacro（由 make robot 调用，别手改）
"""
from __future__ import annotations

import math
from pathlib import Path

R_WHEEL = 0.076          # 轮半径（= 碰撞圆柱半径，物理口径不许动）
WIDTH = 0.050            # 轮宽
N = 9                    # 辊子数
TILT = math.radians(45)  # 辊子轴线与轮轴夹角
OUT = Path(__file__).resolve().parents[1] / "robot" / "rollers.xacro"


def sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def norm(a):
    m = math.sqrt(sum(v * v for v in a))
    return [v / m for v in a]


def rpy_from_R(R):
    """固定轴 XYZ（R = Rz·Ry·Rx）→ (roll, pitch, yaw)。"""
    pitch = math.asin(max(-1.0, min(1.0, -R[2][0])))
    roll = math.atan2(R[2][1], R[2][2])
    yaw = math.atan2(R[1][0], R[0][0])
    return roll, pitch, yaw


def basis(z_axis):
    """给定圆柱轴（默认 +z）应指向的方向，构造正交基并给出 rpy。"""
    z = norm(z_axis)
    # 参考方向取径向，保证两个侧面对称；退化时换一个
    ref = [0.0, 1.0, 0.0] if abs(z[1]) < 0.95 else [1.0, 0.0, 0.0]
    x = norm(cross(z, ref))
    y = cross(z, x)
    R = [[x[0], y[0], z[0]],
         [x[1], y[1], z[1]],
         [x[2], y[2], z[2]]]
    return rpy_from_R(R)


def fmt(v, nd=5):
    s = f"{v:.{nd}f}"
    return "0" if s in ("-0.00000", "0.00000") else s


def main() -> int:
    Rc = R_WHEEL / (1.0 + math.sin(math.pi / N))
    r = R_WHEEL - Rc
    L = WIDTH / math.cos(TILT)
    print(f"辊子：n={N}  Rc={Rc:.4f}  r={r:.4f}  R_c+r={Rc + r:.4f}（=轮半径 {R_WHEEL}）"
          f"  L={L:.4f}  相邻间隙={2 * Rc * math.sin(math.pi / N) - 2 * r:+.4f}")
    assert abs((Rc + r) - R_WHEEL) < 1e-9, "包络必须等于轮半径"
    assert L * math.cos(TILT) - WIDTH < 1e-9, "辊子轴向长度不能超出轮宽"

    parts = [
        '<?xml version="1.0" ?>',
        "<!-- 由 scripts/gen_rollers.py 生成，勿手改（make robot 会重新生成）。",
        "     几何约束见该脚本文件头：辊子包络 = 轮半径、轴线 45°、左右镜像。 -->",
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">',
    ]
    for hand, tag in ((+1, "l"), (-1, "r")):
        parts.append(f'\n  <xacro:macro name="mecanum_rollers_{tag}">')
        for i in range(N):
            th = 2.0 * math.pi * i / N
            # 辊子中心（轮坐标系：轮轴 = y）
            px, pz = Rc * math.cos(th), Rc * math.sin(th)
            # 切向 t̂：位置角对 θ 求导
            tx, tz = -math.sin(th), math.cos(th)
            # 轴线：cos45·ŷ + hand·sin45·t̂
            axis = [hand * math.sin(TILT) * tx,
                    math.cos(TILT),
                    hand * math.sin(TILT) * tz]
            roll, pitch, yaw = basis(axis)
            parts.append(
                f'    <visual name="roller_{tag}{i}">'
                f'<origin xyz="{fmt(px)} 0 {fmt(pz)}" rpy="{fmt(roll)} {fmt(pitch)} {fmt(yaw)}"/>'
                f'<geometry><cylinder radius="{r:.5f}" length="{L:.5f}"/></geometry>'
                f'<material name="amr_grey_lt"/></visual>')
        parts.append("  </xacro:macro>")

    parts.append("\n</robot>")
    OUT.write_text("\n".join(parts) + "\n", encoding="utf-8")
    print(f"→ {OUT}  （{N} 辊 × 2 手性 = {N * 2} 个 visual）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
