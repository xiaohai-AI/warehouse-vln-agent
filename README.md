# depot-vln

亚马逊式自动化仓库里的 **麦克纳姆全向车 + 双目/3D 激光/IMU 组合 SLAM + 从感知长出来的语义地图 + LangGraph 指代导航**。

入口文档：[SPEC.md](SPEC.md)（需求源）· **[docs/REPORT.md](docs/REPORT.md)（技术报告：每个数字 + 复跑命令 + 失败清单）**

```
不自动跑 ros2：世界生成、指令基准、语义分析都是纯离线的一条 make。
仿真、Nav2、量测这几步按约定手动起（见下）。
```

## 现在能跑什么（都有实测输出，不是"应该能跑"）

`make accept` 当前：**PASS 16 / FAIL 0 / 未跑 0**（明细 [docs/ACCEPTANCE.md](docs/ACCEPTANCE.md)，脚本生成别手改）。
已知边界与未测项在 [docs/LIMITATIONS.md](docs/LIMITATIONS.md)（唯一权威），判据→产物→复现命令在 [docs/DELIVERY.md](docs/DELIVERY.md)，演示脚本在 [docs/DEMO.md](docs/DEMO.md)。

| 判据 | 阶段 | 实测 |
|---|---|---|
| P1 世界自检（22 项） | P1 | **PASS** 巷道净宽 3.200 m（144 站位 min=mean=max）、A4b 挡轮护栏、G0/G0b 货位空间唯一 |
| P1 地图一致性（5 项） | P1 | **PASS** 390 静态碰撞体、288 入库点里 287 可用、6 巷中线全自由 |
| P1 性能 | P1 | **PASS** visual 1334 / collision 573、步进 2.3 s、RTF **0.9925** |
| P0 / LANG-1 | P0/P6 | **PASS** **69/69 = 100%**，歧义静默猜错 0 次 |
| M1 速度跟踪 | P2 | **PASS** 4/4，真值 vx +0.1% / vy −0.1% / 斜行 0.0° |
| M2 侧移入库 | P2 | **PASS** 5/5，真值 0.012–0.098 m |
| S1 双目深度 | P3 | **PASS** 中位 **1.71%** / p95 **6.06%**（640×480，302,058 px） |
| SLAM-1 S1 / S2 | P4 | **PASS** ATE **0.356 / 0.332 m**，地图 90.6% / 89.8% |
| SLAM-1 S3 | P4 | **未达标** ATE 32.15 m / 地图 25.0% —— 归因：周期货架主导对应，沿轴 9–14% vs 横向 94–103%，见 REPORT §5.1 |
| **SLAM-1 S4** | P4 | **ATE 0.288 m / 漂移 0.260 / 地图 90.9%** —— 同一部相机只取地面带做 2D 光流，三项全面优于 S1；未达自设目标 0.6×S1（该线属 SPEC 的 S3），见 REPORT §5.1b |
| **SLAM-1 S5** | P4 | **ATE 0.227 m / 漂移 0.159 / 地图 93.7%** —— 地面光流(体坐标速度)+轮速(vx)+去偏陀螺(航向) 的 EKF；里程计链 0.291 m、偏航误差 ≤0.21°；五配置最好，相对 S1 降 36%，见 REPORT §5.1c |
| GRAPH-1 | P5 | **PASS** 路径中位误差 **7.2%**（156 节点 / 158 边） |
| SEG-1 bay 网格 | P5 | **PASS** **144/144 bay**，中位 0.030 m、数量 0.0%、朝向 0.003° |
| SEG-2 占用/类别 | P5 | **PASS** precision / recall / top-1 **全 1.000**、错格率 0% |
| P6-1 约束层 | P6 | **PASS** 离线自检 34 项；真模型 69/69（对照裸模型 52/69、静默猜 3）|
| P6-2 真工具层 | P6 | **PASS** 真跑 52 次（直达 34 / 改道 1 / 失败或调试 17），最长 68.5 m，末段误差最好 0.00 m |
| P6-2b verify 占用判定 | P6 | **PASS** 真跑 4/4（空/有、最小箱/最高桶 drum、L/R 面、三条巷）|
| P6-2c verify 离线审计 | P6 | **PASS** 39 份深度存档复算，基准 24/24；分离度 **空位 0 px vs 有货 689–4142 px** |
| P6-2d verify 俯仰容限 | P6 | **PASS** 安全带 **−4°…+3°**：仰 +3°→311 px 准 / **+4°→126 px 漏检**；俯 −4°→17 px 准 / **−5°→307 px 假阳性** |
| P6-2e 动态障碍 | P6 | **PASS** 推车横穿行驶线 → 前向安全层压住 → 让开后继续到达、verify 正确 |
| P6-2f Nav2 绕行 | P6 | **PASS** 两层 costmap 都标注（0.04 m）→ 路径到车心 **0.63 m（受控台）/ 0.74 m（整条 demo）**；A/B 反证：未标注时只 0.40 m（穿进膨胀区）|

## 快速开始

```bash
git clone <this-repo> depot-vln && cd depot-vln

# 0) 依赖（一次性）
python3.10 -m venv --system-site-packages .venv
.venv/bin/pip install -r requirements.txt

# 1) 世界（纯离线，秒级）+ 地图 + 性能
make world            # -> world/warehouse.sdf + slot_grid.json + ground_truth/*（22 项自检）
make map-check        # 静态地图 5 项一致性
make perf             # 规模与 RTF

# 2) 机器人模型（纯离线）
make robot            # xacro -> .urdf -> .sdf（含摩擦注入与 SDF 语法检查）

# 3) 指令基准 + 语义图（纯离线，不需要 ROS）
make bench && make bench-run    # 69 条基准 -> results/bench_<date>.csv + transitions/*.jsonl
make graph                      # GRAPH-1

# 4) 汇总
make accept           # -> docs/ACCEPTANCE.md
```

判据可以**一条命令自证**（离线，不需要 ROS/GPU，CI 跑的就是这条）：

```bash
make ci        # = bash scripts/ci_local.sh：判据账本 + 文档一致 + 交付自检 + 安全层自检
```

需要仿真的判据（**手动起**，按顺序）：

```bash
make sim SIM_ARGS='gui:=false'   # 或 ros2 launch launch/depot_sim.launch.py gui:=true 看画面
make cam-bridge                  # 相机桥（同时是渲染开关）
make m1-truth                    # M1
make m2-static                   # M2（需 Nav2）
make s1-depth                    # S1 双目深度
CFG=s1 make slam1                # SLAM-1（s2 / s3 同理）
make seg2-robot && make seg2-capture && make seg2   # SEG-2
make gpu-observe                 # 开渲染前后各观察 10 分钟（GPU 是风险点）
make down                        # 收工必跑
```

## 自动验证（CI）

`.github/workflows/verify.yml` 在每次 push / PR 跑**离线判据链**（约 1 分钟，无 ROS、无 GPU、无显示器）：

| 步骤 | 检查什么 |
|---|---|
| `make accept` | 16 条判据重新对账 → 重写 `docs/ACCEPTANCE.md`（含 P6-1 对抗端到端、P6-2c 深度图复算）|
| `make docs-check` | 9 项文档/产物一致性（合计=行数、判据版本号、存档数…）|
| `bash scripts/delivery_check.sh` | 16 条判据记账 + 文档一致 + 产物齐（不硬编码 PASS 数）|
| `make guard-selftest` | 前向安全层 7 例纯逻辑 |

依赖见 `requirements-ci.txt`（= Agent 层依赖 + numpy；**不需要 rclpy**）。**本地等价验证**（隔离 venv，
无系统包、无 rclpy，等价于 runner）：

```bash
python3.10 -m venv /tmp/ci-venv2 && /tmp/ci-venv2/bin/pip install -r requirements-ci.txt   # 实测 8 s
PATH=/tmp/ci-venv2/bin:$PATH CI_PY=/tmp/ci-venv2/bin/python bash scripts/ci_local.sh        # 4/4
```
不放进 CI 的：`make world`（要 opencv 且会重写被跟踪的真值）、以及所有需要 Gazebo/GPU 的判据 ——
后者的证据以产物形式提交，见 [docs/DELIVERY.md](docs/DELIVERY.md)。

## 文档地图

| 文件 | 作用 |
|---|---|
| [SPEC.md](SPEC.md) | 需求源（判据原文都在这里） |
| **[docs/REPORT.md](docs/REPORT.md)** | 技术报告：架构、每个判据的实测 + 复跑命令、**失败与限制**、Humble→Jazzy 迁移清单 |
| [docs/PHASES.md](docs/PHASES.md) | 阶段闸门逐条对账 |
| [docs/ACCEPTANCE.md](docs/ACCEPTANCE.md) | `make accept` 生成的汇总表（别手改） |
| [docs/DECISIONS.md](docs/DECISIONS.md) | 与 SPEC 的偏差与关键决定（**D1–D32**，含 verify 判据每一版为什么改、**D32 = 全向底盘巡航必须对正**） |
| [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) | 踩坑日志（§21–**§38**；含 S3 完整归因、costmap 两层不一致、试验台的静默坑） |
| **[docs/LIMITATIONS.md](docs/LIMITATIONS.md)** | **唯一权威**降级/未测清单（L1–L25，分四类） |
| [docs/DELIVERY.md](docs/DELIVERY.md) | 判据 → 支撑产物 → 复现命令三方对齐 |
| [docs/DEMO.md](docs/DEMO.md) | 演示脚本（四段，含"某行没出现"的现场排障表） |
| [docs/STATUS.md](docs/STATUS.md) | **每次收工的状态快照**（新会话先读它；末尾有交接摘要） |
| [docs/DATA_FORMATS.md](docs/DATA_FORMATS.md) | 产物格式；运行时禁读 `ground_truth/` 的依据 |

## 目录

```
SPEC.md                 需求源（唯一）
config/world.yaml       世界生成器的唯一配置源（改这个，别改 .sdf）
config/nav2_omni.yaml   Nav2 全向参数（MPPI Omni，PreferForward 权重 0）
config/slam_toolbox.yaml   建图参数
config/ekf_omni.yaml    S2 的 EKF（wheel+IMU）
config/rtabmap_stereo.yaml S3 的视觉里程计参数
depot_vln/
  layout.py            几何唯一来源（生成器与自检共用，含半深货位）
  sdf.py               SDF 发射器（名字前缀即实体类别）
  truth.py             确定性占用/类别真值（打分用，运行时禁止读）
  camera.py            双目深度 -> 点云（含 R_y(-pitch) 坐标变换）
  extrinsics.py        相机/雷达外参
  semantics.py / semantic_graph.py   货位状态库 / 语义图（Dijkstra）
  agent/               LangGraph 图：state / nlu / tools / nodes / graph
  mock/world.py        mock 工具层（基准不需要 Gazebo）
scripts/
  build_world.py / verify_world.py / make_floor_texture.py / apply_world_textures.py
  depot_geometry.py / make_static_map.py    地图只能从 SDF 静态碰撞体派生
  build_bench.py / eval_bench.py / check_graph.py / accept.py
  m1_velocity_tracking.py / m2_side_dock.py
  slam_route.py / slam_metrics.py / slam1_run.sh         SLAM-1 三配置
  seg1_bay_grid.py / seg1_capture_all.sh / seg1_go.py    SEG-1（累积点云）
  seg2_capture.py / seg2_detect.py                      SEG-2（双目深度 + 占用/类别）
  s3_smoke.sh / s3_check.py / s3_diag.py                S3 冒烟与归因诊断
  stereo_info.py / image_frame_relay.py                 S3 的 camera_info 与 frame 改写
  drive_to.py / capture_cloud*.py / gpu_watchdog.py
robot/depot_bot.urdf.xacro
launch/                  depot_sim / slam / slam_s2 / slam_s3 / nav2_omni
docs/                    见上表
```

## 三条硬边界（写在代码里，不靠自觉）

1. **只有 `act` 节点能调工具层**，工具层是唯一能碰 `/cmd_vel` 的地方 —— 这是图拓扑性质。
2. **`interrupt()` 只用来等一句人话**；机器人动 60 s 期间图是在轮询状态，不是挂起。
3. **每个环都有上限**：`recursion_limit=30`、`recover ≤ 2`、`clarify ≤ 2`；超限带原因码返回。

## 已知边界（摘要，权威清单是 [docs/LIMITATIONS.md](docs/LIMITATIONS.md)）

| 边界 | 实测数字 |
|---|---|
| verify 俯仰安全带 | **−4° ≤ pitch ≤ +3°**（两端都有真跑；俯侧单调：−5°→307、−6°→635、−7°→1019 px）|
| 位姿误差天花板 | 判据工作面 **|δ| ≤0.15 m 时 24/24**；+0.30→14/24；**实测漂移到过 0.53 m** → 那个量级只有约一半判对 |
| `unseen` 兜底粒度 | 真实触发 0/34；抹 98% 窗口深度才触发；**半坏不兜底**（50–95% 时仍给答案）|
| 绕行余量 | demo 0.74 m（物理不碰撞只需 0.49 m → 余量 51%）；**不可抬高 inflation 换余量**（会赔巷道宽度 + 需回归 M2）|
| 类别判定 | `cls` 一律留空（`carton_s`/`carton_m` 在 1.75 m 处像高只差 9 px，`D25`）|

## 与 SPEC 的偏差

只说值得知道的，全部记录在 [docs/DECISIONS.md](docs/DECISIONS.md)：最主要是占地 **67.0 × 40.0 m**
而不是 60×40 —— 24 bay × 2.4 m + 两端 4.5 m 横通道 + 2×0.2 m 墙，几何上就是 67.0 m。
保住了 bay 数（144 station / 576 slot）与巷道净宽 3.20 m，改的是总长。

## 许可

MIT，见 [LICENSE](LICENSE)。世界、纹理、真值均由 `make world` 程序化生成（无第三方素材）。
