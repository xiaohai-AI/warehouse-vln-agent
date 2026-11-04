# 交付说明：怎么验、怎么复现、每一条判据由谁支撑

> 规矩不变：**没跑过就是「未跑」，不写「通过」。** 判据的当前状态只看
> `docs/ACCEPTANCE.md`（`make accept` 生成，别手改）；降级/未测/限制只看
> `docs/LIMITATIONS.md`。本文只回答一个问题：**给你一个干净环境，怎么把 16 条判据重新跑出来。**

---

## 0. 三十秒自检（离线，不开仿真）

```bash
cd depot-vln
make accept          # 重算全部判据 → docs/ACCEPTANCE.md   约 25 s
make docs-check      # 文档与产物一致性 9 项             约 5 s
```

两条都必须 **退出码 0**。`make accept` 会顺带做两件"读代码就能算"的事（不需要仿真）：
P1 世界自检（22 项，读 `world/warehouse.sdf` + `world/layout.json`）与 P6-1 约束层离线自检
（34 项单元 + 对抗端到端，读 `tests/test_constraint.py`），并把日志写到
`results/p1_world.log`、`results/p61_selftest.log`。

想一口气跑完并检查产物是否齐：

```bash
bash scripts/delivery_check.sh
```

---

## 1. 完整复现（三条命令覆盖 16 条判据里的真跑部分）

| # | 命令 | 时长 | 覆盖 |
|---|---|---|---|
| 1 | `make accept` + `make docs-check` | ~30 s | P1、P6-1 自检、以及**全部判据的记账**（读 `results/` 下的产物）|
| 2 | `bash scripts/avoid_probe.sh` | ~2.5 min | 起仿真 + Nav2 栈，**Nav2 动态避障绕行**（P6-2f）|
| 3 | `NAV=1 UTT="去 C-L-08-1" VERIFY=depth POSE=odom make p6-demo` | ~6 min | 端到端：语言 → 语义图 → Nav2 导航 → 双目深度核验（P6-2、P6-2b）|

约束层的两条（P6-1）需要 ollama 在跑：

```bash
BACKEND=ollama make bench-run              # 裸模型对照组
BACKEND=ollama-constrained make bench-run  # 约束层实验组
```

> 全部真跑都**必须** `gui:=false`（这台机器没有显示器，且 `gui:=true` 有把 GPU 驱动挂死的
> 记录）。起仿真前先确认 `journalctl -k -b 0 | grep -c "nvidia-modeset: ERROR"` 为 0。

---

## 2. 判据 → 产物 → 复现命令（逐条对齐，文件名为准）

| 判据 | 支撑产物 | 复现命令 |
|---|---|---|
| P1 世界自检（22 项） | `results/p1_world.log`（现场生成）；输入 `world/warehouse.sdf`、`world/layout.json` | `make accept` 内含 |
| M1 速度跟踪 | `results/m1.json`、`results/m1_truth.json` | `make sim SIM_ARGS='gui:=false'` → `make m1-truth` → `make m1` |
| M2 侧移入库位 | `results/m2.json` | `make m2-static`（静态图 + Nav2，5 轮侧移）|
| S1 双目深度误差 | `results/s1_depth.json` | `make s1-depth` |
| SLAM-1 五配置对比 | `results/slam1.json`（S1）、`slam1_s2.json`、`slam1_s3.json`、`slam1_s4.json`、`slam1_s5.json` | `make slam1` / `slam1-s3` / `slam1-s4` / `slam1-s5` |
| GRAPH-1 图 vs 真值 | `results/graph1.json` | `make graph` |
| SEG-1 bay 网格发现 | `results/seg1.json` | `make seg1`（需先 `make seg1-capture-all`，要相机）|
| SEG-2 占用/类别 | `results/seg2.json` | `make seg2`（T2 流水线 + 打分）|
| LANG-1 指令基准（四类） | `results/bench_20261102.csv`（取最新 `bench_2*.csv`）| `make bench-run`（`BACKEND=rule`）|
| P6-1 约束层离线自检 | `results/p61_selftest.log`（现场生成）| `make accept` 内含 / `make p6-selftest` |
| P6-1 约束层真模型基准 | `results/bench_llm/bench_*_constrained.csv`（对照组取同目录最新 `bench_2*.csv`）| `BACKEND=ollama-constrained make bench-run` |
| P6-2 真工具层（真跑证据）| `results/p6_demo_*.json`（**64 份产物 = 28 份 dry + 36 份真跑**；判据只认 `dry=false` 的 36 份）| 第 3 条命令（`make p6-demo`）|
| P6-2b verify 占用判定 | 同 `p6_demo_*.json` 里 `verify=depth` 且 `cam_pitch_deg=0` 且判据版本为当前 `RULE_ID` 的记录 | `UTT="去 C-L-08-1" VERIFY=depth make p6-demo` |
| P6-2c verify 离线审计 | `results/depth_dumps/*.npz`（23 份）| `python3.10 scripts/verify_depth_audit.py` |
| P6-2d verify 俯仰容限 | `p6_demo_*.json` 里 `cam_pitch_deg≠0` 的注入实验 | `CAM_PITCH=-2 UTT="去 C-L-09-1" VERIFY=depth make p6-demo`（同理 +2、−5、−7）|
| P6-2e 动态障碍 | `results/p6_dyn_*.json`、`results/p6_dyn.log`、`results/p6_guard.log` | `GUARD=1 DYN=1 UTT="去 C-L-08-1" VERIFY=depth make p6-demo` |
| P6-2f Nav2 绕行（受控台）| `results/avoid_probe_*.json`、`results/probe_dyn.log` | `bash scripts/avoid_probe.sh` |

**注意 P6-2f 的口径**：只对**受控台**成立（障碍提前摆在路径上，偏出 0.66 m）。
demo 长巷道里车贴脸插入时仍停/等（偏出 0.25 m）—— 见 `REPORT §5.12b`、`LIMITATIONS L3`。

---

## 3. 目录：哪些是源码、哪些是生成物

```
depot_vln/          源码（agent、语义图、verify_depth、SDF 生成…）
scripts/            可执行入口（每一条判据一个脚本；p6_demo.sh / avoid_probe.sh 是编排层）
robot/ world/       URDF/xacro、世界 SDF 与 layout.json（**生成，不手改**）
config/             nav2_omni.yaml、world.yaml、slam 配置
maps/               静态占据图（make map 生成）
docs/               REPORT / DECISIONS / PHASES / STATUS / TROUBLESHOOTING /
                    ACCEPTANCE.md（生成）/ LIMITATIONS.md（唯一权威限制清单）/ 本文
results/            全部产物（JSON/CSV/日志/npz）；**判据只认这里的东西**
transitions/        agent 每次运行的节点轨迹（jsonl）
tests/              单元与对抗测试
```

**不许手改的**：`docs/ACCEPTANCE.md`（`make accept` 生成）、`world/*.sdf`、`robot/*.urdf`
（`make world` / `make robot` 生成）、`maps/*`。

---

## 4. 环境前提（缺一条都会出现"静默无效"而不是报错）

| 项 | 要求 | 为什么会静默失败 |
|---|---|---|
| ROS 2 | Humble | 脚本里 `source /opt/ros/humble/setup.bash`，别用 `PYTHONPATH=` 清掉它 |
| 仿真 | Ignition **Fortress 6.18** | 桥的消息前缀是 `ignition.msgs.*`；Jazzy 的 `gz.msgs.*` 会**静默 0 转发** |
| Python | 项目自带 `.venv`（含 14 个传递依赖）| `PYTHONNOUSERSITE=1`，缺依赖只报 `module not found` 的表象 |
| 显示 | **必须 headless**（`gui:=false`）| 这台机器没显示器；`gui:=true` 有把 GPU 驱动挂死的记录 |
| GPU | 起仿真前 `journalctl -k -b 0 \| grep -c "nvidia-modeset: ERROR"` = 0 | 非 0 还起仿真会加重挂死 |
| 收工 | `make down` | 残留 `gz sim` / `mecanum_odom` 会让下一轮"看起来起不来" |

---

## 5. 遇到问题先看哪里

| 现象 | 去看 |
|---|---|
| 栈起不来 / `READY` 之后目标被拒 | `TROUBLESHOOTING §35.3`（lifecycle 逐个激活）|
| "0 次事件"类结论（0 丢弃/0 重试/0 压住）| `TROUBLESHOOTING §34`（必须先证明进程活着）|
| verify 判错（空位报有货 / 有货报空）| `TROUBLESHOOTING §33`（货架是通透的）+ `DECISIONS D27/D28` |
| 桥转发 0 条消息 | `TROUBLESHOOTING §31`（消息前缀与 venv）|
| 脚本被莫名杀死 | `TROUBLESHOOTING §35.1`（名字含 `nav2_` 会被栈自己的 pkill 杀掉）|
| 想改判据阈值 | 先读 `DECISIONS D27/D28/D29`：窗口宽度、near 余量、俯仰容限都是**实测定版**的，不是可调参数 |

---

## 6. 交付边界（一句话版）

**判据面 16 条全部有真跑或离线证据（PASS 16 / FAIL 0 / 未跑 0），可复跑、可对账。**
四条能力边界必须原样告知使用方，最硬的是 **L1：视觉无法确认"是哪一格"**（画面无编号，
属世界模型问题）。完整清单：`docs/LIMITATIONS.md`（L1–L22）。
