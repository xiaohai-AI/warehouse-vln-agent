# 三个可录制的 GUI 演示

这三条覆盖项目的三层，录成视频/截图正好能讲清"感知 → 语义 → 语言导航"：

| # | 演示 | 看的是什么 | 典型时长 |
|---|---|---|---|
| 1 | **Nav2 导航** | ROS 2 Humble + Nav2 把麦克纳姆车开进货架巷 | 2–4 min |
| 2 | **自然语言指令** | LangGraph 把一句话变成 9 段航迹并开到货位 | 5–12 min |
| 3 | **SEG-2 感知采集** | 车在每个 bay 转 ±90° 拍照（语义库存的来源） | 5–15 min |

> 全程 Gazebo GUI 可见。录屏/截图建议放在 `docs/media/`（见
> [docs/media/README.md](media/README.md)）。

## 0. 每次开始前

```bash
cd ~/Project/depot-vln
echo "$DISPLAY"          # 必须非空；SSH 里没有 GUI
make down                # 清掉残留的 Gazebo / Nav2 / 桥
```

`make world` 必须先跑过一次（`world/warehouse.sdf` 与 `maps/depot_static.pgm` 是生成物）。
第一次录之前建议先 `make ci`，确认 61 条断言全绿。

### Gazebo GUI：跟着机器人

World 是 67×40 m，机器人只有 0.5 m——不跟拍的话画面里基本是个点：

1. 左侧 **World** 树里找到 `depot_bot`；
2. 右键 → **Follow**（再点一次取消跟随）；
3. 滚轮缩放、左键旋转、中键平移；`F11` 全屏。

### 跟随相机只看到一片灰？

世界 SDF 里**没有屋顶**（只有 floor / walls / columns / racks / cargo / forklift …），
灰色通常来自两件事：

1. **出生点离西墙只有 2.3 m**（spawn x=-31.05，西墙内表面 -33.4）。右键 Follow 会把相机
   退到西墙外，于是一片灰墙。
2. Follow 的距离太近，相机钻进了车体/俯视车顶。

按顺序试：

1. Follow 之后**滚轮向下拉远**几次；
2. World 树 → `walls` → `wall_west` → 右键 **Hide**（录完再 Show）；
3. 直接从 A 巷内出生，避开西墙：
   ```bash
   make down
   make sim SIM_ARGS="gui:=true x:=-27 y:=-10.75"
   # 另一个终端
   make nav-to X=-10 Y=-10.75
   ```
4. 不用 Follow：用 Orbit（左键旋转 / 中键平移 / 滚轮缩放）把相机放进西横通道，朝 +X 看。

### GPU / 独显

`launch/depot_sim.launch.py` 现在默认 `gpu_offload:=true`，会把 GUI + OGRE2 都赶到
NVIDIA（PRIME offload；同时设 GLX/EGL/Vulkan）。先确认驱动正常：

```bash
nvidia-smi -L
glxinfo -B | grep -E "OpenGL renderer|OpenGL vendor"   # 没装就 sudo apt install mesa-utils
```

如果强制独显反而起不来，退回核显/软件渲染：

```bash
make sim SIM_ARGS="gui:=true gpu_offload:=false"
# 或
make sim SIM_ARGS="gui:=false software_render:=true gpu_offload:=false"
```

### 录屏

最容易的是 **OBS Studio**：两个来源——Gazebo 窗口 + 终端窗口，画中画或左右分屏。
命令行党可以用 `ffmpeg`：

```bash
# 录整个屏幕（DISPLAY 通常是 :0 或 :1）
ffmpeg -f x11grab -video_size 1920x1080 -framerate 30 -i "${DISPLAY:-:0}.0" \
       -c:v libx264 -pix_fmt yuv420p docs/media/01_nav2.mp4 -y
```

转 GIF（控制体积）：

```bash
ffmpeg -i docs/media/01_nav2.mp4 \
       -vf "fps=12,scale=1280:-1:flags=lanczos" -loop 0 docs/media/01_nav2.gif -y
```

建议每段 GIF **≤10 MB、10–20 秒**；长视频放 YouTube/Bilibili，README 里放链接。
GitHub 单文件硬上限 100 MB，超过 50 MB 会警告。

---

## 1. Nav2 导航（2–4 分钟）

**终端 A** — 起仿真 + GUI：

```bash
cd ~/Project/depot-vln
make down
make sim SIM_ARGS="gui:=true"
```

等 Gazebo 窗口出来、左下角仿真时间在走（约 10–20 s）。

**终端 B** — 给 Nav2 一个目标（`nav-to` 会自动起 Nav2 栈，跑完再拆掉）：

```bash
cd ~/Project/depot-vln
make nav-to X=-20 Y=-10.75
```

预期终端：

```
[nav] Nav2 没在跑 → 起栈（静态图 + /scan 中继 + bringup）→ results/nav2_stack.log
[nav] Nav2 全部 active
...
[nav] 退出码 0
```

**画面**：车从西横通道拐进 A 巷，沿中线开到 `x=-20`（A 巷 y=-10.75）停住。
录 10–15 秒：起步、拐弯、进巷、停位。终端那一行 `Nav2 全部 active` 可以切进来做说明。

**收尾**：仿真还在跑；`make down` 拆掉。想保留 Nav2 栈继续看：`KEEP=1 make nav-to ...`。

> **如果 Nav2 显示 active 但车不动**：先看这三个日志——
> `tail -n 20 results/map_pub.log`、`tail -n 20 results/map_odom.log`
> 不能有 `ModuleNotFoundError`；`tail -n 30 results/nav2_run.log | grep -E "Invalid frame|extrapolation"`
> 必须为空。再 `ros2 topic echo /map --once` 确认 `/map` 有数据。

---

## 2. 自然语言指令端到端（5–12 分钟）

`p6-demo` 自己完成：重建机器人 → 起 Gazebo GUI → 起相机桥 → 跑 LangGraph → 评分 → 拆栈。
不需要第二个终端。

```bash
cd ~/Project/depot-vln
make down
GUI=1 RTF=0.30 make p6-demo UTT="去 B-L-07-2" VERIFY=depth
```

`VERIFY=depth` 用双目深度复核，不需要 Ollama；`GUI=1` 打开 Gazebo GUI；
`RTF=0.30` 是按"开 GUI 后算力被分走"折算的墙钟预算（如果还是超时，改 `RTF=0.22`）。

预期终端（节选）：

```
[p6] 起仿真（**Gazebo GUI**...）
[p6] 仿真就绪（/odom 有 1 个发布者...）
[p6] 起相机桥...
[act] seq=1 → B@07 观测位 (-13.20,-6.45) yaw +1.57 | 航段 9 段 / 22.8 m
[act]   航段 1/9 → W:C ...
...
节点轨迹   understand>resolve>plan>act>verify>respond
目标       B-L-07-2
原因码     OK
回答       已到达 B-L-07-2（走 22.8 m，行驶 ... s，核验 ... s）。现场核验：...
RESULT: PASS
```

**画面**：车从出生点出发，穿西横通道、进 B 巷、在 7 号位前停住并朝货架。
把终端里的「目标 / 回答 / RESULT: PASS」和 GUI 里的车剪在一起，就是一条完整的 VLN 故事。

**想录"歧义 → 反问"**（更体现 LangGraph）：

```bash
GUI=1 RTF=0.30 make p6-demo UTT="去 10 号位" ANSWER="C 巷左边第一层" VERIFY=depth
```

终端会先打印 `agent asks` 和 `user says`，再开始开；画面会到 C-L-10-1。

**收尾**：demo 默认跑完就拆栈；`KEEP=1` 可以保留现场。它会把机器人重建成 p6 的
320×240 相机档；录完用 `git checkout -- robot` 恢复仓库里的 SEG-2 档（640×480@10）。

---

## 3. SEG-2 感知采集（5–15 分钟）

这是语义库存的来源：车开到每个 bay 的观测位，左/右各转 90° 拍一帧 RGB-D。

**顺序很重要**：先 `make seg2-robot`（640×480 @10 Hz、俯仰 16°），再起仿真，
否则相机是 p6 的 320×240 档。

**终端 A**：

```bash
cd ~/Project/depot-vln
make down
make seg2-robot
make sim SIM_ARGS="gui:=true"
```

**终端 B**（等 GUI 起来后）：

```bash
cd ~/Project/depot-vln
make cam-bridge
```

**终端 C**：

```bash
cd ~/Project/depot-vln
rm -f results/seg2_frames_demo.npz        # 上次的成品会让脚本直接跳过
make seg2-capture AISLE=A BAYS=1-2 OUT=results/seg2_frames_demo.npz
```

`BAYS=1-2` = 4 个观测（2 bay × 左/右），足够录清"开过去 → 转 90° → 拍 → 转另一侧"的循环。
想多录两个 bay 就用 `BAYS=1-3`。

预期终端：

```
[seg2] 巷道 A（y=-10.75）bay 1..2 -> 4 个观测（每 bay 左/右各一帧）
[seg2] A 巷转场 ... 预算 ... s 墙钟
[seg2]  1/4  A-1-L  位姿 (...)
[seg2]  2/4  A-1-R  ...
[seg2] A 巷 4 帧 -> results/seg2_frames_demo.npz
```

**画面**：车沿 A 巷开到 1 号位，原地左转 90° 对着货架、停稳、拍；再右转 180° 拍另一侧；
然后开到 2 号位重复。GUI 里最清楚的是**车头转向货架**那一下。

**拍完顺手看检测结果**（终端，不需要 GUI）：

```bash
.venv/bin/python scripts/seg2_detect.py \
    --frames results/seg2_frames_demo.npz \
    --out results/seg2_demo.json --plot
```

4 帧样本太小，**很可能显示 FAIL**（4 帧里漏 1 帧就是 recall 0.75，低于 0.85 的阈值）——
这是样本量问题，不是检测回归。想放"真实数字"就用手上的 48 帧 A 巷：

```bash
make seg2 FRAMES=results/seg2_frames_A.npz
# 期望：precision 1.000 / recall 0.912 / top-1 0.935
```

**收尾**：

```bash
make down
# 可选清理（都被 .gitignore 忽略，不会进仓库）
rm -f results/seg2_frames_demo.npz results/seg2_demo.json results/seg2_det.png
```

`make seg2-robot` 生成的机器人 SDF/URDF 与仓库里已提交的一致（实测 byte-identical），
所以这个 demo 不会把 `robot/` 弄脏。

---

## Bonus（可选）：动态障碍 + 前向安全层

如果想把"车被横穿的推车挡住"也录进去：

```bash
make down
GUI=1 GUARD=1 DYN=1 RTF=0.30 \
  make p6-demo UTT="去 C-L-21-1" VERIFY=depth
```

推车在 C 巷 `x=-16` 横穿行驶线；`GUARD=1` 起前向安全层，终端会打印压住/放行。
这条依赖时序，可能要多跑一次；不算在三条主演示里。

---

## 录完怎么放进仓库

1. 文件放 `docs/media/`，命名 `01_nav2.gif`、`02_vln.gif`、`03_seg2_capture.gif`；
2. 在 [README.en.md](../README.en.md) 或 [README.md](../README.md) 的仿真演示段落里加：

```markdown
![Nav2 demo](docs/media/01_nav2.gif)
```

3. 每个 GIF 控制在 10 MB 内；长视频给外链（YouTube/Bilibili），不要直接塞仓库。
