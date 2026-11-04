depot-vln 阶段清单（现状 · 2026-10-30）

判据原文在 SPEC.md §10，实测数字在 docs/ACCEPTANCE.md（脚本生成），
完整报告在 docs/REPORT.md。这份文件只做"一眼看全 + 手上有活时查命令"。

────────────────────────────────
一、八个阶段，现在到哪
────────────────────────────────

P0 定义 —— 指令基准 + 评测骨架
  做什么：69 条取货指令（direct/attribute/ambiguous/unreachable 四类均衡），CSV schema，一条命令复跑。
  判据：LANG-1
  状态：✅ 69/69 = 100%，歧义静默猜错 0 次
  命令：make bench && make bench-run

P1 世界 —— 仓储世界生成器
  做什么：67×40 m（6 巷道 / 144 bay / 576 货位 / 342 占用），导出真值，硬闸门自检，控 RTF。
  判据：世界自检（22 项）+ 地图一致性（5 项）
  状态：✅ 净宽 3.200 m（144 站位 min=mean=max）、RTF 0.9925；
        A4b 挡轮护栏 0.32 m、G0/G0b 货位空间唯一都是硬闸门
  命令：make world + make map-check + make perf

P2 机器人 —— 麦克纳姆底盘 + 全向 Nav2
  做什么：各向异性轮摩擦、自己写里程计（Fortress 的 MecanumDrive 不发 odom）、
          Nav2 MPPI 全向参数、修入库点落在叉车体内的问题。
  判据：M1 速度跟踪、M2 侧移入库（落点 <0.10 m、5 次成功 ≥4）
  状态：✅ M1 4/4（真值 vx +0.1% / vy −0.1% / 斜行 0.0°）
        ✅ M2 5/5（真值 0.017/0.033/0.012/0.098/0.076 m，recoveries 0）
  命令：make m1-truth / make m2-static

P3 感知 —— 双目 + rgbd 深度标定
  做什么：三路相机桥接、同帧抓取、深度相对误差曲线（rgbd 当独立标尺）。
  判据：S1（中位 <5%、p95 <12%）
  状态：✅ 640×480：1.71% / 6.06%（302,058 px）；320×240：1.99% / 9.49%
        关键前提：世界必须带纹理（纯色世界 8.00% / p95 211%）
  命令：make cam-bridge && make s1-depth

P4 组合 SLAM —— 三配置对比（203 m 固定路线）
  做什么：同一路线同一建图后端，只换里程计来源：轮式 / 轮式+IMU / 双目（+IMU）。
  判据：SLAM-1（S3 的 ATE 要 ≤ 0.6×S1 = 0.214 m）
  状态：✅ S1 ATE 0.356 m / 地图 90.6%
        ✅ S2 ATE 0.332 m / 地图 89.8%（与 S1 在噪声内持平，+IMU 无显著改善）
        ❌ S3 ATE 32.150 m / 漂移 57.7 m / 地图 25.0%（D12 后复跑定版）—— 未达标
        ✅ S4 地面光流：ATE 0.288 m / 漂移 0.260 / 地图 90.9% —— 三项优于 S1（0.356/0.304/90.6%），
           但未达 SPEC 的 0.6×S1=0.214 线；分段还原 98.4–101.9%，侧移也测得到
        ✅ S5 地面光流+轮速 EKF：ATE **0.227 m** / 漂移 **0.159** / 地图 93.7%（五配置最好；里程计链 0.291 m，偏航 ≤0.21°）
           —— v2 修掉抖动（路径 154.8%→101.6%），v3 放大匹配窗 ±0.30 m；
           未达 SPEC 的 0.6×S1=0.214 线（差 9%），瓶颈见 docs/DECISIONS.md D15
           病因已结案：**沿巷道轴的视觉观测退化**（横向/转弯 95–107%，沿轴前进 0%）。
           五次配置变化都改不动（唯一地面纹理 / 稠密角点 / KLT 光流 / IMU 融合 / 速度 ×2.6）。
           机制：沿轴平移在图像上主要是径向缩放（2–10 m 处仅 1–3 px），而走廊沿轴近周期
           （bay 每 2.4 m 同构）→"前进一格"与"没动"在特征层等价。
           证据：docs/TROUBLESHOOTING.md §27 + §27.1
  命令：CFG=s1 make slam1 / CFG=s2 make slam1 / make slam1-s3；冒烟与归因：make s3-smoke

P5 语义 —— 从"有地图"到"知道哪一格放了什么"
  做什么：T1 从累积点云恢复 bay 网格；T2 观测位拍帧 + 双目深度判占用与类别；写成可查语义图。
  判据：SEG-1 / SEG-2 / GRAPH-1
  状态：✅ SEG-1 全库 6 巷 144/144 bay（中位 0.030 m、数量 0.0%、朝向 0.003°）
        ✅ SEG-2 precision/recall/类别 top-1 全 1.000，错格率 0%
           （只评 288 个下层货位；上层 288 个物理不可见，如实记"未评"）
        ✅ GRAPH-1 中位 7.2%（156 节点 / 158 边）
  命令：GROUP=1..3 make seg1-capture-all → make seg1；make seg2-robot && make seg2-capture && make seg2；make graph

P6 Agent —— 自然语言 → 动作
  做什么：LangGraph 图 + 工具层（查图/规划/导航/观察），同一份基准与 rule 基线对比。
  判据：LANG-1 + 分类别对比表
  状态：✅ rule/mock 基线 69/69（LANG-1 的记账口径）
        ⚠️ LLM 后端（3 个模型同口径实测，全部 FAIL）：
           qwen2.5:7b 50/69 静默猜 3 延迟 1.1 s/条 ｜ qwen3-vl:8b 52/69 静默猜 2 **延迟 34–56 s** 且 16/69 回退
           qwen3:latest 33/69 静默猜 8 ｜ 换模型救不了，需约束层；VLM 留给 verify 环节
           命令：BACKEND=ollama MODEL=qwen3-vl:8b make bench-run
        ⏸ 真仿真工具层（替换 MockToolLayer）未接
        命令：make bench-run / BACKEND=ollama make bench-run

P7 交付 —— 教材 + 演示 + 报告
  做什么：每个数字做成可复跑的一节；演示；结项报告（含所有失败）。
  状态：🟡 进行中 —— README、REPORT、PHASES、ACCEPTANCE、DECISIONS、TROUBLESHOOTING、
        STATUS、DATA_FORMATS 都已就位；剩演示材料与 ollama 对比表

────────────────────────────────
二、GPU 安全纪律（请当成习惯）
────────────────────────────────
1. 任何时候不需要仿真了：cd depot-vln && make down
2. 每天第一次起仿真前：journalctl -k -b 0 | grep -c "nvidia-modeset: ERROR" 必须是 0
3. 这台机器没有可用显示器：仿真一律 gui:=false（headless ≠ 不渲染，相机照样走 GPU）
4. 开相机渲染前后各跑一次 make gpu-observe（10 分钟窗口量化内核错误）
5. nav 档雷达 5 Hz，而 make preflight 的阈值是 6 Hz → 那一项 FAIL 是档位不匹配，
   不是故障（技术债，见 docs/REPORT.md §5.4）

────────────────────────────────
三、失败清单（详见 REPORT §5）
────────────────────────────────
· S3 双目视觉里程计未达标（沿轴观测退化）—— 已结案，证据链完整
· SEG-2 上层 288 个货位物理不可见（相机 0.436 m、层板 1.75 m）
· 局部 costmap 配置缺陷（voxel 订阅不存在的 scan3d、z 层数 40 超上限 16 → 局部避障为空）
· make preflight 在 nav 档必 FAIL（阈值与档位不匹配）
· AMCL（OmniMotionModel）比纯 odom 差（0.191 → 0.587 m），默认 LOC=static
· Nav2 启动偶发 rmw 超时（get_state 无响应），重试可恢复
· 仿真非实时：带双目 RTF≈0.44，所以所有采集速度按实测 RTF 定
