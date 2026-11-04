# 数据格式

所有产物都是**生成出来的**（SPEC 方法论④「默认生成即错」）：改 `config/world.yaml` →
`make world`，不要手改 JSON/SDF。下面每一条都标了"谁生成 / 谁可以读"。

**读权限是硬边界**：

```
运行时允许读    world/slot_grid.json（货位几何）、config/*.yaml
运行时禁止读    ground_truth/       （只用于打分；agent 代码里出现这个路径就是 bug）
```

---

## 1. `world/slot_grid.json` —— 货位几何（运行时唯一几何来源）

生成：`scripts/build_world.py`　读：`depot_vln/semantic_graph.py`、`depot_vln/agent/*`、`scripts/check_graph.py`

```
schema            "depot-vln/slot_grid@1"
config            生成它的 config 文件名
seed              真值种子（改它会改 ground_truth）
footprint         {x_min, x_max, y_min, y_max, z_max}
derived           {rack_run, rack_total_y, footprint_x, footprint_y, aisle_clear_width,
                   rack_depth, cross_aisle_width, wall_thickness, block_pitch,
                   n_aisles, bays_per_aisle, n_bays, n_slots}
aisles[]          {name, index, center_y, y0, y1, x0, x1, width}
blocks[]          {name, index, center_y, y0, y1, x0, x1}          # 货架排（背靠背）
bays[]            {aisle, bay, x0, x1, center_x, aisle_center_y, slots[]}
slots{}           地址 -> slot 对象（见下）
address_format    "<Aisle>-<Side>-<Bay:02d>-<Level>"
side_convention   "面朝巷道前进方向(+X)：L=+Y，R=-Y"
```

单个 slot：

```json
"A-L-01-1": {
  "address": "A-L-01-1", "aisle": "A", "side": "L", "bay": 1, "level": 1,
  "pose":     {"x": -27.6, "y": -8.6,  "z": 0.45, "yaw": -1.570796},   # 货位中心（货架面内侧）
  "deck_z": 0.15, "clear_height": 1.6,
  "polygon":  [[-28.8,-9.15],[-26.4,-9.15],[-26.4,-8.05],[-28.8,-8.05]],  # bay 在地面的占地（XY）
  "obs_pose": {"x": -27.6, "y": -10.75, "z": 0.0, "yaw": 1.570796},      # 观测位（巷道中线，朝向货架）
  "obs_distance": 2.15                                                    # 观测位到货架面的距离
}
```

`obs_pose` 是 T2 拍帧的位置，也是 Nav2 侧"侧移入库位"（M2）的目标点 —— 两者必须是同一个点，
否则"看清了"和"能贴靠"会变成两套坐标。

---

## 2. `world/layout.json` —— 完整布局（墙/柱/地面标线/灯）

生成：`scripts/build_world.py`　读：`depot_vln/sdf.py`、`scripts/verify_world.py`

比 `slot_grid.json` 多出 `walls[] / columns[] / markings[] / lights[]`（都是**派生自同一份 config**）。
调几何时看这个文件，能直接对上山墙、承重柱、地面条码的实际坐标。体积大（约 500 KiB），
`.gitignore` 已排除 —— 它是 `make world` 的产物，不入库。

---

## 3. `ground_truth/slots.json` —— 每格真值（打分专用）

生成：`depot_vln/truth.py`（seed 20261026）　读：**只有打分脚本**

```
schema            "depot-vln/ground_truth_slots@1"
seed              20261026
stats             {n_slots: 576, occupied: 342, occupancy_rate: 0.5938, n_cargo: 342,
                   carton_faces: {"A-L": 22, ...}}      # 每面纸箱数，给"属性/关系"指令算期望答案
slots{}           地址 -> 真值
```

```json
"A-L-01-1": {"address": "A-L-01-1", "aisle": "A", "side": "L", "bay": 1, "level": 1,
             "occupied": true, "class": "carton_m",
             "reachable": true, "reason_code": null,
             "confidence": 1.0, "observed_at": 0.0, "source": "generator"}
```

`class` 只可能取 `config/world.yaml → semantics.classes` 里的键
（`carton_s / carton_m / carton_l / tote / pallet / drum`）—— 闭集，P5 先闭集后开集。

---

## 4. `ground_truth/aisles.json` —— 面级聚合 + 封堵区间

生成：同一份 `truth.py`　读：基准生成器、`check_graph.py`

```
blocked[]         {aisle, bay_from, bay_to, x0, x1, barrier_xs[], center_y, width, reason}
                  # 当前只有 F 巷 18–24（reason = "BLOCKED"），用来提供"不可达"用例
rack_faces{}      面 -> 类别计数，例 "A-L": {"carton_m": 11, "carton_s": 7, "pallet": 4, ...}
carton_totals{}   面 -> 纸箱总数，例 "A-L": 22      # "去纸箱最多的那排" 的期望答案
bays{}            扁 bay 键 -> {aisle, bay, center_x, occupied_slots, empty_slots, class_counts, reachable}
                  例 "A-01": {...}
```

注意 `bays` 的键是 `"A-01"` 这种**扁平键**，而 `slot_grid.json → bays[]` 是**数组**；
两者表示同一个东西，别混用。

---

## 5. `ground_truth/poses.csv` —— 真值轨迹（ATE/RPE）

生成：`scripts/record_gt_poses.py`（仿真运行时从 `/world/depot/pose/info` 抓，20 Hz，
**不用 `/odom`** —— 用 `/odom` 等于拿被测对象给自己打分）

```
t,x,y,z,qx,qy,qz,qw,vx,vy,wz        # t = sim time（秒）
```

入库策略：**不入库**（`.gitignore` 已排除），因为每次建图/跑路线都会重写。空表头是 `make world` 生成的模板。

---

## 6. `results/bench_spec.csv` —— 指令基准（输入）

生成：`scripts/build_bench.py`（期望答案从真值算出，不手写）

```
id,category,utterance,expected_behavior,expected_target,expected_reason_code,
clarify_answer,expected_after_clarify,notes
```

```
category           direct | attribute | ambiguous | unreachable
expected_behavior  reach | ask | refuse
expected_target    slot 地址（如 C-L-21-1）| 面（如 B-L）| 巷（如 C）| 空
expected_reason_code  BLOCKED | NO_SLOT | EMPTY_BAY | 空
clarify_answer     歧义用例里"用户第二轮会怎么回答"（用来验证 resume 后能到对的地方）
```

规模：69 条（direct 18 / attribute 18 / ambiguous 15 / unreachable 18），
`results/bench_spec.json` 是同一份的元信息（`schema / n / per_category / seed / columns`）。

---

## 7. `results/bench_<run>.csv` —— 基准执行结果（输出）

生成：`scripts/eval_bench.py`

```
run,id,category,utterance,expected_behavior,expected_target,expected_reason_code,
actual_behavior,actual_target,actual_reason_code,behavior_ok,silent_wrong,
clarify_rounds,questions,seq_monotonic,wall_s,sim_s,nodes,node_path,answer
```

关键列：

```
silent_wrong    歧义指令没问就猜了 —— LANG-1 的唯一硬指标，必须全 False
seq_monotonic   发给工具层的 seq 是否严格递增（过期结果要被丢弃）
nodes/node_path 图节点数与路径，例 understand>resolve>plan>act>verify>respond
sim_s           mock 世界里的"仿真时间"，和 wall_s 分开记（真实仿真时两者会差几十倍）
```

---

## 8. `transitions/<run>.jsonl` —— 状态转移（可回放）

生成：同一脚本，一条转移一行

```json
{"run": "20261026T192340", "id": "D01", "node": "resolve",
 "utterance": "去 C-L-21-1", "reason_code": "", "t_wall": 0.006}
```

用途：报告里"模型不许直发速度"的证据链 —— 能查每一次 `act` 之前经过了哪些节点、
有没有出现不该有的转移。按 run 分文件，`*.jsonl.tmp` 忽略。

---

## 9. `results/graph1.json` —— GRAPH-1 判据结果

生成：`scripts/check_graph.py`

```
criterion, grid_res_m, robot_radius_m, start,
nodes, edges, components, expected_components, blocked_segments,
n_compared, median_rel_err, p90_rel_err, mismatched, pass, samples[]
```

`expected_*` 来自**独立实现**（栅格 Dijkstra），不是被测图的自我声明 —— 这是这条判据能成立的前提。

---

## 10. `results/world_perf.log` —— 世界启动与 RTF

生成：`make perf`（`scripts/world_perf_probe.py`）

```
世界规模       visual / collision / light 三个计数（直接数 SDF 里的标签）
渲染线程就绪   秒；"未出现" 不代表失败 —— 无传感器的纯世界不会初始化渲染线程
开始步进       秒（/world/depot/stats 第一次有数据）← 真正的门槛指标
RTF            稳态实时因子 + iterations
```

最新一次：visual 1334 / collision 573 / light 7 → 开始步进 2.2 s，RTF 1.00。
