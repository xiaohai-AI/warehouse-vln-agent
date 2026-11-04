# 归档：重构前的过程文档（2026-10-26 ~ 2026-11-04）

这里的 9 份文件是重构前的**过程记录**，保留是为了追溯（尤其是失败归因），
**不是现状**。请以仓库根目录的 `README.md` 与 `docs/ARCHITECTURE.md` / `docs/LIMITATIONS.md` 为准。

| 文件 | 原用途 | 现在看它的理由 |
|---|---|---|
| `REPORT.md` | 技术报告：每个判据的数字 + 复跑命令 | 查"重构前实测过什么"（M1/M2/SLAM/SEG 的数字只在这里） |
| `TROUBLESHOOTING.md` | 踩坑日志（§1–§38） | 查某个现象当年是怎么定位的 |
| `DECISIONS.md` | 决策记录（D1–D32） | 查某个设计为什么是那样，包括后来被推翻的 |
| `STATUS.md` | 收工状态快照 | 查重构前的进度与 LLM 后端实测数据 |
| `PHASES.md` / `DEMO.md` / `DELIVERY.md` | 阶段闸门 / 演示脚本 / 判据-产物对齐 | 复跑某个 Gazebo 判据时的操作说明 |
| `ACCEPTANCE.md` | 16 条判据汇总（脚本生成） | 旧的"读日志对账"口径，已被 `tests/` 取代 |
| `DATA_FORMATS.md` | 产物格式 | `results/*.json` 的字段说明 |
| `PHASE_TODO_2026-10-30.md` | 阶段清单 | 同上 |

这些文档里的**数字**是当次快照，与当前代码不保证一致；其中关于"语义地图来自真值"、
"基准由模板生成"、"出生点写死 -31.0"的描述已经在重构中改掉，见
`docs/ARCHITECTURE.md` §7 与 `docs/LIMITATIONS.md`。
