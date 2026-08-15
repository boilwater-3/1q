---
Status: draft
Date: 2026-08-15
Review-Baseline: `feature/remote-identification-radar-phase1` @ `63962f62`
Authority: 阶段 3 common 化收敛的逐项评估结论与执行立项（评估 + 执行计划入口）。
评估准则（2026-08-15 定案）：**物理恒等式优先收敛、装备私有账本/判决链不动**。
Related-Authority:
  - 迁移唯一状态文档：`remote_identification_radar_migration_status_2026-08-15.md`
  - 九项能力边界：`rir_signal_chain_capability_boundary_2026-08-15.md`
  - 执行计划：`common_consolidation_execution_plan_2026-08-15.md`
---

# AR/RIR common 化收敛逐项评估（阶段 3）

## 0. 定位与总结论

RIR 自持化与跟踪升级（阶段 1 / 2-M / 2-T / 2-S / 2-C / N1-N7）完成后，
AR 与 RIR 之间存在一批"同形副本"。本文件逐项判定每份副本的收敛归属，
为阶段 3 提供决策依据，并据此**立项执行 #1-#3 的 common 化收敛**。

总结论：

1. **立项收敛 #1-#3**：收敛动机不是"复用"，而是**消除双源分叉风险**
   （副本 + 注明来源 commit 的形态下，任一侧修改都会制造隐性分叉）。
2. 按"物理恒等式 vs 装备私有口径"分类：**LAPJV 求解器、雷达方程全集、
   天线方向图四个模型**属物理恒等式，列入收敛清单并**本次执行**；
   **检测单元账本、CFAR 判决链、传播/环境子集、航迹池**属装备私有或
   低收益，不动或缓。
3. 收敛执行采用 **一次性收敛到 `src/common/` + 两侧薄适配层**：
   AR/RIR 保留模块内函数名/类名，内部改为调用 common 单源；缺省行为
   逐位一致（等价回归）是收敛门。后续任何一侧修改共享物理恒等式时，
   直接修改 common，不再维护副本。

## 1. 判定准则

- **物理恒等式（收敛）**：纯函数、无状态、无装备语义；两模块对同一输入
  必须给出同一数值（数学事实，不随装备差异）。判据：副本除命名/配置类型
  外逐行同形，且语义差异只能来自"写错"而非"装备不同"。
- **装备私有口径（不动）**：承载装备差异（账本结构、判决门、增益偏置、
  ECCM 语义、聚合策略）。强行动收敛 = 制造假共性 + 双方都要加开关。
- 已有 common 先例佐证准则可行：`common/estimation/ImmFilter`（RIR 直接
  包装）、`KalmanPredictor/Updater`（两模块同源消费）、
  `1q/electromagnetics/RfScene` incident 功率（物理单源，RIR 只消费）。

## 2. 逐项评估

| # | 候选 | AR 位置 | RIR 位置 | 分类 | 结论 |
|---|---|---|---|---|---|
| 1 | LAPJV 指派求解器 | `signal/association/LapjvSolver.*` | `tracking/RirLapjvSolver.*` | 物理恒等式（纯算法，与雷达无关） | **收敛（最高优先）** |
| 2 | 雷达方程全集（回波/噪声/积累/误差/Pd/门限） | `signal/detection/RadarEquations.*` | `internal/RirRadarEquations.*` | 物理恒等式 + 配置类型适配 | **收敛** |
| 3 | 天线方向图 4 模型（高斯/抛物线/余弦幂/sinc² + 扫描损失/旁瓣/后瓣） | `signal/detection/AntennaPatternRuntime.h` | `dwell/RirAntennaPatternRuntime.h` | 物理恒等式 + 配置类型适配 | **收敛** |
| 4 | 检测单元求解 + 干扰时频聚合 | `signal/detection/ArDetectionCellResolver.*` | `dwell/RirDetectionCellResolver.*` | **装备账本** | **不动** |
| 5 | 统计级 CFAR 判决器 | `signal/detection/SignalDetector.*` | `dwell/RirSignalDetector.*` | **装备判决链** | **不动** |
| 6 | 传播/杂波模型子集 | `environment/PropagationModel.*` | `internal/RirPropagationModel.*` | 物理恒等式 + 场景配置适配 | **已执行（第二阶段）** |
| 7 | 航迹对象池 | `signal/tracking/BoostTrackPool.*` | `tracking/RirTrackPool.*` | 通用内存件（无装备语义） | 收敛候选，**暂缓** |
| 8 | KF/IMM 数值核 | common 已单源 | `RirTrackFilter`/`RirImmFilter` 包装 | 已收敛 | 无动作 |

### 2.1 收敛清单细则（#1-#3）

证据：

- LAPJV：两副本除命名空间/守卫/日志前缀外**逐行一致**（2026-08-15 diff
  归一化后差异仅 include 与注释；RIR 侧 46e495dd 注明副本来源）。
- 雷达方程：两版 9 个静态函数同形
  （`ComputeEchoPowerWithGain_dBW`/`ComputeEchoPower_dBW`/
  `ComputeThermalNoisePower_W`/`ComputeIntegrationGain`/
  `ComputeRangeErrorStdDev`/`ComputeAngleErrorStdDev`/
  `ComputeDetectionProbability`/`ComputeThreshold` 等）；差异仅
  `config::engineering::TransmitterConfig` vs `RirTransmitterConfig`
  类型耦合与 Swerling 枚举名。
- 方向图：217 vs 221 行近镜像；差异为配置类型与命名。

收敛形态建议：

1. 落点 `src/common/`（建议 `common/radar/` 或并入 `common/estimation/`
   邻域），**以标量参数签名**（float/double + 通用枚举）承载物理公式，
   不引入任何模块 config 类型——类型耦合是当前副本的最大成因。
2. AR/RIR 各保留**薄适配层**：本模块 config 拆参后调用 common 实现；
   模块内函数名可保持（调用方零改动）。
3. 收敛门（等价回归）：两侧各取一组既有单测输入，common 实现输出与
   收敛前副本**逐位一致**；不一致即存在既存分叉，须先登记再定基准。
4. 已知口径核对项（收敛时必须显式对账，不许静默取一侧）：
   - 噪声功率带宽口径：AR 检测单元用匹配滤波带宽，RIR 方程取发射
     `bandwidth_hz`（能力边界决策 4 遗留核对项）；
   - `ComputeEchoPowerWithGain_dBW` 在两版的增益叠加语义；
   - 方向图主瓣判定边界（旁瓣/后瓣电平闭合条件）。

### 2.2 不动清单细则（#4-#5）

- **检测单元/干扰聚合（#4）**：RIR 有四增益 dB 偏置（缺省 0 dB 等保守
  账本）且无 AR 的抗 RGPO 前沿减半（AR ECCM 语义）；账本结构
  （SINR 分项、单元口径）是装备规格。物理子件若同形（如脉压增益
  `max(1, B·τ)`）随 #2 收敛，账本本体不动。
- **CFAR 判决器（#5）**：同为统计级 CFAR（Pfa 门限 + Swerling Pd + 种子
  判决），但判决门限、硬截断、裕量门与种子管理是装备策略；收敛只会让
  两侧共享一个谁都不完全想要的判决链。
- **传播/杂波子集（#6）**：第二阶段已执行。数值内核收敛到
  `common/radar/VegetationClutterModel`；AR/RIR 只保留公开枚举到 common
  枚举的映射适配层，两侧公开配置与结果类型不变。

### 2.3 航迹池（#7）

通用内存复用件、无装备语义，理论上可收敛；但当前两实现各自稳定、
修改频率低，双源分叉风险小，**暂缓**——出现第三消费者或任一侧改动时
再收敛。

## 3. 执行机制

- **本次直接执行 #1-#3 的 common 化**，落点与步骤见
  `common_consolidation_execution_plan_2026-08-15.md`；不再等待“接触即收敛”。
- **第二阶段扩展收敛 #6**：传播/杂波模型落点 `common/radar/VegetationClutterModel`，
  量测误差与波束宽度解析同步收敛到 `common/radar`；AR/RIR 均保留薄适配层。
- 后续修改已收敛范围内的物理恒等式时，直接修改 `src/common/` 单源；
  两侧适配层保持薄封装，不再维护副本。
- 阶段 3 其余既有条目不变：`max_range_m`/`recognition_dwell_sec` 四域
  归位是 RIR 内部配置语义小项（识别 policy → mission 域），与 common 化
  无关，已执行（字段迁至 `RirMissionConfig`，控制器与校验同步）。
- 本评估结论落入两侧模块 boundaries.md 引用；不新增守护脚本，
  双源分叉风险由“单源 + 适配层”消除。

## 4. 验收（本次 common 化已完成）

1. 两侧对同一物理输入的输出逐位一致（等价回归绿）。
2. AR/RIR include 闭包互不引用（不变式保持）；common 实现无模块 config
   类型依赖。
3. 两侧模块 algorithms.md 登记表与 [evidence] 同步指向 common 单源。

执行完成记录见 `common_consolidation_execution_plan_2026-08-15.md`：
LAPJV / 雷达方程 / 天线方向图已落 `src/common/`，AR/RIR 薄适配层已切换，
相关单元/集成/契约测试全部通过。
