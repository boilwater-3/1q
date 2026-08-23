---
Status: final
Date: 2026-08-15
Completed: 2026-08-16
Authority: 阶段 3 common 化执行计划（LAPJV / 雷达方程 / 天线方向图）——已执行完毕，
  本文转为执行记录；公共接口现行为以 `src/common/` 实现与两侧 algorithms.md
  登记表为准。
Related-Authority:
  - 评估：`common_consolidation_assessment_2026-08-15.md`
  - 迁移唯一状态：`remote_identification_radar_migration_status_2026-08-15.md`
  - 阶段 3b 执行：`common_consolidation_execution_plan_2026-08-21.md`
---

# AR/RIR common 化执行记录（阶段 3 #1-#3，已完成）

## 0. 目标与范围

将 AR/RIR 之间的三组"同形副本"收敛到 `src/common/` 单源，两侧保留薄适配层，
模块内类名/函数名不变，调用方零改动：

| # | 候选 | AR 位置 | RIR 位置 | 落点（已建） |
|---|---|---|---|---|
| 1 | LAPJV 指派求解器 | `src/airborne_radar/signal/association/LapjvSolver.*` | `src/remote_identification_radar/tracking/RirLapjvSolver.*` | `src/common/optimization/LapjvSolver.{h,cpp}` |
| 2 | 雷达方程全集 | `src/airborne_radar/signal/detection/RadarEquations.*` | `src/remote_identification_radar/internal/RirRadarEquations.*` | `src/common/radar/RadarEquations.{h,cpp}` |
| 3 | 天线方向图 4 模型 | `src/airborne_radar/signal/detection/AntennaPatternRuntime.h` | `src/remote_identification_radar/dwell/RirAntennaPatternRuntime.h` | `src/common/radar/AntennaPatternRuntime.h`（header-only） |

接口约束（现行有效）：common 仅标量参数/通用枚举签名，不引用任何模块 config
类型——类型耦合是副本的最大成因；两侧适配层拆参后调用 common。

## 1. 完成记录

- LAPJV / 雷达方程 / 天线方向图均已在 `src/common/` 建单源，AR/RIR 改为薄适配层。
- 已收敛口径的对账决策（噪声带宽以调用方传入为准、增益叠加语义、主瓣判定边界、
  中性日志前缀、通用枚举由适配层转换）已随实现落地。
- 等价回归门达成：两侧对同一物理输入与收敛前逐位一致。
- 验证（2026-08-16）：
  - `unit::common` 138/138
  - `unit::airborne_radar` 585/585
  - `unit::remote_identification_radar` 115/115
  - `integration::remote_identification_radar` 29/29
  - `replay::remote_identification_radar` 3/3
  - `integration::cross_domain` 6/6
  - `contract::public_api` 7/7

实施前的接口设计细则、适配层改造步骤与风险核对项见 git 历史（本文件收口前版本）；
现行接口签名以 `src/common/` 头文件与两侧 `algorithms.md` 登记表为准。
