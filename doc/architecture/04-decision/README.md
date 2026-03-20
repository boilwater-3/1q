# 04 — 决策层

Decision 层是机载雷达仿真系统的战术大脑，负责在单个处理周期内完成 **目标分类 → LPI 控制 → ECCM 对抗**，并把 proposal 归并成下一周期生效的 `RadarControlProfile`。

## 模块设计概述

### 处理链路

```text
DecisionInputFrame
  -> TacticalCoordinator
     -> ThreatAssessmentEvaluator（威胁识别 + 跨周期记忆）
     -> EmissionControlEvaluator（LPI 控制建议）
     -> SurvivabilityEvaluator（ECCM 组合策略建议）
  -> ControlReducer（proposal 归并 + 域级状态管理 + 冲突裁决）
  -> RadarControlProfile
```

### 关键抽象

- **ITacticalDecisionEngine**（公共接口）：`Evaluate(DecisionInputFrame, TacticalStateStore&)` → `TacticalDecisionResult`
- **TacticalCoordinator**（默认实现）：编排三个 evaluator，支持 `IFeatureRepository` 注入做目标特征匹配
- **ITacticalEvaluator**（内部接口）：`Evaluate(DecisionInputFrame, TacticalStateStore&, TacticalEvaluationState&)`，各 evaluator 共享评估状态
- **TacticalStateStore**：跨周期战术记忆（威胁分数缓存、置信度缓存、域级 hold 计数器、分类标签、决策摘要）
- **ControlReducerConfig**（公共类型）：Reducer 可配置参数（LPI/ECCM 功率系数、hold/cooldown 周期数、冲突偏好）

### 输入契约

| 数据源 | 结构 | 用途 |
|--------|------|------|
| 环境层 | `EccmSourceInfo` + `EccmJammerSourceInfoList` | 多源干扰事实（强度、角域、频率重叠、PRF 锁定风险） |
| 信号层 | `AssociationQualityInfo` | 关联质量补位触发（欺骗/转发语义 + 严重度/压力门限） |
| 控制器 | `PerceptionQualityInfo` | 探测质量摘要（模式说明与原因摘要） |
| 信号层 | `DecisionTrackSnapshotList` | 当前活跃轨迹快照 |

### 三个关键边界

1. 环境层只提供"发生了什么"的事实，不做"怎么对抗"的策略选择。
2. 决策层只决定"启用哪些策略组合"，不直接修改探测器内部参数。
3. 信号层只负责执行控制真值，不反向决定是否启用 ECCM。

## 文件说明

| 文件 | 说明 |
|------|------|
| `decision-architecture.md` | **核心文档**：当前实现链路、ECCM/环境职责边界、策略叠加与信号层落点 |
| `decision-cycle-flow.puml` | Decision 周期处理链路流程图（PlantUML 源文件） |
| `decision-cycle-flow.png` | 流程图导出图 |
| `decision-architecture.puml` | Decision 模块架构图（PlantUML 源文件） |
| `decision-architecture.png` | 架构图导出图 |

## 建议阅读顺序

1. 先看 `decision-architecture.md` — 了解当前实现链路，以及 ECCM 与环境/信号层的职责边界。
2. 再看 `decision-architecture.puml` 或 PNG — 建立整体流水线的层次视图。
3. 然后看 `decision-cycle-flow.puml` 或 PNG — 补足单周期内策略生效的数据流动视图。
4. 需要落代码时，回到源码核对以下入口：
   - `include/.../decision/pipeline/ITacticalDecisionEngine.h` — 决策引擎公共接口与共享状态契约
   - `include/.../decision/pipeline/ControlReducerTypes.h` — Reducer 配置与归约结果
   - `src/.../decision/pipeline/TacticalCoordinator.h` — 默认 evaluator 编排
   - `src/.../decision/classifier/ThreatAssessmentEvaluator.h` — 威胁评估
   - `src/.../decision/lpi/EmissionControlEvaluator.h` — LPI 控制
   - `src/.../decision/eccm/SurvivabilityEvaluator.h` — ECCM 生存性评估
   - `src/.../decision/pipeline/ControlReducer.h` — 提案归并为控制真值

## 周期流程图预览

![Decision Cycle Flow](./decision-cycle-flow.png)

## 架构图预览

![Decision Architecture](./decision-architecture.png)
