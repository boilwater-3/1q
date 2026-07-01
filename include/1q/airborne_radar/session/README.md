# AR Session 公开接口 （Ar* 命名）

> 本模块所有公开类型使用 `Ar*` 命名。

## 决策域
- ITacticalDecisionEngine.h — 决策引擎抽象接口（唯一的公共扩展点），含 TacticalDecisionResult、TargetCategory
- DecisionInputFrame.h — 单周期决策输入帧（关联/探测质量摘要 + 轨迹快照）
- DecisionSourceInfo.h — ECCM 干扰源摘要（JammingTechnique 枚举 + EccmSourceInfo）
- TrackStateSnapshot.h — 轨迹快照 DTO（位置/速度/加速度/RCS/航迹状态）
- ControlDirective.h — 决策层输出的控制意图类型

## 控制域
- [ArCommand](ArCommand.h) — 行为决策层下发的战术指令（类型 + 来源）
- [ArControlProfile](ArControlProfile.h) — 信号层下一周期生效的控制真值（LPI/ECCM 开关）

## 周期 IO
- [ArCycleInput](ArCycleInput.h) — 单周期输入
- [ArCycleResult](ArCycleResult.h) — 周期结果 + TrackOutputFrame

## 环境域
- [ArEnvironmentInput](ArEnvironmentInput.h) — 环境运行时输入 + 补丁 + 状态（三合一）

## 适配器
- [ArExternalInputAdapter](ArExternalInputAdapter.h) — 外部输入适配器（平台姿态 → 雷达坐标系）
- [ArExternalOutputAdapter](ArExternalOutputAdapter.h) — 外部输出适配器（雷达航迹 → 平台坐标系）
- [ArCycleInputAdapter](ArCycleInputAdapter.h) — 周期输入一步构造器
- [ArCycleOutputAdapter](ArCycleOutputAdapter.h) — 周期输出适配器（内部帧 → ECEF 输出帧）

## 会话
- [ArSession](ArSession.h) — 主会话（PIMPL，静态工厂 Create / CreateWithDecisionEngine）
- [ArReplaySession](ArReplaySession.h) — 回放会话
- [ArTraceSession](ArTraceSession.h) — 跟踪会话

## 基础类型
- [ArInputValidation](ArInputValidation.h) — 输入校验
- [ArOutputTypes](ArOutputTypes.h) — 输出类型（SignalCycleAbortReason、AssociationQualityMetrics 等）
- [ArSceneTypes](ArSceneTypes.h) — 场景类型（ArSceneTarget 等）

## 调试/观测
- [ArTrackLifecycleRecorder](ArTrackLifecycleRecorder.h) — 航迹生命周期记录器
- [ArTrackOutputDebugView](ArTrackOutputDebugView.h) — 调试视图
