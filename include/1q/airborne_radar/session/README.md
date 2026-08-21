# AR Session 公开接口 （Ar* 命名）

> 本模块所有公开类型使用 `Ar*` 命名。

## 决策域
- DecisionControlTypes.h — 决策观测、外部 profile 覆盖值、提交状态和控制来源；proposal/directive 抽象已收口为内部实现
- DecisionInputFrame.h — 单周期决策输入帧（关联/探测质量摘要 + 轨迹快照）
- TrackStateSnapshot.h — 轨迹快照 DTO（位置/速度/加速度/RCS/航迹状态）

## 控制域
- [ArCommand](ArCommand.h) — 行为决策层下发的战术指令（类型 + 来源）
- [ArControlProfile](ArControlProfile.h) — 信号层下一周期生效的控制真值（LPI/ECCM 开关）

## 周期 IO
- [ArCycleInput](ArCycleInput.h) — 单周期输入
- [ArCycleResult](ArCycleResult.h) — 周期结果 + TrackOutputFrame

## 环境域
- 环境配置已迁移至 config 目录，见 [ArEnvironmentConfig](../config/ArEnvironmentConfig.h)（环境场景配置由 `ArSessionConfig.environment` 聚合，运行期更新通过 `ArRuntimeConfigPatch` 提交）

## 适配器
- [ArPlatformInput](ArPlatformInput.h) — 平台 ECEF 位姿（CycleInput.platform）
- [ArRadarFrameTransform](ArRadarFrameTransform.h) — 平台锚点 ENU → 雷达局部系变换
- [ArExternalOutputAdapter](ArExternalOutputAdapter.h) — 外部输出适配器（雷达航迹 → 平台坐标系）
- [ArCycleOutputAdapter](ArCycleOutputAdapter.h) — 周期输出适配器（内部帧 → ECEF 输出帧）

## 会话
- [ArSession](ArSession.h) — 主会话（PIMPL，静态工厂 Create；通过 SubmitExternalDecision 提交步间 profile 覆盖值）
- [ArReplaySession](ArReplaySession.h) — 回放会话
- [ArTraceSession](ArTraceSession.h) — 跟踪会话

## 基础类型
- [ArInputValidation](ArInputValidation.h) — 输入校验
- [ArOutputTypes](ArOutputTypes.h) — 输出类型（SignalCycleAbortReason、AssociationQualityMetrics 等）
- [ArSceneTypes](ArSceneTypes.h) — 场景类型（ArSceneTarget 等）

## 调试/观测
- [ArTrackLifecycleRecorder](ArTrackLifecycleRecorder.h) — 航迹生命周期记录器
- [ArTrackOutputDebugView](ArTrackOutputDebugView.h) — 调试视图
