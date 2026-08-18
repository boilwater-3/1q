# AR Session 公开接口 （Ar* 命名）

> 本模块所有公开类型使用 `Ar*` 命名。

## 决策域
- DecisionControlTypes.h — 决策观测、外部 profile 覆盖值、提交状态和控制来源；proposal/directive 抽象已收口为内部实现
- DecisionInputFrame.h — 单周期决策输入帧（关联/探测质量摘要 + 轨迹快照）
- TrackStateSnapshot.h — 决策引擎 SPI 输入的轨迹状态快照（位置/速度/加速度/RCS/航迹状态；不作为发布产品）

## 控制域
- [ArCommand](ArCommand.h) — 行为决策层下发的战术指令（类型 + 来源）
- [ArControlProfile](ArControlProfile.h) — 信号层下一周期生效的控制真值（LPI/ECCM 开关）

## 周期 IO
- [ArCycleInput](ArCycleInput.h) — 单周期输入
- [ArCycleResult](ArCycleResult.h) — 周期结果 + ArDetectionOutputFrame（量测输出帧，TARGET-OQ-1 处置后传感器公开输出保持量测形态）
- [ArDetectionOutput](ArDetectionOutput.h) — 量测输出帧与量测记录（雷达局部 ENU 位置 + 量测协方差 + 检测裕量）

## 环境域
- 环境配置已迁移至 config 目录，见 [ArEnvironmentConfig](../config/ArEnvironmentConfig.h)（环境场景配置由 `ArSessionConfig.environment` 聚合，运行期更新通过 `ArRuntimeConfigPatch` 提交）

## 适配器
- [ArExternalInputAdapter](ArExternalInputAdapter.h) — 外部输入适配器（平台姿态 → 雷达坐标系）

## 会话
- [ArSession](ArSession.h) — 主会话（PIMPL，静态工厂 Create；通过 SubmitExternalDecision 提交步间 profile 覆盖值）
- [ArReplaySession](ArReplaySession.h) — 回放会话
- [ArTraceSession](ArTraceSession.h) — 跟踪会话

## 基础类型
- [ArInputValidation](ArInputValidation.h) — 输入校验
- [ArOutputTypes](ArOutputTypes.h) — 输出类型（SignalCycleAbortReason、AssociationQualityMetrics 等）
- [ArSceneTypes](ArSceneTypes.h) — 场景类型（ArSceneTarget 等）

## 调试/观测
- [ArTrackLifecycleRecorder](ArTrackLifecycleRecorder.h) — 航迹生命周期记录器（观测工具面；由 Session 自动注入内部轨迹快照驱动）
- [ArExclusionCauseRecorder](ArExclusionCauseRecorder.h) — 排除原因差分记录器
