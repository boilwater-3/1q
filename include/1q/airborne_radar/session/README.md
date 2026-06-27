# AR Session 公开接口

## 决策域
- ITacticalDecisionEngine.h — 决策引擎抽象接口（唯一的公共扩展点），含 TacticalDecisionResult、TargetCategory
- DecisionInputFrame.h — 单周期决策输入帧（关联/探测质量摘要 + 轨迹快照）
- DecisionSourceInfo.h — ECCM 干扰源摘要（JammingTechnique 枚举 + EccmSourceInfo）
- TrackStateSnapshot.h — 轨迹快照 DTO（位置/速度/加速度/RCS/航迹状态）
- ControlDirective.h — 决策层输出的控制意图类型

## 控制域
- RadarCommand.h — 行为决策层下发的战术指令（类型 + 来源）
- RadarControlProfile.h — 信号层下一周期生效的控制真值（LPI/ECCM 开关）

## 周期 IO
- RadarCycleInput.h — 单周期输入
- RadarCycleInputBuilder.h — 周期输入构造器
- RadarCycleOutputBuilder.h — 周期输出构造器
- RadarCycleResult.h — 周期结果 + TrackOutputFrame

## 环境域
- RadarEnvironmentInput.h — 环境运行时输入 + 补丁 + 状态（三合一）

## 适配器
- RadarExternalInputAdapter.h — 外部输入适配器（平台姿态 → 雷达坐标系）
- RadarExternalOutputAdapter.h — 外部输出适配器（雷达航迹 → 平台坐标系）

## 会话
- RadarSession.h — 主会话（PIMPL，静态工厂 Create / CreateWithDecisionEngine）
- RadarReplaySession.h — 回放会话
- RadarTraceSession.h — 跟踪会话

## 基础类型
- RadarInputValidation.h — 输入校验
- RadarOutputTypes.h — 输出类型（SignalCycleAbortReason、AssociationQualityMetrics 等）
- RadarSceneTypes.h — 场景类型（RadarSceneTarget 等）

## 调试/观测
- RadarTrackLifecycleRecorder.h — 航迹生命周期记录器
- RadarTrackOutputDebugView.h — 调试视图
