# EOS Session 公开接口

## 会话核心
- EosSession.h — 主会话门面（PIMPL），提供 Step/StepWithResult/TryApplyRuntimeConfig
- EosTraceSession.h — 跟踪会话（录制模式）
- EosReplaySession.h — 回放会话（重放模式）

## 周期 IO
- EosCycleInput.h — 单周期输入
- EosCycleResult.h — 周期结果 + 输出帧
- EosCycleOutputAdapter.h — 周期输出适配器（内部帧 → 外部 ECEF）

## 环境域
- 环境配置已迁移至 config 目录，见 EosEnvironmentConfig.h（环境场景配置由 `EosSessionConfig.environment` 聚合，运行期更新通过 `EosRuntimeConfigPatch` 提交）

## 场景与类型
- EosSceneTypes.h — 场景类型
- EosOutputTypes.h — 输出类型（EosDetectionRecord 等）

## 适配器
- EosPlatformEcefPose.h — 平台 ECEF 位姿（CycleInput 字段派生与输出反算共用）
- EosExternalOutputAdapter.h — 外部输出适配器

## 校验与调试
- EosInputValidation.h — 输入校验
- EosOutputDebugView.h — 调试视图
- EosDetectionLifecycleRecorder.h — 检测生命周期记录器
