# ESR Session 公开接口

## 会话核心
- EsrSession.h — 主会话门面（PIMPL），提供 Step/StepWithResult/TryApplyRuntimeConfig
- EsrRecordingSession.h — Replay 记录包装会话
- EsrReplaySession.h — 回放会话（重放模式）

## 周期 IO
- EsrCycleInput.h — 单周期输入
- EsrCycleInputBuilder.h — 周期输入构造器
- EsrCycleResult.h — 周期结果 + 输出帧
- EsrCycleOutputBuilder.h — 周期输出构造器

## 环境域
- 环境配置已迁移至 config 目录，见 EsrEnvironmentConfig.h（环境场景配置由 `EsrSessionConfig.environment` 聚合，运行期更新通过 `EsrRuntimeConfigPatch` 提交）

## 场景与类型
- EsrOutputTypes.h — 输出类型（TruthAssociationRecord 等）

## 适配器

## 校验与调试
- EsrInputValidation.h — 输入校验
