# EOS Session 公开接口

## 会话核心
- EosSession.h — 主会话门面（PIMPL），提供 Step/StepWithResult/ApplyRuntimeConfig
- EosTraceSession.h — 跟踪会话（录制模式）
- EosReplaySession.h — 回放会话（重放模式）

## 周期 IO
- EosCycleInput.h — 单周期输入
- EosCycleInputBuilder.h — 周期输入构造器
- EosCycleResult.h — 周期结果 + 输出帧
- EosCycleOutputBuilder.h — 周期输出构造器

## 环境域
- EosEnvironmentInput.h — 环境运行时输入
- EosEnvironmentInputPatch.h — 环境输入补丁
- EosEnvironmentInputState.h — 环境输入状态

## 场景与类型
- EosSceneTypes.h — 场景类型
- EosOutputTypes.h — 输出类型（EosDetectionRecord 等）

## 适配器
- EosExternalInputAdapter.h — 外部输入适配器
- EosExternalOutputAdapter.h — 外部输出适配器

## 校验与调试
- EosInputValidation.h — 输入校验
- EosOutputDebugView.h — 调试视图
- EosDetectionLifecycleRecorder.h — 检测生命周期记录器
