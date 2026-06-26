# ESR Session 公开接口

## 会话核心
- EsrSession.h — 主会话门面（PIMPL），提供 Step/StepWithResult/ApplyRuntimeConfig
- EsrTraceSession.h — 跟踪会话（录制模式）
- EsrReplaySession.h — 回放会话（重放模式）

## 周期 IO
- EsrCycleInput.h — 单周期输入
- EsrCycleInputBuilder.h — 周期输入构造器
- EsrCycleResult.h — 周期结果 + 输出帧
- EsrCycleOutputBuilder.h — 周期输出构造器

## 环境域
- EsrEnvironmentInput.h — 环境运行时输入
- EsrEnvironmentInputPatch.h — 环境输入补丁
- EsrEnvironmentInputState.h — 环境输入状态

## 场景与类型
- EsrSceneTypes.h — 场景类型（发射机场景目标等）
- EsrOutputTypes.h — 输出类型（TruthAssociationRecord 等）

## 适配器
- EsrExternalInputAdapter.h — 外部输入适配器
- EsrExternalOutputAdapter.h — 外部输出适配器

## 校验与调试
- EsrInputValidation.h — 输入校验
- EsrOutputDebugView.h — 调试视图
- EsrEmitterLifecycleRecorder.h — 发射机生命周期记录器
