---
Status: active
Last-reviewed: 2026-08-07
Authority: AR 设计权威入口
Answers: AR 模块是什么、和谁交互、设计文档怎么导航
---

# Airborne Radar 设计

`airborne_radar` 提供机载雷达探测、航迹维护、环境/干扰建模、战术决策、控制指令归约、trace/replay、
调试视图和生命周期事件。对外提供稳定 `ArSession` 门面；环境服务、信号流水线、控制器、战术决策、
控制归约和 mutable context 保持 internal。

AR 的心智模型是**物理流水线**：emission → echo → incident RF → front-end ledger → detection cell →
decision。一次输入、一次结果。普通调用方只看到“平台、目标、可选外部 RF”——发射/接收准备、前端账本、
检测单元账本和可靠性裕量是 AR 内部物理步骤，不形成 public token 或外部状态机。

## 决策扩展点

AR 的决策扩展点是同进程步间 observation/response seam：

1. 每个成功周期通过 `DecisionObservation` 输出 `DecisionInputFrame` 与实际控制 profile。
2. 调用方在下一次 `Step` 前运行外部模块，并用 `SubmitExternalDecision` 提交完整的 profile 覆盖值
   （整包替换 native 归约结果，绕过 hold/cooldown）。
3. 外部模块不替换内部对象；威胁分类和内部 baseline 每个成功周期仍持续计算，外部长期生效后内部
   baseline 仍能立即接管。
4. trace/replay 在外部覆盖被接受时立即写入独立 `decision_input` 事件，并在 `ArReplayRecord` 中固化
   observation、pending/applied internal proposals、来源 cycle/batch、reducer 计数和最终 profile。

## 模块定位要点

1. `ArSessionConfig` 描述硬件、任务、策略、环境四域配置：直接构造并逐字段赋值，或从
   `ArProfileConstants.h` 的预定义语义常量整域赋值，或经 `ArSessionConfigBuilder` 薄封装整域设置。
2. `ArCycleInput` 提供绝对周期时间、单一世界坐标平台状态、目标和独立 interference frame；自然环境配置
   由 `ArSessionConfig.environment` 提供，运行期更新通过 `ArRuntimeConfigPatch` 提交。
3. `ArSession::Step()` 获取本周期 track output，`StepWithResult()` 获取结构化执行结果；拒绝周期不复用
   上一帧。
4. `ArPolicyConfig::tracking.enable_kalman_filter` 的 struct 默认值为 `false`（语义默认关闭滤波，需显式
   开启）。直接构造 `ArSessionConfig{}` 的消费者若依赖 Kalman 跟踪，必须显式置 `true`——这是
   2026-07-31 与 Builder 语义默认对齐时的有意行为变更。
5. `Ar*` 是 AR 模块的 public API 前缀。`RadarEquations`、`radar_cross_section`、`ComposeRadarAttitudeDeg`
   等领域术语保留原名。历史上的 `Radar*` 模块前缀已一次性迁移到 `Ar*`，不保留 deprecated compat 层；
   新增 public primary 类型不得再使用 `Radar*` 作为模块所有权前缀。

## 文档导航

- 模块边界、非目标、dt_sec 反直觉差异、环境/RF 事实边界、输出/输入/失败语义、滤波后端选型、
  设计变更规则 → [boundaries.md](boundaries.md)
- 数据流图、Public API 边界、时序、输出/调试/归属边界与状态所有权 → [data-flow.md](data-flow.md)
- 算法登记表（配置映射/环境/扫描/统一物理探测链/关联/航迹/决策/控制归约/序列验证）、每算法的实现
  边界与反直觉点、刻意不实现的算法（EKF/UDKF/SRIF 在线自动切换、双 association 路径） →
  [algorithms.md](algorithms.md)

跨模块公共规则（public API 边界、四域配置、三层输出模型、运行期配置提交策略、证据优先开发模式等）
见 `docs/common/contract.md`。
