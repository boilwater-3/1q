---
Status: active
Last-reviewed: 2026-08-23
Authority: AR 设计权威入口
Answers: AR 模块是什么、和谁交互、设计文档怎么导航
---

# Airborne Radar 设计

`airborne_radar` 提供机载雷达探测、航迹维护、环境/干扰建模、战术决策、控制指令归约、Recording/Replay、
调试视图和生命周期事件。对外提供稳定 `ArSession` 门面；环境服务、信号流水线、控制器、战术决策、
控制归约和 mutable context 保持 internal。

AR 的心智模型是**物理流水线**：emission → echo → incident RF → front-end ledger → detection cell →
decision。一次输入、一次结果。普通调用方只看到“平台、目标、可选外部 RF”——发射/接收准备、前端账本、
检测单元账本和可靠性裕量是 AR 内部物理步骤，不形成 public token 或外部状态机。

## 决策扩展点

AR 的内部闭环是「分析本拍特征 → 改下一拍发射控制」，不是出完航迹就算完：

1. 信号链给出航迹、探测/关联质量、过门干扰观测。
2. `TacticalCoordinator` 做威胁分类（运动学特征；特征库接口存在但默认 Session 未接线），
   再出 LPI/ECCM 提案；`ControlReducer` 收成 `ArControlProfile`，下一成功发射时消费。
3. 唯一 public 决策缝是 `SubmitExternalDecision()`：调用方在下一次 `Step` 前整包替换
   下一拍 profile（绕过 hold/cooldown）。外部模块不替换内部对象；分类与内部 baseline
   每个成功周期仍持续计算。
4. `DecisionObservation` / `DecisionInputFrame` **不得**进入周期记录（规则 15f，已落地）。
   外部模块读产品航迹、干扰观测、本拍 `control_profile`。Recording 在覆盖被接受时写独立
   submit 事件；周期记录只留 `applied_decision_source` 与 cycle/batch。
   观测袋与 `DecisionInputFrame` public 头均已删除；`DecisionInputFrame` 移至
   `src/airborne_radar/decision/`（内部 TacticalCoordinator 输入，不外发）。

## 模块定位要点

1. `ArSessionConfig` 描述硬件、任务、安装指向、策略、环境条件五域配置：直接构造并逐字段赋值，或从
   `ArProfileConstants.h` 的预定义语义常量整域赋值到 `ArSessionConfig`。
2. `ArCycleInput` 提供绝对周期时间、单一世界坐标平台状态、目标和独立 interference frame；自然环境配置
   由 `ArSessionConfig.environment` 提供，运行期更新通过 `ArRuntimeConfigPatch` 提交。
3. `ArSession::StepWithResult()` 生产分层周期记录（规则 15）；`Step()` 只返回产品航迹。
   拒绝周期不复用上一帧。落地前 `Step()` 仍从扁平 `ArCycleResult.output_frame` 取出。
4. `ArPolicyConfig::tracking.enable_kalman_filter` 的 struct 默认值为 `false`（语义默认关闭滤波，需显式
   开启）。直接构造 `ArSessionConfig{}` 的消费者若依赖 Kalman 跟踪，必须显式置 `true`——这是
   2026-07-31 与 Builder 语义默认对齐时的有意行为变更。
5. STT 模式支持**指定航迹跟随指向**（方案 A）：外部通过 runtime patch 只指定目标
   （`designated_external_target_id`），波束指向由 AR 用自身航迹推导；指定航迹丢失/未确认时
   自动回退 TWS，回退状态经周期记录 / 调试视图 / 生命周期事件暴露。指向来源优先级与冻结
   契约见 boundaries.md「STT 指定航迹跟随与自动回退」；TWS/TAS 生效模式下 session 级波束
   按扫描表逐周期推进（扫描动画，见 boundaries.md「扫描动画接线」）；指定指令可带限时
   捕获窗口（`designation_duration_cycles`），窗口耗尽未捕获即作废（回到扫描，成因
   `kAcquisitionTimeout`，见 boundaries.md「限时锁定指令」）。
6. `Ar*` 是 AR 模块的 public API 前缀。`RadarEquations`、`radar_cross_section`、`ComposeRadarAttitudeDeg`
   等领域术语保留原名。历史上的 `Radar*` 模块前缀已一次性迁移到 `Ar*`，不保留 deprecated compat 层；
   新增 public primary 类型不得再使用 `Radar*` 作为模块所有权前缀。

## 文档导航

- 模块边界、非目标、dt_sec 反直觉差异、环境/RF 事实边界、输出/输入/失败语义、滤波后端选型、
  设计变更规则 → [boundaries.md](boundaries.md)
- 数据流图、Public API 边界、时序、输出/调试/归属边界与状态所有权 → [data-flow.md](data-flow.md)
- 算法登记表（配置映射/环境/扫描/统一物理探测链/关联/航迹/决策/控制归约/序列验证）、每算法的实现
  边界与反直觉点、刻意不实现的算法（EKF/UDKF/SRIF 在线自动切换、双 association 路径） →
  [algorithms.md](algorithms.md)

跨模块公共规则（public API 边界、条件五域配置、三层输出模型、运行期配置提交策略、证据优先开发模式等）
见 `docs/common/contract.md`。
