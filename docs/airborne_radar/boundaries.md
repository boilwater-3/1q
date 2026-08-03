---
Status: active
Last-reviewed: 2026-08-03
Authority: AR 模块级边界、非目标与设计变更规则
Answers: AR 有哪些模块级禁令与边界、哪些非目标、配置/环境/校验/滤波的特殊语义、文档变更规则
---

# Airborne Radar 模块边界

本文承载 AR 的模块级边界、非目标、反直觉配置语义和设计变更规则。算法级边界（统一物理探测链的两
噪声基准、饱和降级、检测门限归属、欺骗 ECCM 路由等）见 [algorithms.md](algorithms.md)。

## 与 common 契约的关系

AR 遵守 `docs/common/contract.md`：

1. public API 只暴露稳定 session/config/input/output/trace/replay/decision DTO 门面。`EnvironmentService`、
   `SignalPipeline`、`ArController`、`MutableArContext`、tracking lifecycle 和战术决策部件不通过 public
   header 暴露。
2. `ArSessionConfigBuilder` 是薄封装（整域赋值 + `Build()` 返回副本）；语义档位是 `ArProfileConstants.h`
   中的预定义结构体常量，不承担 leaf setter 或隐式 validation。
3. AR 输出遵守三层模型：系统输出（`TrackOutputFrame`）、结构化执行结果（`ArCycleResult`）、调试/生命周期/
   replay 视图分离。
4. decision seam 是同进程步间 observation/response，是唯一的 public 决策扩展点。

## dt_sec 校验边界（反直觉，勿按"四模块一致"补齐）

`ValidateArCycleDeltaTime`（`double` 类型）对 `dt_sec` 仅校验有限性 + 正值，**故意不含** EOS/SBIRS 的
`dt_sec ≤ 10/frame_rate_hz` 上界。

1. AR 配置中没有 `frame_rate_hz` 概念——主动雷达的周期节拍由 PRF、驻留时间、航迹更新率等雷达域量在
   各自子链路里约束，而非成像帧率。
2. dt 的合理性由 PRI/rejitter、emission 调度和 signal pipeline 消费链把关，不适用一个全局 frame_rate 上界。
3. 该差异已由 `ArCycleInput.dt_sec` 为 `double`（其余模块 `float`）、AR 无 frame_rate 字段、本校验链实测
   三方共同固化。

不得为"四模块一致"给 AR 加 frame_rate 上界。

[evidence: src/airborne_radar/session/ArInputValidation.cpp]

## 环境/RF 事实边界

`EnvironmentScenarioConfig`（`ArSessionConfig.environment`）与 `EnvironmentSnapshot` 只承载自然环境事实：

1. **不包含** jammer、J/S、J/N、干扰检测布尔值或预计算接收功率。AR 的外部 RF 输入是独立的
   `oneq::electromagnetics::RfEmissionFrame`，通过 `ArCycleInput::interference` 直接传入。空 frame 表示无
   外部 RF；非空 frame 必须与 AR 周期号、绝对窗口起点和时长完全一致，否则整个周期拒绝。
2. AR **不从环境场景推导干扰**。调用方只提供实际发射事实（发射 platform/equipment/emission 身份、ECEF
   运动学、天线、极化和参数化波形）；接收功率、PSD、J/N、饱和和观测质量全部由 AR 当前接收机链计算。
3. 大气物理附加损耗由信号层 `ComputeTargetSpecificAtmosphericLossDb` 用**每个目标的真实几何**计算，
   不在环境层重复计算；环境层硬编码几何的死计算已移除，`EnvironmentSnapshot` 不再承载
   `atmospheric_physics_loss_db` 字段。
4. AR **不保留** legacy jammer DTO、技术类别、J/S 摘要、欺骗/转发适配层，也不直接获取 ECM 的
   `EcmDeceptionMode` 真值。外部欺骗发射与压制发射经同一 `RfEmissionFrame` 路径进入接收前端。

拒绝暴露 `SpaceWeatherContext`（年积日、F10.7、地磁 Ap、仿真 Unix 时间戳）：当前 GTD7 大气模型退化为
ISA 标准大气，这些字段全部未被消费，属未接入的死输入。仿真时间统一以 `ArCycleInput::cycle_start_time_s`
为唯一来源。

[evidence: tests/unit/airborne_radar/ar_environment_config_contract_test.cpp]
[evidence: tests/unit/airborne_radar/ar_environment_service_test.cpp]

## 输出/输入校验与失败行为

`ArSession` 和 `ArController` 都有明确的失败语义：

1. cycle input 校验失败时不执行 pipeline，`ArCycleResult` 携带 validation issues，controller 设置显式 abort
   reason `kValidationRejected`（保留 replay/trace 数值语义）。
2. **非执行周期统一不复用（五模块统一规则）**：`Step()` 与 `ArCycleResult.track_output_frame`
   返回默认空帧（`cycle_index==0`、空 tracks/emission），不论是否存在上一有效输出。调用方仅凭
   `Step()` 返回值即可判定本轮无新航迹。状态判断统一走 `StepWithResult().status`
   （`kRejectedInvalidInput`/`kPoweredOff`/`kRejectedExecution`）。
3. controller 内部 `last_cycle_reused_previous_output` 仅是 RF 接收/检测侧跨周期状态机的簿记标志
   （捕获/恢复 snapshot 用），不进入 public `ArCycleResult`，也不等于公开输出帧被复用；不得据此推断
   `Step()` 会回传历史航迹。
4. signal pipeline abort 时不会发布合成的最新输出。
5. 电源状态单源：`ArSessionConfig::sensor_enabled` 是唯一来源（mission 域无电源字段），运行时电源唯一入口
   为 `ArRuntimeConfigPatch::has_sensor_enabled`。
6. 设备关机是已接受的非执行配置边界：撤销周期副作用后 finalize 关机配置，并保留外部决策等待下一成功周期。

[evidence: tests/contract/airborne_radar/ar_public_api_convenience_test.cpp::RejectedCycleDoesNotReusePreviousOutput]
[evidence: tests/contract/airborne_radar/ar_public_api_convenience_test.cpp::StepReturnsEmptyCurrentFrameOnRejectedInput]

## 输出边界要求

1. track output 保持系统侧航迹语义；名称、仿真便利信息或调试归因不能替代 stable association key/status。
2. `ArCycleResult` 由 `ArSession` 汇总 controller、context 和 pipeline 状态，承载执行状态、validation issues、
   abort reason、submitted commands、control profile、association quality metrics、decision observation 和已采用
   来源 provenance；完整 proposal、待消费响应与 reducer 计数只属于内部 `ArReplayCycleRecord`，不进入 public
   业务结果。
3. query/debug/lifecycle/replay 是诊断辅助，不是用户扩展 signal pipeline 的入口；决策 SPI 不拥有输出结构，
   也不能绕过内部 output adapter 写系统输出。

## 滤波后端选型（人工配置为主，不做在线自动切换）

AR 使用标准 Joseph 形式 Kalman 滤波器（KF）作为生产后端。IMM 生命周期（`enable_imm_lifecycle`）是包裹 KF
的多模型融合层，**不是**独立后端。选型决策人工配置在先，理由三点：

1. **可复现性优先于智能性**。在线自动选型会使同一想定因阈值微调走不同后端，结果不可比。
2. **选型决策依赖外部真知**。"目标是否机动"等判据，仿真期真值已知，泄露到选型逻辑等同作弊。
3. **可解释性**。工程评审需能追溯到具体后端与参数。

明确不做：在线残差驱动的自动后端切换。EKF/SRIF/UDKF 的否决依据见 [algorithms.md](algorithms.md) 滤波后端
评估表；可接受的"智能"形态只有只读 NIS 诊断和基于任务剖面先验的 IMM 配置。

## 非目标

1. 不恢复宽 public customization surface。
2. 不把 `EnvironmentService`、`SignalPipeline`、`ArController`、`MutableArContext`、tracking lifecycle 或
   foundation 工程算法暴露为用户可替换 API。
3. 不把单一默认 association 路径包装成 public algorithm family；只有存在多个生产实现时，才通过受控配置
   暴露选择。
4. 不做在线残差驱动的自动滤波后端切换。
5. 不把测试 mock 便利接口升级为 public SPI。
6. 不让外部 decision engine 绕过内部 control reducer 和 command mapper。
7. 不把 debug/lifecycle/replay 字段混入系统输出语义。

上述边界由文档结构守护和 public API contract 测试守护。

[evidence: tests/contract/airborne_radar/ar_public_api_convenience_test.cpp]
[evidence: tests/contract/check_public_api_boundary.cmake]

## 设计变更规则

1. 新增、删除或改变 public SPI 时，必须同步本文档集、consumer tests 和 `ar_public_api_convenience_test`。
2. 任何新增 runtime patch 字段，必须明确是否影响 pipeline config、自然环境 scenario 或 RF operating state，
   并接入提交/回滚流程。
3. 探测路径如改变 RCS、大气、干扰、波束、SNR、检测概率或量测协方差语义，必须同步 algorithms.md 和相关
   signal/detection tests。
4. 数据关联和 lifecycle 行为变化，必须同步 association quality metrics、decision frame 说明和对应测试。
5. 战术决策或控制归约策略变化，必须补充 LPI/ECCM/ControlReducer 测试，并在 `[evidence: ...]` 标注中记录
   决策依据。
6. 输出字段变化必须保持 `TrackOutputFrame`、`ArCycleResult`、debug/lifecycle/replay 三层分离。
7. 验证优先使用 `unit::airborne_radar`、`contract::airborne_radar`、`replay::airborne_radar`、
   `batch_validation::airborne_radar`、`integration::cross_domain` 以及 AR guards。
