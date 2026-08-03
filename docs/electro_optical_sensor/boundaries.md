---
Status: active
Last-reviewed: 2026-08-03
Authority: EOS 模块级边界、非目标与设计变更规则
Answers: EOS 有哪些模块级禁令与边界、哪些非目标、frame_rate/dt 耦合与帧级 config 的特殊语义、文档变更规则
---

# EOS 模块边界

本文承载 EOS 的模块级边界、非目标、反直觉配置语义和设计变更规则。算法级边界（辐射传输、噪声·NEP、
空间频谱、杂散光等 foundation 算法）见 [algorithms.md](algorithms.md)。

## 与 common 契约的关系

EOS 遵守 `docs/common/contract.md`：

1. public API 只暴露稳定 session/config/input/output/trace/replay 门面。`EosSession` 是对外门面，
   只委托内部 `EosController`；Controller、Pipeline、CompositionRoot、foundation 算法不通过 public
   header 暴露。
2. `EosSessionConfigBuilder` 是薄封装（整域赋值 + `Build()` 返回副本）；语义档位是
   `EosProfileConstants.h` 中的预定义结构体常量（如 `profiles::kWideAreaSearchMission` +
   `profiles::kWideAreaSearchDetection`），不承担 leaf setter 或隐式 validation。旧"Mission Profile
   跨域覆写 `policy.detection.minimum_snr_db`"语义已消除：配置不再有隐式优先级，任何字段的赋值即
   最终决定（档位在前、微调在后时微调胜出）。
3. EOS 输出遵守三层模型：系统输出、结构化结果、调试视图分离。
4. `EosSession::StepWithResult` 在执行 pipeline 前调用 `ValidateEosCycleInput`；存在 error 级问题时
   不执行 pipeline、返回默认空帧并记录校验失败状态（符合 contract.md §实现安全与失败语义规则 3）。
5. EOS runtime config 属于先完整校验、后一次提交的原子语义；`EosRuntimeConfigResolver` 解析 patch，
   失败时不替换当前配置。

## dt_sec 校验边界（反直觉，勿按"五模块一致"补齐）

`ValidateEosCycleInput` 对 `dt_sec` 的校验链为：有限性 → 正值 → 上界 `dt_sec ≤ 10 / frame_rate_hz`
（默认 30 Hz → 上限 ≈ 0.333 s）。`frame_rate_hz` 从 pipeline 当前配置动态读取，runtime patch 热更新
帧率后校验阈值自动跟随。

该上界**故意仅适用于 EOS 与 SBIRS**——两者都是有 `frame_rate_hz` 概念的成像/凝视传感器。SAR/ESR/AR
**故意不含**此上界，因其配置无 `frame_rate` 字段、节拍由各自域量（孔径几何 / scan_rate×dt / PRF）
决定。不得为"五模块一致"给那些模块强加 frame_rate 上界。

[evidence: tests/unit/electro_optical_sensor/eos_input_validation_test]

## 帧级上下文 config 语义（反直觉）

`FrameContext` 是 EOS 帧级物理状态（光学孔径/FOV、波段、NEP/噪声、环境模型、工作模式、探测范围）
的权威容器，但它的几个语义容易误读：

1. **FOV 是记录成员门，不是 SNR 前置过滤器**：视场外目标不生成 detection/attribution；视场内但超出
   `dmin_m/dmax_m` 的目标仍计算通道 SNR、保留记录，最终 `detected=false`。范围只参与最终检测资格。
2. **扫描相位是否重置由 resolver 显式给出**：runtime patch 中的扫描速率或工作模式变化经 resolver
   校验后更新内部配置并决定是否重置扫描相位，不能由调用方隐式假设。
3. **环境 preset 不是 flat 参数**：`EosEnvironmentScenarioConfig` 只含一个 preset 和标准
   `atmospheric_physics` 观测；preset → 物理参数（辐射算法、气溶胶/湍流因子）的映射是设计内容，
   调用方不选择具体辐射算法，也不填写 custom 因子。replay 的 session-config payload 只记录
   `preset + atmospheric_physics`，内部派生数值不进入 schema，也不形成第二套可配置状态。
4. **硬件配置只保留当前生产链实际消费的字段**：波段、孔径、探测器和俯仰边界为 public/session/replay
   配置；焦距不再是 public/session/replay 配置。

[evidence: tests/unit/electro_optical_sensor/eos_pipeline_test]
[evidence: tests/unit/electro_optical_sensor/eos_runtime_config_resolver_test]
[evidence: tests/unit/electro_optical_sensor/eos_environment_model_test]

## 输入校验、失败输出与运行期状态

无效输入不会直接污染 pipeline 状态：

1. 首个周期输入无效时，返回默认空帧（`cycle_index=0`、空检测）。
2. 已有成功周期后再遇到无效输入，返回默认空帧（不复用），同时在 result 中记录校验失败状态。
3. 设备关机返回 `kSensorPoweredOff` 合法非执行状态；返回默认空帧，不得把关机映射为 `kOutputContractViolation`。
4. runtime patch 必须原子校验；任一字段无效时整个 patch 被拒绝。
5. **电源状态单源（COMMON-OQ-4 字段提升）**：`EosSessionConfig::sensor_enabled` 是唯一来源，整块
   mission patch 只更新扫描任务，不触碰电源；`has_sensor_enabled` 叶子是运行期电源唯一入口。
6. controller runtime state 支持 capture/restore，但必须拒绝不兼容的 pipeline snapshot 或其他
   controller 实例的 snapshot。

`ValidationCode` 仅保留 `EosCycleInput` 实际校验路径会触发的编码。环境观测字段（太阳辐照度、云量、
风速、背景温度、太阳角、昼夜类型）已迁入 `config::EosEnvironmentScenarioConfig`，不再属于周期输入域，
故不声明对应校验编码，以免误导调用方以为可以在 `CycleInput` 上校验这些字段。

`target.range_m` 的权威校验在 `EosInputValidation`（`<= 0` 为 error），controller 在校验失败时不执行
pipeline，故正常 Session 路径不会把非法 `range_m` 传入 pipeline。pipeline 内部的
`SafePositive(target.range_m, 1000.0f)` 仅为深度防御，兜底值 1000m 不构成合法输入约定。

[evidence: tests/unit/electro_optical_sensor/eos_input_validation_test]
[evidence: tests/unit/electro_optical_sensor/eos_controller_runtime_state_test]
[evidence: tests/contract/electro_optical_sensor/eos_public_api_convenience_test]
[evidence: tests/replay/electro_optical_sensor/eos_replay_session_test]

### 非执行周期统一不复用（五模块统一规则）

EOS 非执行周期（校验失败/关机/执行 abort）的 `Step()` 与 `EosCycleResult.output_frame` 一律返回**默认空帧**
（`cycle_index=0`、空检测），**永不复用**上一有效输出。调用方用 `StepWithResult().executed_this_cycle` /
`abort_reason` 判断周期状态。`reused_previous_output` 字段已删除。

[evidence: tests/contract/electro_optical_sensor/eos_public_api_convenience_test.cpp::StepReturnsEmptyFrameOnValidationFailureAfterSuccess]

## 专项序列验证边界

`batch_validation::electro_optical_sensor` 覆盖双目标焦面交叉、昼/黄昏/夜间、融合/红外/可见光
通道切换、扫描速率重定向、关机恢复和无效输入恢复。EOS 不因此声明跨周期目标跟踪身份。

影响退出码的硬检查只有：replay 完成、预期非执行周期数、failure marker 数、逐周期 attribution
完整、帧内 detection ID 唯一。

属于 warning/error 观测项（不影响退出码）：FOV/lifecycle 行为、非法 runtime patch 原子性、真正的
扫描扇区边界热更、通道 SNR 的昼夜物理趋势。batch 没有直接读取 lifecycle recorder，因此不得把
场景名扩大为这些内部状态的硬契约。场景 ID 与运行方式由 `examples/batch_validation/README.md` 维护。

## 非目标

1. 不暴露用户自定义 pipeline、controller、环境模型或 foundation algorithm 类型。
2. 不把仿真目标 ID/name 混入 `EosOutputFrame` 的 raw detection。
3. 不把 debug view、lifecycle 或 replay 当作真实传感器输出。
4. 不把环境 preset 简化为无语义 flat 参数；preset 到物理参数的映射是设计内容。
5. 不为测试 mock 便利新增 public 扩展点。
6. 不恢复旧"Mission Profile 跨域覆写 policy"的隐式优先级语义。

上述边界由文档结构守护和 public API 契约测试守护。

[evidence: tests/contract/check_docs_structure]
[evidence: tests/contract/electro_optical_sensor/eos_public_api_convenience_test]

## 设计变更规则

1. `EosOutputFrame`、`EosDetectionRecord`、`EosCycleResult` 或 attribution/debug/lifecycle 语义变化，
   必须同步本文档集和输出边界测试。
2. 环境 preset、radiative transfer model、aerosol/turbulence 默认值变化，必须同步 algorithms.md 和
   `eos_environment_model_test`、`eos_pipeline_test`。
3. runtime patch 的可变字段、原子性或 scan reset 规则变化，必须同步本文档集和 runtime resolver 测试。
4. foundation 算法如果从 internal 变成 public API，必须在 algorithms.md 的 `[evidence: ...]` 标注中
   记录扩展理由和兼容策略。
5. 新增 debug/replay 字段时，必须保持真实输出、结构化结果和仿真辅助视图三层分离。
6. 验证优先使用 `unit::electro_optical_sensor`、`contract::electro_optical_sensor`、
   `replay::electro_optical_sensor`、`batch_validation::electro_optical_sensor` 以及 EOS guards。
