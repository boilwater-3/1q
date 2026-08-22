---
Status: active
Last-reviewed: 2026-08-23
Authority: SAR 模块级边界、非目标与设计变更规则
Answers: SAR 有哪些模块级禁令与边界、哪些非目标、配置/环境/校验的特殊语义、文档变更规则
---

# SAR 模块边界

本文承载 SAR 的模块级边界、非目标、反直觉配置语义和设计变更规则。算法级边界（RDA 不 fallback、
MoCo 阈值等）见 [algorithms.md](algorithms.md)。

## 与 common 契约的关系

SAR 遵守 `docs/common/contract.md`：

1. public API 只暴露稳定 session/config/input/output/Recording/Replay 门面。`SarSession` 是对外门面，
   只委托内部 `SarController`；Controller、ProcessingPipeline、CompositionRoot 不通过 public header 暴露。
2. 会话配置直接赋值 `SarSessionConfig`；语义档位是
   `SarProfileConstants.h` 中的预定义结构体常量（如 `profiles::kHighResolutionImagingMission`、
   `profiles::kL3BackprojectionProcessing`）。档位常量是完整子域
   结构体，整域赋值会重置未管理字段（如 `scene_center_*`、`l3_waypoints`），正确用法是"先赋档位、
   再设场景数据"。运行期热更新直接写 `SarRuntimeConfigPatch`（显式 `has_*`）；不提供 ConfigBuilder。
3. SAR 输出遵守两通道 + 可选投影模型：产品通道、信封通道、调试视图分离（旧称三层会话输出模型）。
   **勿与**下文「成像路径 L1/L1.5/L2/L3」混淆——后者是 RDA/动补/BP 成像档位，不是会话输出模型。
4. `SarSession::StepWithResult` 在运行期配置和成像链路前调用 `ValidateSarCycleInput`；存在 error 级
   问题时记录 `invalid_cycle_input` abort，返回默认空帧（不复用上一有效输出，符合 contract.md
   §实现安全与失败语义规则 3）。
5. SAR runtime config 属于立即提交；`SarController` 在每次 pipeline 执行前捕获 raw pulse、trajectory、
   pulse ID 和 PRF 分数余量，执行 abort 时恢复这些跨周期状态。配置不随执行失败回滚，
   执行状态也不得被失败周期污染。

### 非执行周期统一不复用（五模块统一规则）

SAR 非执行周期（校验失败/执行 abort/设备关机）的 `Step()` 与 `SarCycleResult.output_frame` **永不复用**
上一有效输出。调用方用 `StepWithResult().status` / `abort_reason` 判断周期状态。
`reused_previous_output` 字段已删除。

实现细节：校验失败路径返回严格默认空帧（`cycle_index=0`、空载荷）；pipeline 中止路径
（squint 成像门/SNR 门限/退化图像检测/状态恢复失败）在 `InitializeOutputFrameMetadata` 之后触发，因此
`output_frame.cycle_index == input.cycle_index`（元数据已写入但无有效成像产物——squint 门
在 raw echo 生成之前执行，拒绝帧亦无 raw echo 标记）；
设备关机路径（`sensor_enabled=false`）在管线入口短路，输出帧保持严格默认空帧。
三条路径均属"非执行"，区别仅在 `cycle_index` 来源——这是有意设计，不构成合约违反。

[evidence: tests/contract/sar/sar_public_api_convenience_test.cpp::StepReturnsEmptyFrameOnValidationFailureAfterSuccess]

## 电源状态单源（COMMON-OQ-4 字段提升）

`SarSessionConfig::sensor_enabled` 是电源唯一来源（mission 域无电源字段），
`SarRuntimeConfigPatch::has_sensor_enabled` 叶子是运行时电源唯一入口。运行期关闭
传感器不重建会话，只通过 patch 立即生效。关机时管线入口短路：`SarCycleStatus::kPoweredOff` +
`abort_reason=kSensorPoweredOff`，`status=kPoweredOff`（关机是合法非执行状态，
不是校验错误也不是执行失败），输出帧严格默认空帧，跨周期状态（raw pulse 缓冲、孔径拼接、
PRF 分数余量）不推进。

[evidence: tests/unit/sar/sar_session_pipeline_test::PoweredOffCycleShortCircuitsWithEmptyFrame]
[evidence: tests/unit/sar/sar_runtime_config_resolver_test::SensorEnabledLeafUpdatesConfig]

## dt_sec 校验边界（反直觉，勿按"四模块一致"补齐）

`ValidateSarCycleInput` 对 `dt_sec` 仅校验有限性 + 正值，**故意不含** EOS/SBIRS 的
`dt_sec ≤ 10/frame_rate_hz` 上界。

1. SAR 配置中无 `frame_rate_hz` 概念——其合成孔径时间由孔径几何（平台速度、方位分辨率、斜距）决定，
   而非成像帧率。
2. dt 的合理性由 PRF 分数余量、孔径拼接和跨周期 raw history 约束（见 data-flow.md）。
3. 该差异已由 `SarInputValidation.cpp` 的实测校验链与 `SarCycleInput` 无 frame_rate 字段共同固化。

不得为"四模块一致"给 SAR 加 frame_rate 上界。

[evidence: tests/unit/sar/sar_input_validation_test]

## Environment 几何、传播与地表背景契约

`SarEnvironmentConfig` 是 public 四域配置之一，当前五个字段均有确定语义。它只作用于 session
内部生成 raw echo；外部完整孔径 raw IQ 已位于外部生成器接收机之后，session 必须逐样本保留，
不得再次施加坐标转换、大气衰减或地表背景。

内部生成路径的几何转换：

SAR 是「场景目标平台锚点 ENU 输入契约」（docs/common/contract.md）的**文档化例外**：
孔径跨多脉冲、场景固定于地面，几何锚点是配置期确定的场景中心（非逐周期移动的平台），
故点目标输入保持 LLA（`SarPointTarget`），库内使用 scene-center 相对 ENU 几何。
外部完整 IQ 的 `pulse_states` 亦要求调用方直接填 scene-center 相对 ENU（见
`SarCycleInput.h`）；已删除无生产调用的 `SarCycleInputAdapter` /
`SarExternalInputAdapter`（原 ECEF/LLA→scene-center 脉冲便利层）。

1. 场景中心经纬度和 `terrain_reference_altitude_m` 组成局部原点，平台、点目标和 L3 航路点使用同一转换。
2. `use_flat_earth_geometry=true` 时，`x = R cos(lat0) Δlon`、`y = R Δlat`、
   `z = altitude - terrain_reference_altitude`。
3. `use_flat_earth_geometry=false` 时，经 WGS-84 LLA→ECEF→ENU；两条路径的原点和轴语义一致。
4. 非法 LLA 或转换失败必须结构化中止，不能静默回退到另一条几何路径。

内部回波的大气与地表模型：

1. 若 `enable_atmospheric_attenuation=true`，单程比损耗 `gamma_db_per_km` 对斜距 `r_m` 形成双程损耗，
   每个散射体的复振幅相应衰减；关闭时严格退化为 1（无衰减）。
2. 地表单元 RCS 由 `surface_backscatter_sigma0_db` 和期望地距/方位分辨率之积决定；当前低成本背景用
   场景中心周围确定性 3×3 代表性单元相干叠加到 raw IQ，其平均功率进入干扰/噪声账本，
   **不得计为点目标 signal power**。
3. 地形参考高程、大气比损耗和 sigma0 必须有限；大气比损耗还必须非负。replay roundtrip 保真全部
   environment 字段，但不改变上述来源边界。

[evidence: tests/unit/sar/sar_session_pipeline_test]
[evidence: tests/unit/sar/sar_session_config_builder_test]
[evidence: tests/replay/sar/sar_replay_codec_roundtrip_test]

## 专项序列验证边界

`batch_validation::sar` 使用多静态散射体和平台几何验证当前成像能力，不把 SAR 伪装成目标跟踪器。

六类序列覆盖：多散射点分辨、squint 门控恢复、raw/range-compression/L1 阶段切换、非法 runtime 组合
原子拒绝、无效输入恢复、低 SNR 恢复。

影响退出码的硬检查：
1. replay 输出数完整。
2. 预期非执行周期数。
3. failure marker 数。
4. 恢复后重新产图。
5. 特定序列的非法 runtime patch 原子拒绝。
6. range-compression-only 阶段。

属于 warning/error 观测项（不影响退出码）：completed stage 低于 L1、图像质量缺失、SNR 非有限、
熵非正、跨场景趋势。batch 没有直接读取 lifecycle recorder 或断言完整 ring-buffer 状态，因此不得把
场景名扩大为这些内部状态的硬契约。场景 ID 与运行方式由 `tests/consumer/batch_validation/README.md` 维护。

## 三层输出结构：L1/L1.5 分裂与诊断架构

### OutputFrame 的 trivially_copyable 约束

`SarOutputFrame` 是纯标量元数据（21 字段），受编译哨兵守护为 `trivially_copyable`。
聚焦图像（`SarFocusedImage`）和原始相位历史（`SarRawPhaseHistory`）包含 `std::vector`，
**结构性地无法放入 OutputFrame**。这不是偶然设计——它确保 OutputFrame 可零拷贝传递、序列化友好。

因此 SAR 的三层模型存在结构性 L1/L1.5 分裂：

| 层级 | 类型 | 内容 | trivially_copyable |
|---|---|---|---|
| L1 | `SarOutputFrame` | 产品元数据（处理阶段、网格尺寸、SNR、分辨率、熵、对比度、阶段标志） | ✅ |
| L1.5 | `SarFocusedImage` + `SarRawPhaseHistory` | 产品数据（复数图像矩阵、原始 I/Q 向量） | ❌ |
| L2 | `SarCycleResult` | L1 + L1.5 + 执行状态 + 诊断 | — |
| L3 | `SarProductDebugView` / `SarProductLifecycleRecorder` | 人读视图、生命周期事件 | — |

`Step()` 返回 L1 元数据；`StepWithResult()` 返回 L1 + L1.5 + 执行元数据。
L1 和 L1.5 共同构成"本周期的完整产品输出"。

[evidence: include/1q/sar/session/SarCycleResult.h — SarOutputFrame 结构定义]
[evidence: tests/unit/sar/sar_output_boundary_contract_test.cpp — trivially_copyable 编译期哨兵]

### 诊断架构：issues 为唯一诊断通道（统一问题列表模型，规则 14）

SAR 的 `SarIssueList issues` 承载统一问题列表（kInfo/kWarning/kError），每条包含
severity + phase + code + message + 可选定位（location/field）。本模块 code 全集单一事实
来源：`include/1q/sar/session/SarIssueCodes.h`（规则 14c）。`SarPipelineAbortReason`
枚举通过 `AbortReasonToDiagnosticCode()` 映射到诊断码字符串（如 `kSnrBelowMinimum` →
`"sar.snr_below_minimum"`），人读 message 由调用方提供。

**phase 来源标签**：输入/运行期配置校验问题（`ValidateSarCycleInput`、
`ValidateRuntimeConfigForStep`）→ `kInputValidation`，code 编码为 `"sar.validation.<snake>"`；
创建时配置校验（`ValidateSarSessionConfig`，`CreateWithDiagnostics` 出参）返回同一
`SarIssueList`（`phase = kInputValidation`、`severity = kError`、`field` 定位配置字段路径，
同条件 code 与运行期路径逐字一致，如 `"sar.validation.sample_window_too_small_for_pulse"`）；
执行诊断（pipeline 内联 kInfo/kWarning、`RecordAbort` 中止条目）→ `kExecution`。
校验拒绝路径不调用 `RecordAbort` —— 校验问题本身就是 error 级诊断（规则 9 写二），
abort_reason（写一）与日志（写三）在调用点补齐。

**运行期聚合码豁免（契约 14c 例外，2026-08）**：`ValidateRuntimeConfigForStep` 对硬件/任务
字段非法（`AreSarHardwareAndMissionFieldsValid` 失败）产出单一聚合码
`"sar.validation.invalid_config"`，而创建时 `ValidateSarSessionConfig` 对同一批字段产细分码
（`carrier_frequency_not_positive`、`bandwidth_not_positive` 等）。聚合是
`AreSarHardwareAndMissionFieldsValid` 返回 `bool` 签名（无出参 issue 列表）决定的必然结果；
运行期路径仅在 runtime config patch 后触发且 config 已在创建时校验过一次，聚合码不影响
拒绝语义。契约 14c"同条件 code 逐字一致"在此处为文档豁免：运行期硬件/任务字段失败统一
报告 `invalid_config`。

**周期级执行摘要日志（规则 13a）**：正常完成周期（`status == kCompleted`）在
`SarProcessingPipeline::RunCycle` 尾部输出 `[SarPipeline] cycle_index={} …` 的
`PROJECT_LOG_INFO` 摘要（周期号、完成处理阶段、L1/L3 成像标志、估计信噪比、场景目标数），
仅用于人读运行信息（规则 3），不参与状态判断。SAR 无逐目标门控排除（集体成像模型，
所有几何/SNR 门均为整周期中止 → 三写），故规则 13b 的按目标排除诊断对 SAR 为空洞条款
（SAR 的 kInfo/kWarning 正常路径诊断仍按本节"唯一诊断通道"承载；`SarIssueCause`
字段仅为五模块 `*Issue` 结构逐字同构保留，恒 `kNone`）。

### abort_reason 粗粒度枚举 + 细粒度诊断

`SarPipelineAbortReason` 是 `std::uint16_t` 底层类型的强类型枚举，包含 6 个粗粒度值，
与 AR/ESR/EOS/SBIRS 对齐：

| 值 | 语义 |
|---|---|
| `kNone` | 正常执行 |
| `kValidationRejected` | 输入/配置校验失败 |
| `kPipelineExecutionFailed` | 管线内部执行失败 |
| `kExternalInputRejected` | 外部原始 IQ 输入校验失败 |
| `kRuntimeStateRestoreRejected` | 运行时状态恢复失败 |
| `kSensorPoweredOff` | 设备关机：管线入口短路（COMMON-OQ-4 字段提升，见"电源状态单源"） |

细粒度失败信息由 `SarIssue::code`（如 `"sar.snr_below_minimum"`）和
`PROJECT_LOG_ERROR` 双写承载，不进入 public `abort_reason`。
`RecordAbort` 执行三写：粗粒度 `abort_reason` + 结构化诊断 + 人读日志；
校验拒绝路径由校验问题本身承载 error 级诊断（规则 9/14）。

[evidence: include/1q/sar/session/SarCycleResult.h — SarPipelineAbortReason 枚举定义]
[evidence: src/sar/session/SarDiagnosticUtils.cpp — WriteAbort 三写逻辑]

## 非目标

1. 不恢复旧会话工厂或旧文档树。
2. 不把 internal algorithm object 变成 public 替代入口。
3. 不把历史 evidence 文档重新常驻 `docs/sar/`。
4. 不为形式对称把 SAR 输入改造成其它模块的 external adapter 模型。
5. 不用测试阈值放宽替代模型、坐标、算法和契约问题的拆分。

上述边界由文档结构守护和 public API 契约测试守护。

[evidence: tests/contract/check_sar_doc_governance]
[evidence: tests/contract/check_public_api_boundary]

## 设计变更规则

1. public header、session/config/input/output 变化必须同步本文档集和 public API contract 测试。
2. pipeline、轨迹、raw history、聚焦路径或算法限制变化必须同步 algorithms.md。
3. 能力启用、否决或替代关系必须在 algorithms.md 的 `[evidence: ...]` 标注中记录依据。
4. 历史原因只保留摘要说明，不恢复被删除的旧审计文档目录。
5. 验证优先使用 `unit::sar`、`contract::sar`、`replay::sar`、`batch_validation::sar`、SAR guards
   以及 CTest `sar_cxx11_compat`（labels `compatibility;sar`）；当前没有独立的 `integration::sar` 分区。
