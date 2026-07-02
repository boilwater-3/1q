# src/ 架构与安全审查报告

Status: draft
Last-reviewed: 2026-07-02
Authority: proposed remediation plan for src architecture and safety issues

审查范围：`src/` 全部 6 个模块（约 6.85 万行）及 `include/`，重点覆盖共享基础设施（`common`）、5 个模块的 session/runtime 编排层，以及跨代码库的漏洞模式扫描。

本文是整改草案，不替代各模块 `design.md` 或 `docs/common/contract.md` 的当前权威内容。进入实现前，应按问题所属模块将最终规则并入对应权威文档，并补充测试或 contract guard。

验证进度：

- 2026-07-02：已用当前工作树核实 H1、H3、H4、H6、M3 的关键证据点。
- 2026-07-02：已修复 H3 的运行时接入缺口：`SarSession::StepWithResult` 现在调用 `ValidateSarCycleInput` 并在 error 级问题上 abort。
- 2026-07-02：已修复 H4 的 AR 环境快照一致性缺口：非默认 `environment` 数据配 `has_environment=false` 现在会触发输入校验 error。
- 2026-07-02：已修复 H6 的直接缺陷：`BlakeAtmosphericLoss` 的 `theta_deg` 现在进入低仰角等效路径因子。
- 2026-07-02：已修复 H5 的主要加固缺口：JSON 解析增加最大嵌套深度、尾随内容拒绝、`\uXXXX` 完整性校验和 surrogate 拒绝。
- 2026-07-02：已修复 M3 中 `NormalizeAngle180` 的 while 循环风险，改为 `std::fmod` 常数时间归一化。
- 2026-07-02：已修复 `TraceSink.cpp` 的 2 处异常和 M6 写入状态缺口。
- 2026-07-02：已修复 `ReplayTrace.cpp` 的 12 处异常路径：目录、manifest、chunk、index、failure/report 读写失败均改为无异常错误状态或空 reader。
- 2026-07-02：已修复 M5：`ReplayTrace` 读取 manifest 等整文件内容前会检查大小上限。
- 2026-07-02：已修复 `JsbsimAdapter.cpp` 的 3 处异常路径：加载失败和 RunIC 失败写入初始化诊断，`FlightManager` 进入 `kAborted` 而不是让异常穿透构造。
- 2026-07-02：已修复 M1 的 AR/ESR 校验拒绝语义不一致：AR validation reject 现在有结构化 abort reason，ESR validation reject 不再合成空帧或推进 batch。
- 2026-07-02：已修复 L2：`EosSession::Step` 不再使用逗号运算符串联执行与结果构造。
- 2026-07-02：已局部修复 L5：`src/common/atmosphere/AtmospherePhysics.h` 使用命名常量表达默认有效地球半径因子。
- 2026-07-02：已修复 M3 中 `SafePositive` 的重复定义：`NumericGuard.h` 复用 `ClampUtils.h` 的共享模板。
- 2026-07-02：已修复 M7 的内部 RCS 命名误导：新增语义化函数名并将生产调用切换过去，旧 REOS/SIMD 名称仅保留为兼容 wrapper。
- 2026-07-02：已推进 H2 第一批迁移：SAR 新增内部 `sar::extension::SarController` 与 `SarSessionCompositionRoot`，`SarSession` 退回对外门面。
- 2026-07-02：已推进 H2 第二批迁移：SAR 新增内部 `SarProcessingPipeline`，并补齐 controller/pipeline runtime state 快照与恢复边界。

## 高优先级问题

### H1. 17 处 `throw` 直接违反项目硬性约束

Status: fixed on 2026-07-02.

`CLAUDE.md` 明确约束“Never introduce C++ exceptions.”，但当前仍有下列异常路径。若项目以 `-fno-exceptions` 编译会直接失败；否则异常可能穿透构造函数或 I/O 调用并终止仿真流程。

| 文件 | 处数 | 性质 |
|---|---:|---|
| `src/common/replay/ReplayTrace.cpp:50,125,134,140,277,349,374,404,487,659,672,809` | 0 | 已改为无异常错误状态或空 reader |
| `src/flight_dynamic/adapter/JsbsimAdapter.cpp:186,214,271` | 0 | 已改为初始化诊断；`FlightManager` 失败时进入 `kAborted` |
| `src/common/trace/TraceSink.cpp:35,54` | 0 | 已改为无异常失败路径：打开失败/超大帧记录错误并返回 |

当前复扫结论：`src/` 与 `include/` 已无 `throw` / `std::runtime_error` / `<stdexcept>` 命中。`ReplayTrace` 仍保持原 public API，但内部 I/O 失败不再抛异常；`JsbsimAdapter` 保留构造入口并通过 `InitDiagnostics` 暴露失败原因。

### H2. SAR 缺少 Controller / CompositionRoot 抽象层

Status: fixed for controller/composition-root/pipeline migration on 2026-07-02.

修复前 `src/sar/session/SarSession.cpp` 的 `StepWithResult` 单方法约 90 行，同时承担调度、校验和成像数据流职责。该形态与 AR/ESR/EOS 的 Session -> Controller -> Pipeline 分层不一致。

已实施修复：

- 新增内部 `src/sar/runtime/SarController.h/.cpp`，承接 SAR 单周期调度、输入校验、运行期配置 gate、诊断策略、上一帧复用状态和 controller runtime state 快照。
- 新增内部 `src/sar/pipeline/SarProcessingPipeline.h/.cpp`，承接 raw history、waveform、L1/L3 imaging、退化图像检测，以及 raw pulse/trajectory 累积状态快照。
- 新增 `src/sar/session/SarSessionCompositionRoot.h/.cpp`，统一装配 `SarProcessingPipeline` 与 `SarController`。
- `SarSession::StepWithResult` 现在只委托 `controller.RunOnce(input)` 与 `controller.BuildCycleResult(input)`；`TryApplyRuntimeConfig` 也委托 controller。
- SAR public API 未暴露 Controller，`include/1q/sar/session/SarSession.h` 签名保持不变。

当前 H2 架构缺口已关闭：Session 不再直接编排成像链路，Controller 与 Pipeline 均有 owner/schema 快照校验。后续若需要，可单独设计“pipeline 自报失败后自动回滚”的策略，但这已不是本条 H2 的分层缺口。

### H3. SAR cycle 输入校验未接入 session 执行链

Status: fixed on 2026-07-02.

当前代码已有 `ValidateSarCycleInput` 及单元测试，但修复前 `src/sar/session/SarSession.cpp` 的 `StepWithResult` 只校验 `dt_sec <= 0` 与 runtime config，未调用该校验函数，导致 platform、point target、raw IQ pulse 状态的 error 级问题仍可进入成像链路。AR/ESR/EOS 均已有 session 层校验 gate。

风险示例：`point_targets[].radar_cross_section_dbsm` 为 NaN 时，`std::pow(10, NaN / 10)` 会污染回波矩阵，最终可能被退化图像检测兜底成“成像失败”，而不是暴露根因“输入非法”。

已实施修复：`SarSession::StepWithResult` 在运行时配置和成像链路前调用 `ValidateSarCycleInput`，若存在 error 级问题，则记录 `sar.invalid_cycle_input` abort，并按既有语义复用上一帧。

当前 `ValidateSarCycleInput` 覆盖：

- `dt_sec` 为有限正数。
- platform 位置、速度、姿态角字段有限。
- `point_targets` 的位置和 RCS 字段有限。
- 外部 raw IQ pulse 数量一致性、pulse 状态有限性和序列连续性。

剩余可选加固：角度范围、target 标识规则、可见性规则仍可继续补充，但原报告中“NaN RCS 污染成像链路”的核心风险已被阻断。

### H4. `has_xxx` 手动同步是系统性缺陷源

Status: fixed for AR cycle input on 2026-07-02.

`ApplySceneStateToCycleInput` 曾因漏设 `has_environment` 导致环境快照被写入但不被消费。`src/airborne_radar/model/ArCycleInputAdapter.cpp:13,40` 仍延续“flag 与数据冗余但必须手动同步”的模式。

一旦某条路径写入 `environment` 却忘置 flag，杂波、干扰和大气数据会完全不进入信号链，且没有报错、日志或 abort。

已实施修复：`ValidateArCycleInput` 现在区分两种情况：

- `has_environment=false` 且 `environment` 为默认构造值：视为省略环境快照，不校验默认字段。
- `has_environment=false` 但 `environment` 含非默认数据：报告 `kEnvironmentSnapshotFlagMismatch` error，避免环境事实被静默跳过。

剩余治理方向：长期仍可评估移除冗余 flag，由 presence 类型表达“是否提供环境快照”；短期核心静默跳过风险已由校验层阻断。

### H5. 自研 JSON 解析器缺少深度限制和 EOF 校验

Status: fixed on 2026-07-02.

`src/common/config/JsonReader.cpp` 存在外部配置解析加固缺口：

- `ParseObject` / `ParseArray` 递归调用 `Value`，无深度限制。
- `\uXXXX` 读取 4 位 hex 时未检查 EOF。
- 解析成功后未校验是否到达 EOF，形如 `{"a":1}garbage` 的输入会被接受。
- 不处理 surrogate pair，可能产出非法 UTF-8。

已实施修复：当前自研解析器保留，但已补充：

- 最大嵌套深度限制。
- 顶层 JSON value 后的 EOF 校验，拒绝尾随垃圾。
- `\uXXXX` 必须完整读取 4 个十六进制字符。
- 拒绝未实现的 UTF-16 surrogate escape，避免产出非法 UTF-8。

剩余可选加固：完整 surrogate pair 合成、数字语法更严格校验、以及长期替换为成熟 JSON 库仍可单独评估。

### H6. Blake 大气损耗丢弃仰角

Status: fixed on 2026-07-02.

`src/common/atmosphere/AtmospherePhysics.cpp:35-54` 中 Blake 大气损耗计算直接丢弃 `theta_deg`。Blake 大气损耗的关键是沿斜路径的仰角加权积分，低仰角目标穿越更多大气；当前实现会低估低空小仰角目标的大气损耗。`EvaluateAtmosphericPropagation` 仍传入仰角，但当前无实际效果。

已实施修复：当前简化模型使用受保护的 `1 / sin(elevation)` 低仰角等效路径因子，并限制最小仰角和最大放大倍数，避免 0 度附近数值爆炸。该修复不是完整 Blake 积分模型，但已消除 `theta_deg` 完全无效的问题。

## 中优先级问题

### M1. 冗余 bool 门控导致静默跳过

Status: fixed on 2026-07-02.

已核实并修复：

- `ArSession` / `ArController` 的 validation reject 现在返回 `SignalCycleAbortReason::kValidationRejected`，不再表现为 `kNone`。
- `EsrController` validation reject 仍返回 `EsrPipelineAbortReason::kValidationRejected`，但不再合成空输出帧，也不再推进 `next_batch_id`。
- AR/ESR/EOS 的拒绝语义现在对齐为：未执行、只在已有上一帧时报告 reuse、不把非法输入记作新的有效 batch。

兼容性说明：`SignalCycleAbortReason::kValidationRejected` 以显式数值 4 追加，保留已有 replay/trace 中 `kLifecycleUnavailable=1`、`kInvalidEnvironmentCycle=2`、`kRuntimePreparationFailed=3` 的数值语义。

### M2. `owned_ptr + 同对象裸引用` 双引用模式脆弱

Status: verified, deferred as non-local ownership refactor.

`ArSession.cpp` 与 `EsrSession.cpp` 同时持有 `std::unique_ptr<X> owned_x` 与 `X& x`。当前因 `Impl` 在堆上不移动而安全，但一旦后续重构让 `Impl` 可移动，裸引用会形成 use-after-free 风险。EOS 的 `Impl` 已只用 owned pointer，形态更简单。

建议：以 EOS 为模板重构 AR/ESR 的 `Impl` 成员持有方式。

### M3. 工具函数重复实现并已发生漂移

- `SafePositive` 曾在 `ClampUtils.h` 与 `NumericGuard.h` 各有一份；已修复为
  `NumericGuard.h` 复用 `ClampUtils.h` 的共享模板。
- `CurrentTimestampMs` 在 `TraceSink.cpp` 与 `ReplayTrace.cpp` 各有一份；已验证，待后续 trace/replay 公共时间工具收敛。
- `kNormFloor` / `kNumericFloor` 散落多处，阈值不一致；已验证，待后续按数值语义分层收敛。
- 角度归一化曾存在 while 循环版本与 `fmod` 版本；while 版本对极大输入可能近似死循环。已于 2026-07-02 将 `NormalizeAngle180` 改为 `std::fmod` 常数时间实现，并补充大输入回归测试。
- `MakeIssue` / `MakeLocation` / `IsFinite` / `IsRatioValid` 在 AR/ESR/EOS 输入校验中复制；已验证，待后续 common validation helper 收敛。
- `target_id=0` 严重级别在 AR/EOS/ESR 间不一致；已验证，因各模块 ID 语义不同，需先形成公共 contract 再统一。

建议：下沉到 `src/common/validation/` 与 `src/common/numerics/`，并用单元测试冻结共享语义。

### M4. 命名空间与目录映射混乱

Status: verified, deferred as namespace migration.

同一 `numerics/` 目录下并存 `oneq::internal::numerics` 与 `oneq::common::numerics`；coordinate 相关能力散落在 `oneq::coordinate`、`oneq::internal::coordinate_utils`、`oneq::internal::geometry`。

建议：按 `CLAUDE.md` 的 namespace-directory mapping 约束统一命名空间归属，并在迁移前先定义保留/迁移规则。

### M5. 外部 trace 文件读取无大小上限

Status: fixed on 2026-07-02.

`src/common/replay/ReplayTrace.cpp:485-490` 使用 `buffer << input.rdbuf()` 一次性读入整个 trace 文件，没有大小上限。写入侧已有 `0xFFFFFFFF` 守卫，读取侧缺失。

已实施修复：`ReadWholeFile` 在读取前使用 `seekg` / `tellg` 检查文件大小，并施加 `0xFFFFFFFF` 字节上限；超限时 reader 进入无异常失败状态。

### M6. TraceSink 写入后不检查流状态

Status: fixed on 2026-07-02.

`src/common/trace/TraceSink.cpp:66-68` 调用 `output_.write(...)` 后未检查 `fail()` / `bad()`。磁盘满或 I/O 错误时 trace 数据可能静默丢失，影响回放可信度。

已实施修复：`FlatbufferFileTraceSink::Record` 写入后检查 `fail()` / `bad()` 并记录错误；构造打开失败和超大帧路径也不再抛异常。

### M7. RCS public API 名泄漏不存在的 SIMD 细节

Status: fixed for internal RCS surface on 2026-07-02.

`src/common/rcs/RcsPhysics.h` 中部分函数名带 `_xmm4r4`、`_v128b_ps`、`_ymm8r4`、`_AVX` 后缀，但实现为标量。已新增语义化名称：

- `ComputeCylinderRcs`
- `ComputeBistaticCylinderRcs`
- `ComputePlanarPlateRcs`
- `InitializeTreeScatterer`
- `ComputeLeavesParametricEquation`

生产代码和单元测试已切换到语义化名称；旧 REOS/SIMD 风格名称仅保留为 inline 兼容 wrapper，避免破坏已有内部调用。

### M8. FlightManager 游离于统一 cycle 范式

Status: verified, deferred as flight_dynamic public-boundary migration.

`include/1q/flight_dynamic/FlightManager.h:168-172` 直接暴露 `GetAdapter()`、`GetAutopilot()`、`GetEngineManager()` 等内部子系统，并采用 `FlightManagerState` 枚举状态机，而不是其它模块的 cycle/result/abort-reason 协议。

建议：短期先文档化 flight_dynamic 的特殊边界；中期评估是否引入更统一的 session/cycle 外壳，避免公开内部子系统所有权。

## 低优先级与样式问题

- L1：已验证；`JsonReader.cpp` 的 `operator[]` 缺失时返回静态全局 `kNullValue` 引用。改为 `optional` 或指针会改变 `JsonValue` public API，本轮不混批。
- L2：已修复；`EosSession.cpp` 的 `Step` 现在显式先 `RunOnce`，再返回 `BuildCycleResult(input).output_frame`。
- L3：已验证；`GeometryTransform.h` 全量引入 `Eigen/Core`，可评估前向声明降低编译成本。
- L4：已验证；多处 `ptr.reset(new T)` 可逐步替换为 `std::make_unique`，纯样式，不应与语义修复混批。
- L5：局部已修复；`src/common/atmosphere/AtmospherePhysics.h` 已抽出
  `kDefaultEffectiveEarthRadiusFactor`。公开 `include/1q/environment/AtmosphericTypes.h`
  中的同值默认仍属更宽的 public-header 常量治理，不与本轮内部修复混批。
- L6：已验证；`AtmospherePhysics.cpp` 同函数并列 `tc_celsius` 与 `tk_kelvin`，调用方容易传错温标。该项涉及 REOS 对齐函数签名，需兼容迁移。

## 已确认的良好实践

- RAII 规范较好：裸 `new` 均由智能指针包裹，未发现裸 `delete`。
- 数学防护较完整：关键 `sqrt`、`pow`、`acos` / `asin` 路径普遍有 clamp、eps 或范数下界守卫。
- FlatBuffers 反序列化路径在 `GetRoot` 前先 Verify。
- 未发现 release-build 依赖 `assert` 丢失关键校验的模式。
- 未发现 `sprintf` / `strcpy` / `gets`、`const_cast` 或真正危险的 `reinterpret_cast` 使用。
- `StandardAtmosphere` 的 ISA 1976 分层实现与 `TimingRegimeModel` 的 CFAR 门限推导数值稳健，可作为后续数值模块范本。
- 三个雷达模块的 `RuntimeCycleState` 与 owner/schema 快照校验机制设计清晰，能防止跨实例误恢复。

## 建议整改顺序

| 阶段 | 任务 | 理由 |
|---|---|---|
| P0 | 修复 H1：消除 17 处 `throw` | 已修复 |
| P0 | 修复 H3 与 H4：SAR 输入校验、`has_xxx` 一致性断言 | 已修复 |
| P1 | 修复 H6：Blake 仰角生效 | 已修复 |
| P1 | 修复 H5：JSON 解析器加固或替换 | 已修复主要加固缺口 |
| P2 | 推进 H2：SAR Controller 层迁移 | 已完成 Controller/CompositionRoot/Pipeline 迁移 |
| P2 | 推进 M3 与 M4：剩余工具下沉、命名空间统一 | M3 已部分修复，M4 需迁移计划 |
| P3 | 处理 M2、M8、L1/L3/L4/L6 | 已验证，均不与本轮语义修复混批 |

## 总体结论

代码库工程质量整体较高，RAII、数值防护、FlatBuffers 校验和核心算法实现都有较好基础。当前最值得优先治理的是两类系统性风险：

1. 异常使用违反项目硬性约束，影响构建模型和失败语义一致性。
2. “数据存在但不被消费、错误发生但无信号”的静默跳过模式，会显著降低仿真结果的可解释性和可调试性。

本轮已优先处理会导致异常穿透、输入污染、静默跳过或错误结果解释的项；剩余 M2/M4/M8/L1/L3/L4/L6 属架构迁移、public API 迁移或纯样式治理，应进入独立批次并配套契约文档。H2 已完成 Controller/CompositionRoot/Pipeline 分层与快照边界迁移。
