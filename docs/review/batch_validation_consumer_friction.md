# 批量验证框架消费侧困扰点观察

Status: draft
Last-reviewed: 2026-07-03
Authority: non-normative observations from batch validation framework consumption (revised)

本草案记录在构建并运行 `examples/batch_validation/`（92 个场景跨 AR/EOS/ESR/SAR 四模块的批量参数扫描框架）过程中，从**消费侧**观察到的 API、配置与可观测性困扰点。这些点不是 bug 报告，而是"接入体验"层面的非规定性观察，供模块演进时参考。

依据 `docs/common/contract.md` §文档结构，`docs/review/` 草案不得作为权威文档引用。本文条目推进到有结论时，应回写为契约规则（进 `contract.md`）、模块设计（进对应 `design.md`）或开放议题（进 `open_questions.md`），再从本文删除对应章节。

观察背景：框架仅消费对外公开的 Session / Adapter / Replay 接口与 `config_loader`，不触及任何 `src/` 内部实现。下列条目均为这一接入路径上实际遇到的返工点，已通过源码回读与 2026-07-03 本地运行复核；其中部分条目已被后续示例或文档缓解，本文按当前状态标注。

## 1. 回放分叉语义在 `divergence_found` 字段上失真

**现象**：ESR 近距离（10km）高功率场景回放失败时，`XxxReplaySessionResult` 三个字段呈现矛盾状态：

```
replay_ok=false   playback.divergence_found=false   first_error="ESR replay output divergence (EsrCycleResult)"
```

`ok=false` 且 `first_error` 文本明确含 "divergence"，但 `divergence_found` 为 `false`。

**根因**：`src/common/replay/ReplayTrace.cpp:1329` 处 `divergence_found` 只在通用比较路径（`actual_output_payload` 非空且与 inline 不符）置 true。各模块的 `ReplayXxxTrace` 走自定义比较路径，回调把 `actual_output_payload` 留空（源码注释 line 1326-1328 明确说明 "comparison handled by module"），改用 `*error` 字符串表达分叉。结果是：**模块级分叉让 `ok=false` + `first_error` 含 "divergence"，但 `divergence_found` 恒为 false**。

**影响**：消费方无法用结构化字段 `divergence_found` 判断是否发生分叉，只能解析 `first_error` 字符串。这与 `contract.md` 三层输出模型中"状态判断不得依赖解析日志文本"的原则冲突。四模块（AR/EOS/ESR/SAR）的 `ReplaySession.cpp` 共享这一模式（当前路径如 `src/airborne_radar/session/ArReplaySession.cpp:250`、`src/electronic_surveillance_radar/session/EsrReplaySession.cpp` 的 `OnCycleOutput` 同构）。

**复核状态（2026-07-03）**：本轮已修复。初版修复通过 `IsModuleOutputDivergenceError` 靠解析回调 `error` 文本中的 `"output divergence"` 子串判断分叉——这把文档自己批评的「解析日志文本判断状态」反模式搬进了框架核心，且模块分叉路径无视 `stop_on_first_divergence`、`compared_output_count` 计数口径与 generic 路径不一致。

**边界清理（2026-07-03）**：已彻底去除字符串匹配。`ReplayTraceOutputCallback` 返回类型从 `bool` 改为结构化枚举 `ReplayTraceOutputStatus`（`kHandledByModule` / `kDivergence` / `kOtherFailure`，见 `include/1q/replay/ReplayTrace.h`）；四模块 `OnCycleOutput` 显式返回分叉/一致/其他失败三态，删除 `src/common/replay/ReplayTrace.cpp` 中的 `IsModuleOutputDivergenceError`。`PlaybackReplayTrace` 统一两套分叉路径的 `stop_on_first_divergence` 语义与 `compared_output_count` 计数口径（`kOtherFailure` 不计入比较、不视为分叉）。新增 `ReplayTraceWriterTest.PlaybackOtherFailureIsNotDivergence` 与 `PlaybackModuleDivergenceRespectsStopOption` 覆盖这两个边界；`PlaybackMapsModuleHandledOutputDivergence` 断言改为结构化字段（`divergence_found` 为权威信号，`first_error` 仅透传人读文本）。

## 2. 示例配置 `sar.json` 通不过自身校验

**现象（历史）**：`examples/configs/sar.json` 默认参数（`sample_rate_hz=120e6` + `pulse_width_s=20e-6`）需 `ceil(20e-6 × 120e6)=2400` 个采样点，但 `range_sample_count=2048`。直接使用这组默认配置会触发 `kSampleWindowTooSmallForPulse` 校验失败；批量验证当时能跑通，是因为执行前覆盖了这组三元参数。

**根因**：校验约束 `ceil(pulse_width × sample_rate) ≤ range_sample_count`（`include/1q/sar/config/SarSessionConfigValidation.h:33,56`）被默认配置自身违反。

**修复方式**：`examples/configs/sar.json` 已改为与 `examples/sar/integration_demo.cpp` 同类的验证参数集（`sample_rate=1MHz`、`pulse_width=20us`、`range_sample_count=1024`、`PRF=100Hz`、`synthetic_aperture_time=10.24s`、`slant=100km`）。采样窗口需求为 20 个样本，小于 1024；方位脉冲数 1024 与 100Hz PRF 对应 10.24s 孔径时间。

**影响**：本轮修复后，按 `examples/configs/README.md` 指引加载 `sar.json` 不应再因采样窗口约束第一脚 abort。批量验证注释与 `docs/practice/batch_validation.md` 已同步删除"默认 sar.json 本身无效"的过期叙述。

**复核状态（2026-07-03）**：本轮已修复。`sar_batch_validation /tmp/1q/check_batch_validation_sar_fix` 以 `examples/configs/sar.json` 作为输入配置完成 10 个场景，`warning=0`、`error=0`、回放分叉 0。

## 3. AR 物理探测开关默认关闭，掩盖距离/RCS 趋势

**现象**：`airborne_radar.json` 默认 `enable_physics_detection=false` + `enable_physical_rcs=false`。关闭后雷达走确定性简化路径，所有目标被检出——距离 8km 与 120km、RCS 0.01 与 10 m² 输出**完全一致**。

**影响**：批量框架第一轮 AR 全部 52 场景指标一字不差，差点误判为框架损坏。回读 `detection_count` 才发现物理链路未启用。最终在框架内强制 `enable_physics_detection=true` + `enable_physical_rcs=true` 才让趋势显现。

**合理性**：该默认值对单元测试（要求确定性、可复现）合理。但对"想验证物理探测能力"的接入方是陷阱——JSON 文件本身无法写注释，必须在相邻配置说明中提示"观察距离/RCS 衰减需打开这两个开关"。

**复核状态（2026-07-03）**：本轮已按"物理真实性优先"修复。`examples/configs/airborne_radar.json` 现在默认启用 `hardware.enable_physics_detection` 与 `rcs_physics.enable_physical_rcs`，并将 `rcs_physics.physics_mix_ratio` 设为 `1.0`；`examples/configs/README.md` 已同步说明若消费方需要确定性的简化检出路径，应在本地配置中显式关闭这两个开关。

**后续边界**：该选择牺牲了部分示例确定性，换取默认配置直接体现距离衰减与 RCS 物理趋势；需要确定性路径的测试或 demo 应使用独立测试配置，而不是让用户入口默认隐藏物理链路。

## 4. 失败周期输出为默认零值，与"真实零"不可区分

**现象**：`ArCycleResult` / `EosCycleResult` / `EsrCycleResult` 头部注释均声明"若未执行则保持默认值"。于是 abort 周期的 `match_rate=0`、`fused_snr_db=0`、`confirmed_count=0`——与合法算出的零值无法区分。

**影响**：框架第一版把所有周期直接求均值，一个因几何退化 abort 的场景其 `match_rate=0` 拉低整体均值，趋势失真。被迫加 `executed_this_cycle` 门控、只统计稳态窗口才修正。

**与契约的张力**：`contract.md` 三层输出模型称 `StepWithResult()` 是"状态判断入口"，但失败时结构化数值字段与成功时同形（都是合法零），消费方必须先查 `executed_this_cycle` 才能安全使用任何数值。任何做统计/聚合的消费方若忽略此门控，会静默出错。

**复核状态（2026-07-03）**：本轮已补文档门控。`ArCycleResult` / `EosCycleResult` / `EsrCycleResult` / `SarCycleResult` 以及 `SignalCycleResult` 注释均明确：输出帧、诊断图像、控制真值和指标字段只有在 `executed_this_cycle=true` 时才代表本周期有效计算结果；失败/abort 周期默认值不能按真实零值参与统计。

**后续边界**：未改为 NaN，也未新增 `has_valid_metrics` 字段；现阶段以公开头文件约束消费方先检查 `executed_this_cycle`。

## 5. TraceSink 不可回放，普通 integration demo 缺少完整范例

**现象**：要让 trace 可回放，必须给 `*TraceSession` 传 `ReplayTraceWriter`，而非 `TraceSink`。两者产出格式完全不同：

- `FlatbufferFileTraceSink`（`include/1q/trace/TraceSink.h:45`）产出 `uint32_le length + FlexBuffers map` 单文件，调试用流，**不能**被 `ReplayXxxTrace` 回放。
- `ReplayTraceWriter`（`include/1q/replay/ReplayTrace.h:183`）产出 `manifest.json + events/*.jsonl + indexes/ + crash/` 目录，才是可回放格式。

**影响**：普通 integration demo 仍没有真正录制 + 回放的可运行片段，demo 里的 `enableTrace()` 只打印一行"实际工程应使用 TraceSession"的占位（如 `RadarModule.h:305`）。不过"唯一正确范例藏在单测里"这一旧判断已经过期：`docs/practice/batch_validation.md:136-158` 与 `examples/batch_validation/batch_replay.h:10-15` 已经明确说明 `ReplayTraceWriter` 与 `TraceSink` 的用途差异，并提供批量验证框架内的正确接入姿势。

**复核状态（2026-07-03）**：核心事实仍存在，但 `*TraceSession` 头文件提示本轮已补。`TraceSink.h`、`FlatbufferFileTraceSink` 与四个 `*TraceSessionOptions` 现在明确区分：`sink` 是调试/观测记录，不能直接回放；`replay_writer` 才产出可被 `ReplayXxxTrace()` 消费的 replay trace 目录。

**后续边界**：普通 integration demo 仍缺真正录制 + 回放的可运行片段；该部分可作为后续示例增强，不再是头文件入口完全无提示的问题。

## 6. 坐标输入"位置可 LLA、速度恒 ECEF"的非对称（示例已改用 helper，底层非对称未变）

**现象**：`ExternalKinematics`（`include/1q/coordinate/types.h:138`）允许 `position_frame=kLla` 填经纬度位置，但注释明确"速度固定为 ECEF 坐标系"。这仍是运行时约定，编译器不会把 LLA 位置与 ENU/ECEF 速度类型绑定起来。

**已变化点**：旧版观察中提到的 `examples/electro_optical/session_usage.cpp::MakeTargetLlaWithVelocity(lat, lon, alt, vx, vy, vz, ...)` 已不成立。当前函数参数为 `vel_east_mps / vel_north_mps / vel_up_mps`，并调用 `oneq::coordinate::TryEnuToEcefVelocity()` 写入 `target.kinematics.velocity_mps`。仓库也已有 `include/1q/coordinate/velocity_transform.h` 中的 ENU→ECEF 速度转换辅助。

**剩余影响**：底层结构仍是"位置 frame 可选、速度恒 ECEF"的非对称设计，直接填 `ExternalKinematics` 的消费方仍可能跳过 helper 而填错速度系。该条不再属于"API 主动误导"最高档，更适合归入"运行时 tag / 坐标约定需显著提示"。

**复核状态（2026-07-03）**：提示已补强。`include/1q/coordinate/types.h` 现在明确说明 `velocity_mps` 始终为 ECEF，即使 `position_frame==kLla` 也不能直接填 ENU/NED；AR/EOS/ESR 目标输入注释也同步说明 LLA 位置输入不改变速度坐标系。

## 7. 相邻 Pose 结构体字段不一致且无解释

**现象**：`ArExternalPoseInput` 含 `radar_mount_angles_deg`（雷达相对机身安装角），`EosExternalPoseInput` / `EsrExternalPoseInput` 只有 `platform_attitude_deg`，无 mount 字段。

**合理性**：该不对称有合理原因（AR 需复合雷达安装角建立雷达本地系；光电/电子侦察视轴与机身对齐更自然）。

**影响**：三个结构体并排放置，无任何注释说明"为何 AR 多一个字段"。接入者读到 EOS/ESR 的 Pose 时易怀疑漏字段，需翻 adapter 实现才确认是设计如此。

**复核状态（2026-07-03）**：本轮已补跨模块说明。`ArExternalPoseInput` 注释说明 AR 需要 `radar_mount_angles_deg` 来复合平台姿态与雷达安装角；`EosExternalPoseInput` / `EsrExternalPoseInput` 注释说明二者无独立 mount 字段，是因为示例适配器按传感器视轴与机体系对齐处理。

**后续边界**：这仍是三个模块 Pose 结构的真实差异，但已不再需要翻 adapter 实现才能理解原因。

## 8. 决策层每周期 info 日志无法被消费侧静音

**现象（历史）**：AR 的 `TacticalCoordinator`、`ThreatAssessmentEvaluator`、`LpiEvaluator` 在每次 `StepWithResult` 都向 spdlog 打 `info` 级日志（"Environment is clear"、"Target[x] -> Classification: UNKNOWN"）。50 周期 × 多场景，stdout 被数万行日志淹没。

**张力**：`CLAUDE.md` 工程约束明确"高速仿真热点路径不打日志"，但决策层该路径每周期都打。更麻烦的是日志走 spdlog 全局配置，**消费侧程序无法单独静音某模块日志**，只能整体重定向，连同消费侧自身进度输出一起被冲掉。

**复核状态（2026-07-03）**：本轮已修复默认刷屏问题。`ThreatAssessmentEvaluator` 的逐目标分类日志、`LpiEvaluator` 的每周期 LPI 路径日志、`TacticalCoordinator` 的 jamming/clear 叙事日志均已从 `PROJECT_LOG_INFO` 降级为 `PROJECT_LOG_DEBUG`；`TacticalCoordinator` 状态存储容量超限仍保留 `warn`。`ar_batch_validation /tmp/1q/check_batch_validation_ar_log_fix` 已完成 52 场景且 `warning=0`、`error=0`、回放失败 0；输出 grep 未再出现 `ThreatAssessmentEvaluator` / `LpiEvaluator` / `Environment is clear` / `Target[` 等旧 info 文本。

**后续边界**：仍未提供按 module/phase 的日志级别控制；若未来需要更细粒度消费侧静音能力，应在 logging 配置层设计。

## 9. 计数字段 uint64 与消费侧直觉不符

**现象**：`ReplayTracePlaybackResult` 的 `compared_output_count` / `applied_input_count` 为 `uint64_t`，但消费侧（含框架初版辅助结构、`RecordProperty` 输出）直觉用 `uint32_t`，触发 `-Wc++11-narrowing` 编译错误。

**影响**：较小。属 API 表面比预期"宽"的小别扭，编译期可捕获。

**复核状态（2026-07-03）**：本轮已补字段文档。`ReplayTracePlaybackResult` 的事件/输入/输出/失败计数字段均明确标注使用 64-bit 是为了支持长回放。

## 10. SAR `dt_sec` 为 double，其余三模块为 float

**现象**：`SarCycleInput.dt_sec`（`include/1q/sar/session/SarCycleInput.h:102`）为 `double`，而 `ArCycleInput.dt_sec` / `EosCycleInput.dt_sec` / `EsrCycleInput.dt_sec` 均为 `float`。

**影响**：四模块对外叙事为"高度对称、由 `foundation/SensorContract.h` 的 `ONEQ_SENSOR_SESSION_CONTRACT` 宏锚定"。任何想写跨模块模板代码、假设 `dt_sec` 类型一致的人会被这处隐蔽的精度不一致咬到（如 `auto dt = input.dt_sec;` 在 AR 推导为 float、在 SAR 推导为 double，传入下游浮点敏感代码时行为可能不同）。属"对称叙事里的精度裂缝"。

**候选改进方向**：统一为同一浮点类型；或在 `SensorContract.h` 注释明确"dt_sec 类型不跨模块保证一致，模板代码勿假设"。

**复核状态（2026-07-03）**：本轮已统一为 `float`。`SarCycleInput.dt_sec`、`SarCycleInputAdapter::Build` 参数、`schemas/replay/sar_replay.fbs` 以及生成的 `sar_replay_generated.h` 均改为 `float`；SAR replay roundtrip 测试也改为 `EXPECT_FLOAT_EQ`。

## 11. `position_frame` 是运行时 tag，非类型区分

**现象**：`ExternalKinematics`（`include/1q/coordinate/types.h:138`）用 `enum class PositionFrame { kEcef, kLla }` 区分坐标系，`position_ecef_m` 与 `position_lla_deg_m` 两个字段同时存在。填 `kEcef` 时 `position_lla_deg_m` 仍存在、可读、只是被忽略，反之亦然。

**影响**：表面像类型安全（有枚举），实为带 tag 的 union——**编译器不检查 tag 与所填字段是否匹配**。消费方填 LLA 时，若 `position_ecef_m` 残留非默认值不会被任何机制拦截。注释虽说明"仅与 position_frame 匹配的字段被读取"，但人在填结构体时容易两个都填或填错，且无运行时断言兜底。

**复核状态（2026-07-03）**：本轮已补强 `ExternalKinematics` 注释，强调只读取与 `position_frame` 匹配的位置字段，另一个位置字段会被忽略；同时明确速度坐标系不随位置 tag 切换。

**后续边界**：仍未增加类型级或 debug-only 校验，兼容性风险较低但不能由编译器阻止填错字段。

## 12. SAR `buildExternalOutput()` 恒返回 false

**现象**：`SarModule::buildExternalOutput()`（`examples/sar/SarModule.h:269`）实现为 `return false;`。`SarCycleOutputAdapter` 不存在——SAR 产品是聚焦图像（复数矩阵），模块本就不提供 ECEF 坐标转换。

**影响**：为让四模块的 `*Module` 包装类表面对称而硬塞的占位方法。消费方调用它，得到恒定 `false`，却无法从签名判断"这是能力缺失还是本次输入异常"——**API 表面承诺了一个能力，实现拒绝履行**。对比 AR/EOS/ESR 的同名方法真正做坐标转换并按输入成败返回，SAR 这个是"为了对称而存在的假方法"。

**候选改进方向**：移除 SAR 的 `buildExternalOutput`（破坏对称但诚实）；或改为纯虚/`deleted` 并在类注释说明"SAR 不支持外部坐标输出"；或返回 `std::optional` 以类型表达"永不产生值"。

**复核状态（2026-07-03）**：本轮已删除 `SarModule::buildExternalOutput()` 占位方法，并把 `examples/sar/SarModule.h` 的差异说明改为"SAR 产品是图像而非轨迹/航迹/辐射源，因此不暴露外部坐标输出适配"。

**边界清理（2026-07-03）**：已对齐三模块类头注释。`examples/airborne_radar/RadarModule.h`、`examples/electro_optical/EosModule.h`、`examples/electronic_warfare/EsrModule.h` 的「此外将通过 buildExternalOutput() 提供 ECEF 外部坐标转换」注释均补充跨模块差异说明（"AR/EOS/ESR 均提供；SAR 因产品为图像不暴露此能力，见 SarModule.h"），与 SAR 的叙事对称，接入者不再需要翻 adapter 实现才能理解为何 SAR 缺该方法。

## 13. `AssociationQualityMetrics` 单结构体内量纲语义混杂

**现象**：`AssociationQualityMetrics`（`include/1q/airborne_radar/session/ArOutputTypes.h:34`）同一结构体内混合三种量纲：

- 原始计数（`size_t`）：`prior_track_count` / `detection_count` / `matched_count` / `new_track_count` / `missed_track_count`；
- 归一化比率（`float [0,1]`）：`match_rate` / `new_track_rate` / `missed_track_rate`；
- 代价/强度（`float`）：`mean_match_cost` / `p95_match_cost` / `jamming_severity` / `association_stress`。

字段名无统一前缀区分"计数 vs 比率 vs 代价"。

**影响**：消费方做聚合时易把计数与比率搞混（例如对 `match_rate` 求和而非求均值，或对 `matched_count` 求均值而非求和）。批量框架统计时需逐字段回忆量纲。注释虽在，但 API 表面未用命名或分组帮助区分。

**复核状态（2026-07-03）**：本轮已补量纲文档。`AssociationQualityMetrics` 注释现在把字段明确分为 counts、rates、costs 与 normalized summaries，并标注 counts 适合求和、rates/costs/summaries 通常按有效周期求均值或分位数。

**后续边界**：未做嵌套重构，保持字段名与 ABI 兼容。

## 反直觉程度分级（非结论）

按"API 表面是否主动给出错误或误导信号"分档。前档的共同特征是：**靠加注释解决不了，需在语义或类型层面修正**。

**★★★ API 主动误导（最该修）**

| 条目 | 根因 |
| --- | --- |
| §4 失败周期输出合法零值 | 头文件已明确指标仅在 `executed_this_cycle=true` 时有效；类型层仍未隔离默认零值 |
| §1 `divergence_found` 分叉时为 false | 结构化字段在分叉发生时给出错误答案，迫使消费方解析文本字符串判断状态（本轮已修复：回调返回结构化枚举 `ReplayTraceOutputStatus`，彻底去除字符串匹配） |

**★★ 接入即摔 / 默认值陷阱**

| 条目 | 根因 |
| --- | --- |
| §2 `sar.json` 通不过自身校验 | 示例配置不能被示例加载，接入第一脚就 abort（本轮已修复） |
| §3 AR 物理探测开关默认关闭 | 示例默认已改为物理链路开启，确定性简化路径需显式关闭 |
| §5 TraceSink 不可回放，普通 demo 缺完整范例 | 头文件入口已补 sink/replay_writer 差异；普通 demo 仍缺完整录制+回放片段 |

**★ 对称叙事裂缝 / 假能力 / 量纲混杂**

| 条目 | 根因 |
| --- | --- |
| §10 SAR `dt_sec` double vs 其余 float | 已统一为 float，并同步 SAR replay schema/生成头 |
| §11 `position_frame` 是 tag 非类型 | 仍非类型安全；`ExternalKinematics` 注释已补字段读取规则 |
| §6 LLA 位置 + ECEF 速度 | 示例误导已修；速度恒 ECEF 与 ENU 转换提示已补 |
| §12 SAR `buildExternalOutput` 恒 false | 已删除 SAR 示例包装层中的恒 false 占位方法 |
| §13 `AssociationQualityMetrics` 量纲混杂 | 字段文档已标注 counts/rates/costs/summaries；结构仍未分组 |
| §7 相邻 Pose 字段不一致无解释 | AR/EOS/ESR Pose 注释已补差异原因 |
| §8 决策层每周期 info 日志无法静音 | 热点 info 已降级为 debug；仍缺按模块日志级别控制 |
| §9 计数字段 uint64 与直觉不符 | 字段文档已补 64-bit 长回放语义 |

## 与本次工作的关系

上述观察均来自 `examples/batch_validation/` 框架的实施与 92 场景实际运行。框架本身已绕开或修复部分问题：§1 公共 replay 回放已映射模块级输出分叉（并通过 `ReplayTraceOutputStatus` 结构化枚举彻底去除字符串匹配，见边界清理说明），§2 SAR 示例配置已自洽，§3 AR 示例默认配置已改为直接启用物理检出与物理 RCS，§4 加 `executed_this_cycle` 门控，§5 头文件已区分 `TraceSink` 与 `ReplayTraceWriter`，§6 当前示例已改用 ENU→ECEF helper 且头文件已补速度坐标系提示（底层位置/速度坐标系非对称未变），§10 SAR `dt_sec` 已统一为 `float`，§12 SAR 示例包装层已删除恒 false 的外部输出占位方法且 AR/EOS/ESR 类头注释已补跨模块差异说明。本草案的目的不是阻塞框架，而是把仍存在的消费侧摩擦点沉淀下来，避免下一个接入方重复踩坑。

2026-07-03 复核命令：

```
cmake --build build/llvm-ninja-release-local --target 1q_unit_tests ar_batch_validation sar_batch_validation -j 8
./build/llvm-ninja-release-local/bin/1q_unit_tests --gtest_filter='EccmEvaluatorTest.*:SarReplayCodecRoundtripTest.*:SarInputValidationTest.*:SarCycleInputAdapterTest.*:SarCycleInputAdapterBridgeTest.*:SarReplaySessionTest.*:SarSessionPipelineTest.*:SarControllerRuntimeStateTest.*:PublicHeadersSmokeTest.*:SarPublicApiConvenienceTest.*'
ctest --test-dir build/llvm-ninja-release-local -R 'public_api_boundary_guard|install_manifest' --output-on-failure
./build/llvm-ninja-release-local/bin/ar_batch_validation /tmp/1q/check_batch_validation_ar_physics_default
./build/llvm-ninja-release-local/bin/sar_batch_validation /tmp/1q/check_batch_validation_sar_dt_float
git diff --check
```

框架设计文档见 `docs/practice/batch_validation.md`，其中 §3.1「已知发现」记录了 §1（ESR 近距离回放分叉）等模块属性结论。

---

## 补充：源码覆盖率提升工作中发现的代码逻辑/架构问题（2026-07-03）

以下条目来自另一条独立工作线：为全项目 Branch 覆盖率从 62% 提升至 70% 的过程中，精读了 AR/EOS/ESR/SAR 四模块约 20 个源文件、编写约 300 个单元测试用例。观察维度从"消费侧 API 摩擦"转向"代码内部逻辑与架构"，与上文 §1–§13 互补。

### A1. EccmEvaluator 的 `eccm_activated` 无法区分"有证据"与"保底"

**现象**：`EccmEvaluator::Evaluate` 的 `Result.eccm_activated` 在以下三种路径中均为 `true`：

1. 检测到真实可信干扰源（`hold_only=false` + 高置信度 jammer）；
2. `hold_only=true`（保守持有路径）；
3. 空干扰源或低置信度（confidence < 阈值）触发 `AccumulateCautiousFallback`。

后两条路径中 `AccumulateCautiousFallback` 给 `adaptive_beamforming_score` 赋非零值，仍跨过激活阈值。

**根因**：`AccumulateCautiousFallback`（`src/airborne_radar/decision/EccmEvaluator.cpp`）的语义是"保守地只开自适应波束形成"，但 `Result` 结构体只有一个布尔 `eccm_activated`，不暴露激活原因。调用方若按"`true` = 检测到真实干扰"决策，会被保底路径误导。

**影响**：布尔值**无法区分"有证据激活"和"无证据保底激活"**。调用方需要靠 `proposals` 列表内容间接判断，但 `Result` 没有暴露这个区分，也没有 `activation_reason` 枚举。

**候选改进方向**：在 `Result` 增加激活来源枚举（如 `kEvidenceBased` / `kCautiousFallback`），或增加 `proposals` 的数量/类型摘要字段。

**复核状态（2026-07-03）**：本轮已增加 `EccmEvaluator::ActivationSource`，`Result` 现在携带 `activation_source`。当前来源包括 `kNone`、`kEvidenceBased`、`kCautiousFallback`、`kAssociationPressure`；单元测试覆盖 null proposals、hold-only、空/低置信干扰源、可信干扰源以及 association-pressure 路径的来源断言。

**边界清理（2026-07-03）**：已明确 `activation_source` 反映**实际激活来源**（哪个 proposal 真正达标），而非证据有无。枚举注释说明：即使 `has_credible_multisource_evidence=true`，若五项评分全部未跨过阈值，控制流进入末尾「最低保底」分支强制激活自适应波束形成——此时实际达标的是保底 proposal，故标记 `kCautiousFallback` 而非 `kEvidenceBased`。新增 `EccmEvaluatorTest.CredibleEvidenceButScoresBelowThresholdProducesFallback`（基础重载）与 `...ProducesFallbackAssoc`（关联重载）覆盖这条此前无测试的混合路径。

### A2. 基础 `Evaluate` 重载可能未被生产代码调用（历史覆盖快照曾为 0%）

**现象**：`EccmEvaluator::Evaluate(const EccmSourceInfo&, bool hold_only, proposals)` 这个 public 重载在覆盖率提升工作线的历史快照中曾是 **0% 覆盖**（26 个 branch 全 miss）。现有生产路由中，`TacticalCoordinator` 实际调用的是**另一个**带 `AssociationQualityInfo` 参数的重载。

**复核状态（2026-07-03）**：测试注册问题当前已消除。`build/llvm-ninja-release-local/bin/1q_unit_tests --gtest_filter='EccmEvaluatorTest.*'` 现运行 15 个测试并全部通过，`tests/unit/ar_eccm_evaluator_test.cpp` 已覆盖基础重载及 association-pressure 路径。

**边界清理（2026-07-03）**：已确认生产路径——全仓（排除 tests）中仅 `TacticalCoordinator.cpp` 调用 `EccmEvaluator::Evaluate`，且调用的是带 `AssociationQualityInfo` 的关联重载；基础重载在生产代码中无任何调用点。基础重载已标记 `[[deprecated]]`（注释指向关联重载作为替代），`tests/unit/ar_eccm_evaluator_test.cpp` 作为历史回归覆盖仍调用基础重载，对整个文件 `#pragma clang diagnostic ignored "-Wdeprecated-declarations"` 抑制警告。

**影响**：基础重载现为显式废弃 API，新代码应使用关联重载；历史回归测试保留对其的覆盖。

**候选改进方向**：已完成（标记 `[[deprecated]]`）。

### A3. 四模块 ReplaySession 各自手写巨型 `*Equal` 比较函数

**现象**：`SarOutputFrameEqual`（42 个 branch）、`TrackStateSnapshotEqual`、`EmitterHypothesisEqual` 等结构上几乎相同的深度比较函数，在 AR/ESR/EOS/SAR 四个模块各写一遍，均为匿名命名空间的文件静态函数。

**影响**：
- 四份近似实现，维护成本 ×4——改一个字段要在 4 个地方同步改；
- 匿名函数**无法被单元测试直接调用**，只能通过整个 replay 流程间接触达；
- 只在"输出恰好一致"时走全分支，"输出不一致"的 `return false` 短路分支在现有测试中从未执行；
- 结果：**最关键的正确性校验逻辑（replay 分叉检测）恰恰是测试覆盖率最低的代码**。

**候选改进方向**：提取泛型 `ReplayDivergenceDetector<T>` 或声明式比较框架，将四份手写实现收敛为配置。

**复核状态（2026-07-03）**：按本轮确认延后。该项属于 replay 比较框架的结构性重构，不阻塞当前批量验证修复；后续若推进，应单独形成跨模块 replay comparator 设计与迁移计划。

### A4. 部分模块 `OnCycleOutput` 有不可达的 trailing fallback

**现象**：AR/EOS/ESR 的 `OnCycleOutput` 形态类似：

```cpp
if (type == "CycleResult") { ... }
else if (type == "OutputFrame") { ... }
else {
    *error = "...does not support...";
    return false;  // ← 逻辑不可达
}
```

函数入口已有类型检查保证只会是这两种 payload 类型，末尾 fallback **在当前控制流下不可达**。SAR 不属于同一形态：`src/sar/session/SarReplaySession.cpp` 只有入口类型检查，后续直接处理 `SarOutputFrame` 并返回，没有同样的 trailing fallback。

**影响**：不是 bug，但制造虚假安全感——维护者可能以为这个 else 会做防护，实际永远不执行。每次新增 payload 类型时，容易误判这个分支的作用。

**复核状态（2026-07-03）**：本轮已移除 AR/EOS/ESR 三处入口类型检查后的 trailing fallback。非法 payload 类型仍由函数入口检查返回原有错误，合法 `CycleResult` / `OutputFrame` 路径直接返回比较结果。

### A5. flatbuffers 生成代码系统性拉低覆盖率基线，且当前报告存在路径重复计数

**现象**：8 个 `*_replay_generated.h` 文件的 Branch 覆盖基本卡在 **50%** 左右——flatc 生成的 `Verify()` 等防御性方法很少被调用。按当前 `coverage_report/summary.txt` 的唯一文件口径，8 个 generated replay header 合计约 `2136` 个 branch、`1062` 个 missed branch；但当前报告把部分源码目录重复传给 `llvm-cov`，导致 summary 中同一 generated 文件重复出现。按报告 TOTAL 口径聚合，generated replay header 被计入约 `8544` 个 branch、`4248` 个 missed branch。

**影响**：修复前 `coverage_report.sh` 没有排除 generated 目录，也没有去重传入的源码目录，导致整体 Branch 覆盖率同时受"机器生成代码"和"路径重复计数"影响。历史报告 TOTAL 为 `40361` branches / `13827` missed / `65.74%`；按当时重复计数口径排除 generated replay header 后约为 `31817` branches / `9579` missed / `69.89%`。因此，旧覆盖率数字不宜直接解读为纯手写代码质量。

**复核状态（2026-07-03）**：本轮已修复脚本。`tools/coverage_report.sh` 现在只把 `src/` 与 `include/` 两个根目录传给 `llvm-cov show/report`，并统一增加 `-ignore-filename-regex=.*/generated/.*`。`bash -n tools/coverage_report.sh` 已通过；完整覆盖率数值需重新跑 coverage preset 后更新。

### A6. `ResolveScanStepScale` 有不可达的 switch case

**现象**：`ScanScheduleResolver.cpp` 的 `ResolveScanStepScale` switch 有 `kStby`/`kStt`/`default` 三个 case，但 `ResolveScheduledBeamPointing` 在到达此函数前对 `kStby`（提前 return）和 `kStt`（提前 return）已拦截。因此在当前 `ResolveScheduledBeamPointing` 调用路径下，这两个 case 不会被触达。

**边界**：`ResolveScanStepScale` 本身在 `src/airborne_radar/signal/pipeline/ScanScheduleResolver.h` 中暴露，单测或其他内部调用仍可直接传入 `kStby`/`kStt`。所以这里不是全局不可达，而是主调度路径上的死分支/重复防护。

**影响**：与 A4 同类——不是 bug，但 switch 里写了主路径不会走到的 case，读代码的人会误以为这些模式会在调度路径中被特殊处理。

**复核状态（2026-07-03）**：本轮保留直接调用兜底并补测试。`ScanScheduleResolverTest.ResolveScanStepScaleDefinesDirectCallFallbacks` 明确验证 TAS=0.5、TWS/STBY/STT/未知模式=1.0；主调度路径仍在进入 helper 前提前处理 STBY/STT。

### 反直觉程度分级（覆盖率视角）

| 条目 | 性质 | 与上文 §1–§13 的关系 |
| --- | --- | --- |
| A1 `eccm_activated` 语义失真 | 已增加激活来源枚举 | 与 §1（`divergence_found` 失真）同类：结构化字段给出错误答案 |
| A2 基础 Evaluate 历史 0% 覆盖 | 测试注册已恢复；生产路径仍待确认 | 独立发现，不与上文重叠 |
| A3 四模块手写 Equal 重复 | 架构重复 + 不可测；本轮延后 | 与 §1 回放分叉语义相关：分叉检测逻辑本身最缺测试 |
| A4 OnCycleOutput dead else | 已移除入口检查后的不可达 fallback | 独立发现 |
| A5 生成代码拉低基线 | 度量失真（脚本本轮已修复，待重算覆盖率） | 独立发现，影响覆盖率数字的可信度 |
| A6 ResolveScanStepScale 不可达 case | 直接调用兜底语义已测试 | 与 A4 同类 |

上述 A1–A6 条目推进到有结论时，同样应回写为契约规则、模块设计或开放议题。
