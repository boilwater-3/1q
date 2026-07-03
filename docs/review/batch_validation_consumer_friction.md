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

**复核状态（2026-07-03）**：仍存在。运行 `esr_batch_validation` 复现 `FAIL ok=0 divergence=0 ... err=ESR replay output divergence (EsrCycleResult)`。

**候选改进方向**（待讨论，非结论）：
- 在各模块 `on_cycle_output` 回调中，当 `*error` 被设为分叉描述时同步置 `playback.divergence_found=true`；
- 或在 `XxxReplaySessionResult` 增加强类型分叉标志，与通用 `divergence_found` 解耦。

## 2. 示例配置 `sar.json` 通不过自身校验

**现象**：`examples/configs/sar.json` 默认参数（`sample_rate_hz=120e6` + `pulse_width_s=20e-6`）需 `ceil(20e-6 × 120e6)=2400` 个采样点，但 `range_sample_count=2048`。直接使用这组默认配置会触发 `kSampleWindowTooSmallForPulse` 校验失败；批量验证当前能跑通，是因为执行前覆盖了这组三元参数。

**根因**：校验约束 `ceil(pulse_width × sample_rate) ≤ range_sample_count`（`include/1q/sar/config/SarSessionConfigValidation.h:33,56`）被默认配置自身违反。

**绕开方式**：`examples/sar/integration_demo.cpp` 不读 `sar.json`，而在程序内用 `WriteTempConfig()` 写一份验证过的临时配置（`sample_rate=1MHz`、`pulse_width=20us`、`range_sample_count=1024`）。`examples/batch_validation/sar_batch_validation.cpp:131-142` 也明确注释 `sar.json` 默认值违反采样窗口约束，并在 `ApplyCaseToConfig()` 中覆盖为同一类自洽参数。因此批量验证能跑通，但并不证明 `sar.json` 本身可直接消费。

**影响**：任何按 `examples/configs/README.md` 指引"加载 sar.json 接入"的消费方会立刻撞墙。`sar.json` 的默认值与 demo 实际用的值不一致，且无注释解释。

**复核状态（2026-07-03）**：问题本体仍存在，批量程序已局部绕开。运行 `sar_batch_validation` 成功，是因为场景执行前覆盖了违例参数。

**候选改进方向**：修正 `sar.json` 的 `sample_rate/pulse_width/range_sample_count` 三者自洽；或在配置文件加注释标注约束；或在 `config_loader` 加载失败时给出指向 demo 验证参数的提示。

## 3. AR 物理探测开关默认关闭，掩盖距离/RCS 趋势

**现象**：`airborne_radar.json` 默认 `enable_physics_detection=false` + `enable_physical_rcs=false`。关闭后雷达走确定性简化路径，所有目标被检出——距离 8km 与 120km、RCS 0.01 与 10 m² 输出**完全一致**。

**影响**：批量框架第一轮 AR 全部 52 场景指标一字不差，差点误判为框架损坏。回读 `detection_count` 才发现物理链路未启用。最终在框架内强制 `enable_physics_detection=true` + `enable_physical_rcs=true` 才让趋势显现。

**合理性**：该默认值对单元测试（要求确定性、可复现）合理。但对"想验证物理探测能力"的接入方是陷阱——配置文件无任何注释提示"观察距离/RCS 衰减需打开这两个开关"。

**复核状态（2026-07-03）**：仍存在。`examples/configs/airborne_radar.json` 默认关闭，`examples/batch_validation/ar_batch_validation.cpp:191-192` 仍需显式强制打开。

**候选改进方向**：在 `airborne_radar.json` 相关字段加注释说明默认关闭的原因与开启条件；或在 `docs/airborne_radar/design.md` 接入章节明确这一点。

## 4. 失败周期输出为默认零值，与"真实零"不可区分

**现象**：`ArCycleResult` / `EosCycleResult` / `EsrCycleResult` 头部注释均声明"若未执行则保持默认值"。于是 abort 周期的 `match_rate=0`、`fused_snr_db=0`、`confirmed_count=0`——与合法算出的零值无法区分。

**影响**：框架第一版把所有周期直接求均值，一个因几何退化 abort 的场景其 `match_rate=0` 拉低整体均值，趋势失真。被迫加 `executed_this_cycle` 门控、只统计稳态窗口才修正。

**与契约的张力**：`contract.md` 三层输出模型称 `StepWithResult()` 是"状态判断入口"，但失败时结构化数值字段与成功时同形（都是合法零），消费方必须先查 `executed_this_cycle` 才能安全使用任何数值。任何做统计/聚合的消费方若忽略此门控，会静默出错。

**候选改进方向**：失败周期将数值字段置 NaN（而非默认零）；或增加 `has_valid_metrics` 标志；或在各 `*CycleResult` 的数值字段文档里强制注明"仅在 `executed_this_cycle=true` 时有效"。

## 5. TraceSink 不可回放，普通 integration demo 缺少完整范例

**现象**：要让 trace 可回放，必须给 `*TraceSession` 传 `ReplayTraceWriter`，而非 `TraceSink`。两者产出格式完全不同：

- `FlatbufferFileTraceSink`（`include/1q/trace/TraceSink.h:45`）产出 `uint32_le length + FlexBuffers map` 单文件，调试用流，**不能**被 `ReplayXxxTrace` 回放。
- `ReplayTraceWriter`（`include/1q/replay/ReplayTrace.h:183`）产出 `manifest.json + events/*.jsonl + indexes/ + crash/` 目录，才是可回放格式。

**影响**：普通 integration demo 仍没有真正录制 + 回放的可运行片段，demo 里的 `enableTrace()` 只打印一行"实际工程应使用 TraceSession"的占位（如 `RadarModule.h:305`）。不过"唯一正确范例藏在单测里"这一旧判断已经过期：`docs/practice/batch_validation.md:136-158` 与 `examples/batch_validation/batch_replay.h:10-15` 已经明确说明 `ReplayTraceWriter` 与 `TraceSink` 的用途差异，并提供批量验证框架内的正确接入姿势。

**复核状态（2026-07-03）**：核心事实仍存在，消费摩擦已被 batch validation 文档/工具部分缓解。剩余问题是普通 integration demo 与 `*TraceSession` 头文件仍不够显著。

**候选改进方向**：在一个普通 integration demo 中补一段真正录制 + 回放的可运行示例；或在 `*TraceSession` 头文件补"Sink vs ReplayWriter 用途差异"的显著注释，并指向 `docs/practice/batch_validation.md`。

## 6. 坐标输入"位置可 LLA、速度恒 ECEF"的非对称（原示例误导已修）

**现象**：`ExternalKinematics`（`include/1q/coordinate/types.h:138`）允许 `position_frame=kLla` 填经纬度位置，但注释明确"速度固定为 ECEF 坐标系"。这仍是运行时约定，编译器不会把 LLA 位置与 ENU/ECEF 速度类型绑定起来。

**已变化点**：旧版观察中提到的 `examples/electro_optical/session_usage.cpp::MakeTargetLlaWithVelocity(lat, lon, alt, vx, vy, vz, ...)` 已不成立。当前函数参数为 `vel_east_mps / vel_north_mps / vel_up_mps`，并调用 `oneq::coordinate::TryEnuToEcefVelocity()` 写入 `target.kinematics.velocity_mps`。仓库也已有 `include/1q/coordinate/velocity_transform.h` 中的 ENU→ECEF 速度转换辅助。

**剩余影响**：底层结构仍是"位置 frame 可选、速度恒 ECEF"的非对称设计，直接填 `ExternalKinematics` 的消费方仍可能跳过 helper 而填错速度系。该条不再属于"API 主动误导"最高档，更适合归入"运行时 tag / 坐标约定需显著提示"。

**候选改进方向**：在 `ExternalKinematics.velocity_mps` 与各目标输入适配器注释里更显著地标注"速度恒为 ECEF，若手头是 ENU 速度请先调用 `TryEnuToEcefVelocity()`"。

## 7. 相邻 Pose 结构体字段不一致且无解释

**现象**：`ArExternalPoseInput` 含 `radar_mount_angles_deg`（雷达相对机身安装角），`EosExternalPoseInput` / `EsrExternalPoseInput` 只有 `platform_attitude_deg`，无 mount 字段。

**合理性**：该不对称有合理原因（AR 需复合雷达安装角建立雷达本地系；光电/电子侦察视轴与机身对齐更自然）。

**影响**：三个结构体并排放置，无任何注释说明"为何 AR 多一个字段"。接入者读到 EOS/ESR 的 Pose 时易怀疑漏字段，需翻 adapter 实现才确认是设计如此。

**复核状态（2026-07-03）**：基本存在但严重度较低。当前字段自身有注释说明，缺的是跨模块说明"为什么 AR 多 mount、EOS/ESR 没有"。

**候选改进方向**：在三个 `*ExternalPoseInput` 头文件补注释，互相指代并说明字段差异的物理原因。

## 8. 决策层每周期 info 日志无法被消费侧静音

**现象**：AR 的 `TacticalCoordinator`、`ThreatAssessmentEvaluator`、`LpiEvaluator` 在每次 `StepWithResult` 都向 spdlog 打 `info` 级日志（"Environment is clear"、"Target[x] -> Classification: UNKNOWN"）。50 周期 × 多场景，stdout 被数万行日志淹没。

**张力**：`CLAUDE.md` 工程约束明确"高速仿真热点路径不打日志"，但决策层该路径每周期都打。更麻烦的是日志走 spdlog 全局配置，**消费侧程序无法单独静音某模块日志**，只能整体重定向，连同消费侧自身进度输出一起被冲掉。

**复核状态（2026-07-03）**：仍存在。运行 `ar_batch_validation` 时 stdout/stderr 被 `ThreatAssessmentEvaluator`、`LpiEvaluator`、`TacticalCoordinator` 的 info 日志大量刷屏。

**候选改进方向**：将决策层逐周期分类结果降级为 `debug` 或改为节流（如仅状态变化时打）；或提供按 module/phase 的日志级别控制。

## 9. 计数字段 uint64 与消费侧直觉不符

**现象**：`ReplayTracePlaybackResult` 的 `compared_output_count` / `applied_input_count` 为 `uint64_t`，但消费侧（含框架初版辅助结构、`RecordProperty` 输出）直觉用 `uint32_t`，触发 `-Wc++11-narrowing` 编译错误。

**影响**：较小。属 API 表面比预期"宽"的小别扭，编译期可捕获。

**候选改进方向**：保持 uint64（正确），但在字段文档注明类型，降低消费侧初次使用摩擦。

## 10. SAR `dt_sec` 为 double，其余三模块为 float

**现象**：`SarCycleInput.dt_sec`（`include/1q/sar/session/SarCycleInput.h:102`）为 `double`，而 `ArCycleInput.dt_sec` / `EosCycleInput.dt_sec` / `EsrCycleInput.dt_sec` 均为 `float`。

**影响**：四模块对外叙事为"高度对称、由 `foundation/SensorContract.h` 的 `ONEQ_SENSOR_SESSION_CONTRACT` 宏锚定"。任何想写跨模块模板代码、假设 `dt_sec` 类型一致的人会被这处隐蔽的精度不一致咬到（如 `auto dt = input.dt_sec;` 在 AR 推导为 float、在 SAR 推导为 double，传入下游浮点敏感代码时行为可能不同）。属"对称叙事里的精度裂缝"。

**候选改进方向**：统一为同一浮点类型；或在 `SensorContract.h` 注释明确"dt_sec 类型不跨模块保证一致，模板代码勿假设"。

## 11. `position_frame` 是运行时 tag，非类型区分

**现象**：`ExternalKinematics`（`include/1q/coordinate/types.h:138`）用 `enum class PositionFrame { kEcef, kLla }` 区分坐标系，`position_ecef_m` 与 `position_lla_deg_m` 两个字段同时存在。填 `kEcef` 时 `position_lla_deg_m` 仍存在、可读、只是被忽略，反之亦然。

**影响**：表面像类型安全（有枚举），实为带 tag 的 union——**编译器不检查 tag 与所填字段是否匹配**。消费方填 LLA 时，若 `position_ecef_m` 残留非默认值不会被任何机制拦截。注释虽说明"仅与 position_frame 匹配的字段被读取"，但人在填结构体时容易两个都填或填错，且无运行时断言兜底。

**候选改进方向**：保持现状（兼容性优先），但在 `ExternalKinematics` 补显著的"二者只能填其一，否则未定义"警告注释；或加 debug-only 运行时校验。

## 12. SAR `buildExternalOutput()` 恒返回 false

**现象**：`SarModule::buildExternalOutput()`（`examples/sar/SarModule.h:269`）实现为 `return false;`。`SarCycleOutputAdapter` 不存在——SAR 产品是聚焦图像（复数矩阵），模块本就不提供 ECEF 坐标转换。

**影响**：为让四模块的 `*Module` 包装类表面对称而硬塞的占位方法。消费方调用它，得到恒定 `false`，却无法从签名判断"这是能力缺失还是本次输入异常"——**API 表面承诺了一个能力，实现拒绝履行**。对比 AR/EOS/ESR 的同名方法真正做坐标转换并按输入成败返回，SAR 这个是"为了对称而存在的假方法"。

**候选改进方向**：移除 SAR 的 `buildExternalOutput`（破坏对称但诚实）；或改为纯虚/`deleted` 并在类注释说明"SAR 不支持外部坐标输出"；或返回 `std::optional` 以类型表达"永不产生值"。

## 13. `AssociationQualityMetrics` 单结构体内量纲语义混杂

**现象**：`AssociationQualityMetrics`（`include/1q/airborne_radar/session/ArOutputTypes.h:34`）同一结构体内混合三种量纲：

- 原始计数（`size_t`）：`prior_track_count` / `detection_count` / `matched_count` / `new_track_count` / `missed_track_count`；
- 归一化比率（`float [0,1]`）：`match_rate` / `new_track_rate` / `missed_track_rate`；
- 代价/强度（`float`）：`mean_match_cost` / `p95_match_cost` / `jamming_severity` / `association_stress`。

字段名无统一前缀区分"计数 vs 比率 vs 代价"。

**影响**：消费方做聚合时易把计数与比率搞混（例如对 `match_rate` 求和而非求均值，或对 `matched_count` 求均值而非求和）。批量框架统计时需逐字段回忆量纲。注释虽在，但 API 表面未用命名或分组帮助区分。

**候选改进方向**：保持字段不变（兼容性），但按量纲分组嵌套（如 `counts{}` / `rates{}` / `costs{}`）；或在字段文档统一标注单位与取值范围。

## 反直觉程度分级（非结论）

按"API 表面是否主动给出错误或误导信号"分档。前档的共同特征是：**靠加注释解决不了，需在语义或类型层面修正**。

**★★★ API 主动误导（最该修）**

| 条目 | 根因 |
| --- | --- |
| §4 失败周期输出合法零值 | 状态判断入口在失败时与成功同形，无任何信号提示"数据是垃圾"，统计类消费静默出错 |
| §1 `divergence_found` 分叉时为 false | 结构化字段在分叉发生时给出错误答案，迫使消费方解析文本字符串判断状态 |

**★★ 接入即摔 / 默认值陷阱**

| 条目 | 根因 |
| --- | --- |
| §2 `sar.json` 通不过自身校验 | 示例配置不能被示例加载，接入第一脚就 abort |
| §3 AR 物理探测开关默认关闭 | 合理默认值无任何接入提示，掩盖距离/RCS 趋势 |
| §5 TraceSink 不可回放，普通 demo 缺完整范例 | batch validation 已有正确姿势，但普通 demo/头文件入口仍不够显著 |

**★ 对称叙事裂缝 / 假能力 / 量纲混杂**

| 条目 | 根因 |
| --- | --- |
| §10 SAR `dt_sec` double vs 其余 float | 对称叙事里的隐蔽精度裂缝 |
| §11 `position_frame` 是 tag 非类型 | 假装类型安全，编译器不检查 tag 与字段匹配 |
| §6 LLA 位置 + ECEF 速度 | 示例误导已修，但底层坐标约定仍靠运行时 tag 与注释传达 |
| §12 SAR `buildExternalOutput` 恒 false | 为对称硬塞的占位，API 承诺能力实现拒绝履行 |
| §13 `AssociationQualityMetrics` 量纲混杂 | 计数/比率/代价混于一 struct，无命名区分 |
| §7 相邻 Pose 字段不一致无解释 | AR 多 mount 字段无注释，接入者怀疑漏字段 |
| §8 决策层每周期 info 日志无法静音 | 违反热点不打日志约束，消费侧无法按模块静音 |
| §9 计数字段 uint64 与直觉不符 | API 表面比预期宽，编译期可捕获，影响小 |

## 与本次工作的关系

上述观察均来自 `examples/batch_validation/` 框架的实施与 92 场景实际运行。框架本身已绕开这些问题正常工作：§2 覆盖 SAR 违例参数，§3 强制开物理开关，§4 加 `executed_this_cycle` 门控，§5 使用 `ReplayTraceWriter` 并已沉淀到 `docs/practice/batch_validation.md`，§6 当前示例已改用 ENU→ECEF helper。本草案的目的不是阻塞框架，而是把仍存在的消费侧摩擦点沉淀下来，避免下一个接入方重复踩坑。

2026-07-03 复核命令：

```
./build/llvm-ninja-release-local/bin/sar_batch_validation /tmp/1q/check_batch_validation_sar
./build/llvm-ninja-release-local/bin/ar_batch_validation /tmp/1q/check_batch_validation_ar
./build/llvm-ninja-release-local/bin/esr_batch_validation /tmp/1q/check_batch_validation_esr
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

### A2. 基础 `Evaluate` 重载可能从未被生产代码调用（0% 覆盖）

**现象**：`EccmEvaluator::Evaluate(const EccmSourceInfo&, bool hold_only, proposals)` 这个 public 重载在补测试前是 **0% 覆盖**（26 个 branch 全 miss）。现有 `ar_decision_layer_test.cpp` 只通过 `TacticalCoordinator` 间接测试，而 `TacticalCoordinator` 实际路由到的是**另一个**带 `AssociationQualityInfo` 参数的重载。

**影响**：一个 `public` 方法、26 行业务逻辑、在整个测试套件中从未被执行。可能意味着它是遗留废弃 API（新代码已迁移到 association overload），但没有任何 `[[deprecated]]` 标记。

**候选改进方向**：确认生产调用路径；若确为废弃则标记 `[[deprecated]]` 或删除；若仍需要则在 `TacticalCoordinator` 补一条测试覆盖。

### A3. 四模块 ReplaySession 各自手写巨型 `*Equal` 比较函数

**现象**：`SarOutputFrameEqual`（42 个 branch）、`TrackStateSnapshotEqual`、`EmitterHypothesisEqual` 等结构上几乎相同的深度比较函数，在 AR/ESR/EOS/SAR 四个模块各写一遍，均为匿名命名空间的文件静态函数。

**影响**：
- 四份近似实现，维护成本 ×4——改一个字段要在 4 个地方同步改；
- 匿名函数**无法被单元测试直接调用**，只能通过整个 replay 流程间接触达；
- 只在"输出恰好一致"时走全分支，"输出不一致"的 `return false` 短路分支在现有测试中从未执行；
- 结果：**最关键的正确性校验逻辑（replay 分叉检测）恰恰是测试覆盖率最低的代码**。

**候选改进方向**：提取泛型 `ReplayDivergenceDetector<T>` 或声明式比较框架，将四份手写实现收敛为配置。

### A4. 每个模块 `OnCycleOutput` 都有不可达的 trailing `else`

**现象**：四个 ReplaySession 的 `OnCycleOutput` 均为：

```cpp
if (type == "CycleResult") { ... }
else if (type == "OutputFrame") { ... }
else {
    *error = "...does not support...";
    return false;  // ← 逻辑不可达
}
```

函数入口已有类型检查保证只会是这两种 payload 类型，trailing `else` **逻辑上不可达**。

**影响**：不是 bug，但制造虚假安全感——维护者可能以为这个 else 会做防护，实际永远不执行。每次新增 payload 类型时，容易误判这个分支的作用。

**候选改进方向**：移除 dead else，或改为 `assert(false)` 明确标注"逻辑上不应到达"。

### A5. flatbuffers 生成代码系统性拉低覆盖率基线

**现象**：8 个 `*_replay_generated.h` 文件全部精确卡在 **50%** Branch 覆盖——flatc 生成的 `Verify()` 等防御性方法从未被调用。它们贡献 **1068 个"不可达"缺失 branch**，占全项目 branch 总数约 2.7%。

**影响**：即便所有手写代码测到 100%，整体 Branch 也只能到约 86%（生成代码的 50% 是硬天花板）。`coverage_report.sh` 当前没有排除生成代码，导致 60%+ 的数字在惩罚机器生成的代码。

**候选改进方向**：在 `tools/coverage_report.sh` 的 llvm-cov 命令增加 `-ignore-filename-regex='generated/'`，让度量只反映手写代码质量。

### A6. `ResolveScanStepScale` 有不可达的 switch case

**现象**：`ScanScheduleResolver.cpp` 的 `ResolveScanStepScale` switch 有 `kStby`/`kStt`/`default` 三个 case，但 `ResolveScheduledBeamPointing` 在到达此函数前对 `kStby`（提前 return）和 `kStt`（提前 return）已拦截。

**影响**：与 A4 同类——不是 bug，但 switch 里写了永远不会走到的 case，读代码的人会误以为这些模式会被特殊处理。

**候选改进方向**：删除不可达 case，或重构控制流使 `ResolveScanStepScale` 只在可达路径被调用。

### 反直觉程度分级（覆盖率视角）

| 条目 | 性质 | 与上文 §1–§13 的关系 |
| --- | --- | --- |
| A1 `eccm_activated` 语义失真 | 名字暗示不存在的语义 | 与 §1（`divergence_found` 失真）同类：结构化字段给出错误答案 |
| A2 基础 Evaluate 0% 覆盖 | 可能废弃 API | 独立发现，不与上文重叠 |
| A3 四模块手写 Equal 重复 | 架构重复 + 不可测 | 与 §1 回放分叉语义相关：分叉检测逻辑本身最缺测试 |
| A4 OnCycleOutput dead else | 虚假安全感 | 独立发现 |
| A5 生成代码拉低基线 | 度量失真 | 独立发现，影响覆盖率数字的可信度 |
| A6 ResolveScanStepScale 不可达 case | 虚假安全感 | 与 A4 同类 |

上述 A1–A6 条目推进到有结论时，同样应回写为契约规则、模块设计或开放议题。
