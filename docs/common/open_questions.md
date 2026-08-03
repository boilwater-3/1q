---
Status: active
Authority: 非规定性记录（不构成契约约束）
Lifecycle: 条目有结论后回写 contract.md 或 design.md 并从本文删除；不保留已收敛条目
Last-reviewed: 2026-08-03
---

# 跨模块开放议题

登记调查中发现但尚未定论的跨模块架构议题。每条仅记录现状、后果、待决问题、当前边界与再进入条件，
**不构成**已批准的实现要求或契约规则。已定论条目必须迁出本文——契约规则进 `docs/common/contract.md`，
模块设计进对应 `design.md`。

- 何时读本文：评估某项反直觉/非阻断行为是否为已知边界、查某议题的再进入门槛、为 `harden-1q-simulation-module`
  的 Class D 发现登记一条开放议题。
- 何时不读本文：查必须遵守的规则（去 `contract.md`）、查模块设计细节（去对应 `design.md`）、查工程实践
  （去 `docs/practice/`）。

## 议题索引

| ID | 域 | 主题 | 一句话 | Status |
|---|---|---|---|---|
| COMMON-OQ-1 | common | Windows/MSVC 全链验收 | presets/.bat 仅未验收脚手架，CI 只跑 macOS | needs-evidence |
| COMMON-OQ-6 | common | `ApplyRuntimeConfig` 吞 bool | void 版丢弃 Try 的返回值 | open |
| COMMON-OQ-7 | common | 双 cycle_index 冗余 | 非执行周期 input_cycle_index 保留输入号，output_frame.cycle_index 为0 | open |
| COMMON-OQ-8 | common | 周期时间/窗口静默拒绝 | AR/ESR/EOS 各自为政，违反多表现为静默不生效 | open |
| AR-OQ-1 | airborne_radar | 假目标鉴别跨域命名双轨 | 观测域枚举 vs 量测域 bool | open |
| ESR-OQ-1 | electronic_surveillance_radar | 压制干扰感知与 ECCM 链路缺失 | 死字段 + 无结构化观测 + 无 ECCM | open |
| ESR-OQ-2 | electronic_surveillance_radar | 运行时补丁扫描中心静默关边界 | scan center 补丁隐式切扫描模式 | open |
| ESR-OQ-3 | electronic_surveillance_radar | 扫描策略跨域耦合 | mission 值被 hardware mount 静默偏移 | open |
| SBIRS-OQ-1 | sbirs_sensor | 诊断距离的物理语义 | 仅 cue/诊断层，易被误读为测距输出 | open |
| SBIRS-OQ-2 | sbirs_sensor | 分阶段误差统计共享参数 | 三用途共享一组角度/距离统计 | open |
| SBIRS-OQ-3 | sbirs_sensor | 多目标随机样本与输入顺序 | 全局用途流，无 target 不变性 | open |
| SBIRS-OQ-4 | sbirs_sensor | Estimated 航迹真值初始化 | 首捕用真值位置/速度初始化滤波均值 | open |

## Common 非阻塞边界

### COMMON-OQ-1：Windows/MSVC 全链支持验收

- **现状**：仓库存在 Windows Conan/no-Conan presets 和 `scripts/fetch_third_party.bat`，但当前 CI 只在
  macOS 运行。该脚本还存在两个未验收风险点：
  1. 下载来源含 GitLab/archives.boost.io，非锁定 GitHub。
  2. 未对下载内容做 hash 校验。
  [evidence: scripts/fetch_third_party.bat]
- **后果**：
  1. 外部读者可能据 presets 或 `.bat` 误判 Windows 已受支持。
  2. 无校验的下载链在真实 Windows runner 上不可重复，无法证明 configure/build/install/consumer 闭环。
- **待决问题**：如何实现已冻结的 Windows shell/GitHub bootstrap，并在不依赖 Windows Conan 的前提下形成
  可重复的依赖、configure、build、install 和外部 consumer 闭环。
- **当前边界**：这些 presets 和脚本只视为未验收脚手架。不得在文档中宣称 Windows 已受支持，也不得把
  Conan 路径自动提升为正式 Windows 方案。
- **再进入条件 (Stage A)**：提交锁定版本/提交与下载校验矩阵，提供 shell bootstrap 原型，并在真实
  Windows runner 上依次证明 configure、Debug/Release build、install、独立 consumer build/run；随后再决定
  保留、删除或重命名现有 presets 与 `.bat` 入口。

### COMMON-OQ-6：`ApplyRuntimeConfig` 吞掉 `TryApplyRuntimeConfig` 返回值

- **现状**：五模块会话均提供 `ApplyRuntimeConfig(patch)`（void）与 `TryApplyRuntimeConfig(patch)`（bool）。
  void 版内部 `(void)TryApplyRuntimeConfig(patch)` 显式丢弃成功与否。跨模块形态不一致：
  1. ESR 额外提供 `ApplyRuntimeConfigWithResult` 返回结构化 `EsrRuntimeConfigApplyResult`（含状态枚举）。
  2. 其余四模块（AR/SAR/EOS/SBIRS）无此变体。
  3. docstring 警告不一致：SAR/SBIRS 标注"不返回成功与否"，AR/EOS 无此警告。
  与 OQ-2/OQ-8 同属"静默语义"反模式家族。[evidence: src/electronic_surveillance_radar/session/EsrSession]
- **后果**：
  1. void 版失败被静默吞掉，调用方无从感知补丁是否真正生效。
  2. docstring 不一致加剧误用风险。
  3. ESR 独有的结构化结果变体未跨模块推广，跨模块集成时返回形态不统一。
- **待决问题**：
  1. 是否废弃 void 版、统一强制使用 Try/WithResult。
  2. 是否向其余四模块推广 `ApplyRuntimeConfigWithResult` 结构化结果。
  3. 是否统一 docstring 警告。
- **当前边界**：五模块保持 void+Try 双方法，void 版吞返回值为已知设计。ESR 的 WithResult 变体为 ESR
  独有增强，不构成跨模块契约。
- **再进入条件 (Stage A)**：出现真实场景要求 void 版失败必须可观测，或跨模块集成要求统一结果返回形态时，
  先评估推广 `ApplyRuntimeConfigWithResult` 的 API 成本与四模块补丁结构差异，再决定是否统一。

### COMMON-OQ-7：CycleResult.input_cycle_index 与 OutputFrame.cycle_index 冗余

- **现状**：五模块 `CycleResult` 均同时携带 `input_cycle_index`（本次输入周期号）与内嵌
  `OutputFrame.cycle_index`。两者关系随周期成败而变：
  1. 成功路径：两者均取自 `input.cycle_index`，数值恒等。
  2. 非执行路径（COMMON-OQ-5 已统一为不复用）：`output_frame.cycle_index` 保持默认 `0`、
     `input_cycle_index` 为本次输入号，二者分歧。
  [evidence: include/1q/electro_optical_sensor/EosCycleResult]
- **后果**：
  1. 双字段在成功路径冗余、在失败路径语义分裂，阅读者需判断何时相等何时分歧。
  2. 去重与周期失败语义耦合，无法独立处理。
- **待决问题**：
  1. 是否移除 `input_cycle_index`、统一用 `output_frame.cycle_index`。
  2. 或反向统一为仅保留 `input_cycle_index`。
- **当前边界**：五模块保留双字段。非执行周期中 `input_cycle_index` 承载"本次失败周期的归属号"语义，
  `output_frame.cycle_index` 为默认 `0`，不可简单删除。
- **再进入条件 (Stage A)**：评估单一周期号字段的可行性及其对 trace/replay 归属的影响。

### COMMON-OQ-8：周期输入时间/窗口字段无统一契约，违反时静默拒绝

- **现状**：三模块各自为政的周期时间/窗口校验，外部调用方违反时多表现为"静默不生效"而非显式错误。
  1. **AR 编年史校验**：拒绝 `window_start_time_s < 上一周期窗口结束`。
  2. **ESR 周期输入完整性**：要求 RF 帧的 `world_cycle_index`/`window_start_time_s`/`window_duration_s`
     与周期 input 精确相等，空帧也须填这三个窗口字段。
  3. **EOS 帧率-步长耦合**：拒绝 `dt_sec > 10/frame_rate_hz`，1 s 步长须 1 Hz 帧率。
  4. **配置侧不对称**：ESR 零值 `EsrSessionConfig{}` 不合法，而 AR/SBIRS 的 struct 默认即合法档位。
  [evidence: src/electronic_surveillance_radar/validation/EsrInputValidation]
- **后果**：调用方违反时整周期在决策消费点之前被静默拒绝，且无显式错误可供察觉。典型踩坑：
  1. 只递增 `cycle_index` 而忘记推进时间戳 → 外部覆盖即使被接受也从未应用，
     `applied_decision_source` 保持 `kNone`。
  2. RF 帧窗口字段不匹配 → 整周期被拒。
  3. 零值配置误用 → 首周期才暴露为 `kRejected`。
  已有 ar/esr 两个 consumer 教训。
- **待决问题**：
  1. 是否跨模块统一周期时间/窗口契约——共享"时间戳单调前进"与"RF 帧窗口匹配周期"的校验 helper，
     使违反在输入校验即显式可观测。
  2. 是否统一"零值配置"语义，或为 ESR 补 `kDefaultEsrSessionConfig` 显式默认常量。
  3. 各 `CycleInput` docstring 是否显式注明时间戳推进义务。
- **当前边界**：各模块保持现有校验。调用方必须：
  1. AR 推进 `cycle_start_time_s`（≥ 上一窗口结束）。
  2. ESR 填完整平台运动学 + 与周期匹配的 RF 帧窗口字段，并使用语义档位常量而非零值配置。
  3. EOS 保证 `dt_sec ≤ 10/frame_rate_hz`。
  不得在文档中宣称周期时间戳可任意重复，或零值配置为合法默认。
- **再进入条件 (Stage A)**：出现第二个真实消费方因时间戳未推进或 RF 帧窗口不匹配而静默失败（当前已有
  ar/esr 两个 consumer 教训），或跨模块集成要求统一周期时间契约时，先盘点四模块 CycleInput 的窗口字段与
  校验差异，设计共享校验与 docstring 警示，再评估跨模块推广。

## Airborne Radar 非阻塞边界

### AR-OQ-1：假目标鉴别跨域命名双轨

- **现状**：反假目标鉴别判据（同方向多脉冲列）跨两个域表达：
  1. **接收机观测域**：`ArInterferenceObservation.deception_class`（枚举），判据天然归属于此。
  2. **航迹生命周期域**：`TrackLifecycleManager::PromoteState` 消费该标注，量测域以
     `RawTrackMeasurement.classified_as_false_target`（bool）表达。
  `SignalCycleInput` 周期输入端口已收敛（旁路 mutable setter 已删除），不构成公开契约。
  [evidence: include/1q/airborne_radar/ArInterferenceObservation]
- **后果**：
  1. 同一"疑似假目标"概念跨域用了两套命名（枚举 vs bool），跨域阅读增加认知负担。
  2. 内部 `ArDeceptionMeasurementCandidate` 不进入 public result 或 replay，公开观测仍是唯一持久化事实。
- **待决问题**：是否在跨域标注契约收敛时统一两套命名。
- **当前边界**：`SignalCycleInput` 是内部 `ISignalPipeline` API，不构成公开契约；内部
  `ArDeceptionMeasurementCandidate` 不进入 public result 或 replay。两套命名保留，待跨域标注契约收敛时统一。
- **再进入条件 (Stage A)**：跨域标注契约进入冻结时，一并评估命名的统一成本与对 trace/replay 的影响。

## Electronic Surveillance Radar 非阻塞边界

### ESR-OQ-1：压制干扰感知与 ECCM 决策链路缺失

- **现状**：ECM 输出 `RfEmissionFrame` 经 `EsrCycleInput.rf_emissions` 进入 ESR，物理层干扰功率计算已完成
  （`EsrResolutionCellLedger` 将非最强信号功率作为 `interference_power_w` 叠加到 SNR 分母）。但 ESR 存在
  两条未消费的配置链路和三处能力缺口：
  1. `InterceptSuppressionModelConfig` 填充到 `InterceptPipelineConfig.suppression_model`，但
     `InterceptDetectionExecutor` 从未读取。
  2. `EsrEnvironmentSnapshot.spectrum_occupancy_ratio` 注释声称参与噪声计算，但代码不存在。
  3. 无结构化干扰观测输出（对比 AR 的 `ArInterferenceObservation`）。
  4. 无 ECCM 决策引擎（对比 AR 的 `EccmEvaluator` 评分-提案-执行架构）。
  5. 无工作模式自适应切换。
  [evidence: src/electronic_surveillance_radar/intercept/InterceptDetectionExecutor]
- **后果**：
  1. `suppression_model` 和 `spectrum_occupancy_ratio` 为死字段，外部据配置名可能误以为 ESR 具备压制干扰
     感知或自适应抗干扰能力。
  2. 无结构化观测输出使下游消费方无法获得干扰态势感知。
- **待决问题**：
  1. 是否激活死字段消费，将压制干扰对等效噪声底的影响纳入 SNR 和检测门限计算。
  2. 是否定义 `EsrInterferenceObservation` 结构化输出（bearing、频谱、J/N、deception_class）。
  3. 是否实现 ESR 特有的 ECCM 决策措施（接收机重调谐、扫描优先级调整、检测门限自适应、工作模式降级、
     欺骗标记与置信度降级）。
- **当前边界**：ECM 压制干扰在 ESR 接收端仅以通用 RF emission 身份参与分辨单元竞争和 SNR 计算。不产生
  结构化干扰观测输出，不触发 ECCM 反制措施。`suppression_model` 和 `spectrum_occupancy_ratio` 为死字段，
  不得在文档中声称 ESR 具备压制干扰感知或自适应抗干扰能力。
- **再进入条件 (Stage A)**：
  1. 先激活死字段消费（修改 `InterceptDetectionExecutor` 噪声计算），并提供 unit test 证明
     `suppression_noise_scale` 和 `spectrum_occupancy_ratio` 的变化可被检测结果观测到。
  2. 定义 `EsrInterferenceObservation` 公开类型并提供 unit test 覆盖。
  3. 实现 ECCM 评分与提案机制，提供集成测试覆盖"ECM 发射 → ESR 感知 → ECCM 反制 → 检测效果变化"全链路。

### ESR-OQ-2：运行时补丁扫描中心静默关闭显式扫描边界

- **现状**：`EsrRuntimeConfigResolver` 在应用 `has_scan_center_az_deg` 或 `has_scan_center_el_deg` 补丁时，
  会同时设置 `use_explicit_scan_bounds = false`，静默将扫描模式从显式边界切换为中心驱动。两个副作用：
  1. 用户仅调整扫描中心，不会预期丢失之前配置的四个扫描边界角。
  2. 单独补丁 azimuth center 也会导致 elevation 侧跟着切模式（副作用跨轴传播）。
  [evidence: src/electronic_surveillance_radar/session/EsrRuntimeConfigResolver]
- **后果**：用户难以预判 scan center 补丁会清空显式边界配置且跨轴生效，配置结果与意图偏离。
- **待决问题**：
  1. 是否将模式切换设为显式补丁字段（`has_use_explicit_scan_bounds`），而非由 scan center 补丁隐含触发。
  2. 或保留当前行为但增加返回值/日志提示。
- **当前边界**：当前行为为 scan center 补丁隐式关闭显式边界模式。消费方必须知晓此副作用。
- **再进入条件 (Stage A)**：出现真实场景要求"调整扫描中心但保留显式边界模式"，先评估将模式切换提取为
  独立补丁字段的 API 变更成本和向后兼容性。

### ESR-OQ-3：扫描策略跨域耦合（mission.scan + hardware mount 偏移）

- **现状**：`EsrScanPolicyConfig`（mission 域）的扫描字段经 `ApplyScanPolicy` 解算时会被 hardware 域偏移：
  1. `scan_center_az_deg` 减去 `EsrHardwareConfig::antenna_mount_az_deg`。
  2. `scan_start_az_deg` / `scan_end_az_deg` 在 `use_explicit_scan_bounds` 模式下也被 mount 偏移。
  mission 域的值被 hardware 域静默偏移。[evidence: src/electronic_surveillance_radar/session/EsrScanPolicyApplier]
- **后果**：用户只看 mission 配置无法推断实际扫描方向；mount 偏移在内部解算时扣除但文档未明确，集成时易误配。
- **待决问题**：是否在公开 API 中明确扫描中心的坐标系语义，有三个备选：
  1. 定义为"天线坐标系"（已含 mount 偏移）。
  2. 定义为"平台坐标系"（需显式减去 mount）。
  3. 提供查询实际解算扫描几何的 API。
- **当前边界**：扫描配置语义为"天线坐标系"，mount 偏移在内部解算时扣除。文档未明确说明此语义。
- **再进入条件 (Stage A)**：出现因 mount 偏移导致的集成问题或用户误配，先明确公开 API 的坐标系语义并在
  design.md 中固化，再评估是否需要查询 API。

## SBIRS 非阻塞边界

### SBIRS-OQ-1：诊断距离的物理语义

- **现状**：`SbirsDetectionAttributionRecord.estimated_range_m` 明确只属于 cue/诊断层，不代表被动红外测距
  能力。三模式取值来源不同：
  1. Strict/Estimated 使用真值距离。
  2. Sensor-like 使用真值距离叠加比例误差。
  [evidence: include/1q/sbirs_sensor/SbirsDetectionAttributionRecord]
- **后果**：字段名含 `estimated_range` 易被调用方误解为正式传感器测距输出，误用于滤波或定位。
- **待决问题**：字段名称和三模式取值来源是否足以防止调用方把它误解为正式传感器测距输出。
- **当前边界**：不得进入 `SbirsOutputFrame` raw output；消费方只能把它当作仿真归属与诊断辅助量。
- **再进入条件 (Stage A)**：出现真实下游消费者需要区分 truth-derived、filter-derived 或不可用距离，先盘点
  消费路径，再评估重命名、增加来源枚举或显式有效性字段。

### SBIRS-OQ-2：WFOV、Estimated 与 Sensor-like 的分阶段误差统计

- **现状**：三条用途使用独立随机子流，但共同读取 `SbirsErrorModelConfig` 的同一组角度/距离统计参数。
  [evidence: include/1q/sbirs_sensor/SbirsErrorModelConfig]
- **后果**：共享参数掩盖了三条链路在真实载荷上的精度差异，仿真结果可能高估某条链路的精度。
  1. WFOV 搜索。
  2. Estimated 校正量测。
  3. NFOV Sensor-like 输出。
- **待决问题**：是否需要分别表达三用途的精度等级（WFOV 搜索、Estimated 校正量测、NFOV Sensor-like 输出）。
- **当前边界**：共享参数是当前确定性简化，不得宣称代表真实 WFOV/NFOV 载荷精度差异。
- **再进入条件 (Stage A)**：取得可追溯的分阶段参数依据，或构造出共享参数无法满足的 SBIRS 场景验收矩阵后，
  再评估拆分配置；不得仅为形式完整扩大 public API。

### SBIRS-OQ-3：多目标随机样本与 scene 输入顺序

- **现状**：WFOV、Estimated、Sensor-like 各自是一条全局用途随机流。两个确定性边界：
  1. 同一 trace 可确定性 replay（相同输入字节和顺序）。
  2. 多目标在同一周期获得哪个样本取决于 `scene` 遍历顺序。
  [evidence: src/sbirs_sensor/SbirsSensorPipeline]
- **后果**：目标列表置换后，同一 `target_id` 可能获得不同的量测随机序列，跨场景对比或批量验证时结果不可复现。
- **待决问题**：SBIRS 是否需要保证目标列表置换后，每个 `target_id` 仍获得相同的量测随机序列。
- **当前边界**：replay 只保证相同输入字节和顺序的确定性，不承诺 scene permutation invariance。
- **再进入条件 (Stage A)**：外部场景源无法稳定排序，或批量验证明确要求按 target 不受输入顺序影响时，比较
  按 target/channel 派生子流与现有全局用途子流的 snapshot、热更和目标生命周期成本。

### SBIRS-OQ-4：Estimated 航迹的真值初始化

- **现状**：Estimated 航迹初始化分两个阶段：
  1. 首次捕获后用输入场景真值 ECEF 位置和速度初始化滤波均值。
  2. 后续才使用带误差角度量测。
  [evidence: src/sbirs_sensor/SbirsSensorPipeline]
- **后果**：`Estimated` 描述为生产仿真链，但首捕阶段含 truth-seeded 简化，若被当作完全无真值辅助的真实载荷
  跟踪器使用，会高估其无辅助条件下的起始性能。
- **待决问题**：是否需要改为仅由被动角度 cue 和显式距离/运动先验初始化，以形成无真值航迹起始链。
- **当前边界**：`Estimated` 是生产仿真链，但当前仍包含 truth-seeded track initiation 简化，不得描述为完全
  无真值辅助的真实载荷跟踪器。
- **再进入条件 (Stage A)**：先定义被动角度不可观测距离的初始化先验、收敛时间和失败判据，并提供与当前方案
  的捕获率、位置协方差、丢锁率及 replay 对比证据，再决定是否替换。
