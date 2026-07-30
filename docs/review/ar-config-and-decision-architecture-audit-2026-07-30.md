# AR 模块配置全链路审计 & 决策架构研究报告

Status: draft

> **审计日期**: 2026-07-30
> **审计范围**: `airborne_radar` 模块公共门面 (`ArSession`) 下四域配置模型的全部字段，以及决策子系统架构
> **审计方法**: 字段级追踪 — 从公共配置头文件 → 映射层 (`SessionToExecutionMapper` / `RuntimePatchMapper`) → 内部工程配置 → 实际计算代码

---

## 一、四域配置全链路审计总览

| 配置域 | 字段总数 | LIVE | DEAD | 已修复 | 存活率 |
|--------|---------|------|------|--------|--------|
| **Hardware** (`ArHardwareConfig`) | 44 | 44 | 0 | 1 (移除) | 100% |
| **Mission** (`ArMissionConfig` + `ArOrientationConfig`) | 10 | 10 | 0 | 0 | 100% |
| **Policy** (`ArPolicyConfig`) | 22 | 22 | 0 | 1 (激活) | 100% |
| **Environment** (`ArEnvironmentConfig`) | 6 | 6 | 0 | 0 | 100% |
| **合计** | **82** | **82** | **0** | **2** | **100%** |

---

## 二、死配置字段清单

### 2.1 `receiver.polarization` (Hardware 域) — ✅ 已移除

| 属性 | 值 |
|------|-----|
| **字段** | `ArHardwareConfig.receiver.polarization` |
| **类型** | `RfPolarization` (枚举, 默认 `kHorizontal`) |
| **状态** | ✅ **已移除** — 字段、映射、验证、序列化和测试引用全部清理 |
| **根因** | 被 `receiver.scene_polarization` (`RfScenePolarization` 枚举) 取代。RF Scene v2 API 使用 `scene_polarization` 进行极化相关损耗计算。 |

### 2.2 `detection.minimum_detection_margin_db` (Policy 域) — ✅ 已激活

| 属性 | 值 |
|------|-----|
| **字段** | `ArPolicyConfig.detection.minimum_detection_margin_db` |
| **类型** | `float` (默认 `-2.0f`) |
| **映射状态** | ✅ 已映射 — `EngineeringResolvers.h:30` → `engineering::DetectionConfig.min_detection_margin_db` |
| **运行时补丁** | ✅ 可热补丁 — `RuntimePatchMapper.cpp:73,181` 往返映射 |
| **计算消费** | ✅ **已激活** — `SignalDetector::Detect()` 和 `DetectResolvedCell()` 在 Monte Carlo 判决后执行可靠性裕量门限：`snr_db < min_detection_margin_db` 时强制 `detected=false`。`DetectionExecution.cpp:290` 的 `detection_margin_db` 缓冲区语义从原始 SNR 改为相对裕量 `snr_db - min_detection_margin_db`。 |

---

## 三、各域详细审计结果

### 3.1 Hardware 域 (`ArHardwareConfig`)

#### 3.1.1 TransmitterConfig (`transmitter`) — 11 字段，全部 LIVE

| # | 字段 | 类型 | 消费位置 | 状态 |
|---|------|------|---------|------|
| 1 | `equipment_id` | `uint64` | `ArEmissionFactory.cpp:58` | LIVE |
| 2 | `peak_power_w` | `float` | `RadarEquations.cpp:102`, `ArEmissionFactory.cpp:80` | LIVE |
| 3 | `frequency_hz` | `float` | `RadarEquations.cpp:100`, `DetectionExecution.cpp:56/87/188` | LIVE |
| 4 | `bandwidth_hz` | `float` | `RadarEquations.cpp:130`, `DetectionExecution.cpp:286`, `ArReceiverStateBuilder.cpp:43` | LIVE |
| 5 | `pulse_width_s` | `float` | `RadarEquations.cpp:64/67`, `ArEmissionFactory.cpp:83-84/93` | LIVE |
| 6 | `prf_hz` | `float` | `ControlProfileEffects.cpp:32`, `ArSession.cpp:507/511` | LIVE |
| 7 | `transmit_loss_db` | `float` | `RadarEquations.cpp:111`, `ArEmissionFactory.cpp:90` | LIVE |
| 8 | `maximum_peak_power_w` | `float` | `ArEmissionFactory.cpp:86` | LIVE |
| 9 | `maximum_duty_cycle` | `float` | `ArSessionConfigBuilder.cpp:245/249-250` (验证门限) | LIVE |
| 10 | `maximum_pulse_energy_j` | `float` | `ArEmissionFactory.cpp:83-84` | LIVE |
| 11 | `frequency_plan_hz` | `vector<double>` | `ArSession.cpp:507/527/529/531` | LIVE |

#### 3.1.2 AntennaConfig (`antenna`) — 7 字段 + 6 嵌套字段，全部 LIVE

| # | 字段 | 类型 | 消费位置 | 状态 |
|---|------|------|---------|------|
| 12 | `main_beam_gain_db` | `float` | `BeamControlResolver.h:44/85`, `ArEmissionFactory.cpp:65` | LIVE |
| 13 | `nominal_az_beamwidth_deg` | `float` | `BeamwidthResolution.h:61`, `ArEmissionFactory.cpp:66-67` | LIVE |
| 14 | `nominal_el_beamwidth_deg` | `float` | `BeamwidthResolution.h:62`, `ArEmissionFactory.cpp:66-67` | LIVE |
| 15 | `antenna_length_m` | `float` | `BeamwidthResolution.h:65-68`, `BeamControlResolver.h:57/101` | LIVE |
| 16 | `antenna_width_m` | `float` | `BeamwidthResolution.h:70-73`, `BeamControlResolver.h:57/101` | LIVE |
| 17 | `pattern` | `AntennaPatternConfig` | (嵌套，见下) | LIVE |
| 18 | `enable_directional_pattern` | `bool` | `BeamControlResolver.h:45/87` | LIVE |
| 19 | `pattern.model_type` | `AntennaPatternModelType` | `AntennaPatternRuntime.h:102` | LIVE |
| 20 | `pattern.max_sidelobe_level_db` | `float` | `AntennaPatternRuntime.h:209`, `ArEmissionFactory.cpp:68-69` | LIVE |
| 21 | `pattern.backlobe_level_db` | `float` | `AntennaPatternRuntime.h:197`, `ArEmissionFactory.cpp:70-71` | LIVE |
| 22 | `pattern.scan_loss_coeff_db_per_deg2` | `float` | `AntennaPatternRuntime.h:170` | LIVE |
| 23 | `pattern.max_scan_loss_db` | `float` | `AntennaPatternRuntime.h:173-174` | LIVE |
| 24 | `pattern.boresight_offset_deg` | `AzimuthElevationDeg` | `AntennaPatternRuntime.h:167-168` | LIVE |

#### 3.1.3 ReceiverConfig (`receiver`) — 12 字段，全部 LIVE（`polarization` 已移除）

| # | 字段 | 类型 | 消费位置 | 状态 |
|---|------|------|---------|------|
| 25 | `equipment_id` | `uint64` | `ArReceiverStateBuilder.cpp:18` | LIVE |
| 26 | `noise_figure_db` | `float` | `RadarEquations.cpp:129`, `DetectionExecution.cpp:259` | LIVE |
| 27 | `receive_loss_db` | `float` | `SignalDetector.cpp:36`, `DetectionExecution.cpp:257` | LIVE |
| **28** | **`polarization`** | **`RfPolarization`** | **仅验证+序列化，无计算消费** | **DEAD** |
| 29 | `cross_polarization_isolation_db` | `float` | `ArEmissionFactory.cpp:72-73`, `RfLinkBudget.cpp:142-143` | LIVE |
| 30 | `minimum_far_field_range_m` | `float` | `ArReceiverStateBuilder.cpp:39-40`, `RfScene.cpp:564` | LIVE |
| 31 | `has_co_site_isolation` | `bool` | `RfLinkBudget.cpp:258` | LIVE |
| 32 | `co_site_isolation_db` | `float` | `RfLinkBudget.cpp:286` | LIVE |
| 33 | `maximum_linear_input_power_w` | `float` | `ArReceiverStateBuilder.cpp:45-46`, `ArRfFrontEndResolver.cpp:52` | LIVE |
| 34 | `preselector_bandwidth_hz` | `float` | `ArReceiverStateBuilder.cpp:37` | LIVE |
| 35 | `interference_observation_jn_gate_db` | `float` | `ArSession.cpp:599`, `ArInterferenceObservationResolver.cpp:214` | LIVE |
| 36 | `scene_polarization` | `RfScenePolarization` | `ArReceiverStateBuilder.cpp:33`, `ArEmissionFactory.cpp:74` | LIVE |
| 37 | `co_site_paths` | `vector<RfCoSiteIsolationPath>` | `ArReceiverStateBuilder.cpp:41`, `RfScene.cpp:248/264` | LIVE |

#### 3.1.4 RcsPhysicsConfig (`rcs_physics`) — 8 字段，全部 LIVE

| # | 字段 | 类型 | 消费位置 | 状态 |
|---|------|------|---------|------|
| 38 | `enable_physical_rcs` | `bool` | `DetectionExecution.cpp:78` | LIVE |
| 39 | `physics_mix_ratio` | `float` | `DetectionExecution.cpp:82/126` | LIVE |
| 40 | `cylinder_weight` | `float` | `DetectionExecution.cpp:117-119` | LIVE |
| 41 | `min_equivalent_radius_m` | `float` | `DetectionExecution.cpp:64` | LIVE |
| 42 | `max_equivalent_radius_m` | `float` | `DetectionExecution.cpp:65` | LIVE |
| 43 | `min_rcs_m2` | `float` | `DetectionExecution.cpp:122` | LIVE |
| 44 | `max_rcs_m2` | `float` | `DetectionExecution.cpp:123` | LIVE |
| 45 | `bistatic_psi_offset_deg` | `float` | `DetectionExecution.cpp:108` | LIVE |

---

### 3.2 Mission 域 (`ArMissionConfig` + `ArOrientationConfig`)

**10 字段，全部 LIVE，无死配置。**

| # | 字段 | 类型 | 消费位置 | 状态 |
|---|------|------|---------|------|
| 1 | `power_on` | `bool` | `SignalPipeline.cpp:120`, `ArSession.cpp:498` | LIVE |
| 2 | `orientation.mount_angles_deg` | `EulerAnglesDeg` | `BeamControlResolver.h:139-141`, `ArOrientationUtils.h:209/244` | LIVE |
| 3 | `orientation.scan_center_deg` | `AzimuthElevationDeg` | `ScanScheduleResolver.cpp:58`, `BeamControlResolver.h:127-129` | LIVE |
| 4 | `orientation.mechanical_scan_limits_deg` | `AzimuthElevationLimitsDeg` | `ScanScheduleResolver.cpp:172`, `BeamControlResolver.h:119` | LIVE |
| 5 | `orientation.electronic_scan_limits_deg` | `AzimuthElevationLimitsDeg` | `ScanScheduleResolver.cpp:172`, `BeamControlResolver.h:120` | LIVE |
| 6 | `orientation.scan_start_position` | `ScanStartPosition` | `ScanScheduleResolver.cpp:219` | LIVE |
| 7 | `orientation.scan_sequence` | `ScanSequence` | `ScanScheduleResolver.cpp:220` | LIVE |
| 8 | `orientation.work_mode` | `ArWorkMode` | `ScanScheduleResolver.cpp:186/194/205/206/241` | LIVE |
| 9 | `orientation.commanded_beamwidth_enabled` | `bool` | `ScanScheduleResolver.cpp:27`, `BeamwidthResolution.h:53`, `ControlProfileEffects.cpp:98` | LIVE |
| 10 | `orientation.commanded_beamwidth_deg` | `CommandedBeamwidthDeg` | `ScanScheduleResolver.cpp:29-35`, `BeamwidthResolution.h:55-57` | LIVE |
| 11 | `orientation.stabilization_mode` | `StabilizationMode` | `BeamControlResolver.h:121` | LIVE |

> 注: `ArOrientationConfig` 嵌套在 `ArMissionConfig.orientation` 下，共 10 个子字段 + `power_on` = 11 个叶子节点，但 `orientation` 本身作为聚合计为 1 个顶层字段，总计 10 个顶层字段。

---

### 3.3 Policy 域 (`ArPolicyConfig`)

**22 字段，1 个 DEAD。**

#### 3.3.1 ArDetectionPolicyConfig (`detection`) — 4 字段

| # | 字段 | 类型 | 消费位置 | 状态 |
|---|------|------|---------|------|
| 1 | `minimum_snr_db` | `float` | `SignalDetector.cpp:61/86` | LIVE |
| 2 | `pfa` | `float` | `SignalDetector.cpp:57/84`, `RadarEquations.cpp:165-177/268-280` | LIVE |
| 3 | `pulse_count` | `int` | `DetectionExecution.cpp:226/268`, `ControlProfileEffects.cpp:31/71-72` | LIVE |
| **4** | **`minimum_detection_margin_db`** | **`float`** | **已映射但从未消费** | **DEAD** |

#### 3.3.2 BeamControlConfig (`beam_control`) — 4 字段，全部 LIVE

| # | 字段 | 类型 | 消费位置 | 状态 |
|---|------|------|---------|------|
| 5 | `pointing.nominal_beamwidth_deg` | `CommandedBeamwidthDeg` | `ScanScheduleResolver.cpp:19` | LIVE |
| 6 | `scheduler.azimuth_step_count_hint` | `uint32` | `ScanScheduleResolver.cpp:214` | LIVE |
| 7 | `scheduler.elevation_step_count_hint` | `uint32` | `ScanScheduleResolver.cpp:217` | LIVE |
| 8 | `scheduler.prefer_dense_tas_sampling` | `bool` | `ScanScheduleResolver.cpp:207` | LIVE |

#### 3.3.3 AssociationConfig (`association`) — 1 字段，LIVE

| # | 字段 | 类型 | 消费位置 | 状态 |
|---|------|------|---------|------|
| 9 | `distance_gate_sigma` | `float` | `DataAssociation.cpp:125/216/223/227/248/339/348/352/383` (平方后作为 `unassigned_cost`) | LIVE |

#### 3.3.4 TrackingConfig (`tracking`) — 4 字段，全部 LIVE

| # | 字段 | 类型 | 消费位置 | 状态 |
|---|------|------|---------|------|
| 10 | `enable_kalman_filter` | `bool` | `SignalComponentFactory.cpp:63/157` | LIVE |
| 11 | `kalman_measurement_noise_std` | `float` | `SignalComponentFactory.cpp:52-53/69/126/163`, `DetectionExecution.cpp:296` 等 | LIVE |
| 12 | `speed_decay_ratio_on_loss` | `float` | `TrackFilter.cpp:78` | LIVE |
| 13 | `rcs_decay_ratio_on_loss` | `float` | `TrackFilter.cpp:79` | LIVE |

#### 3.3.5 LifecycleConfig (`lifecycle`) — 5 字段，全部 LIVE

| # | 字段 | 类型 | 消费位置 | 状态 |
|---|------|------|---------|------|
| 14 | `confirm_hits` | `uint32` | `TrackLifecycleManager.cpp:708` | LIVE |
| 15 | `max_miss_before_lost` | `uint32` | `TrackLifecycleManager.cpp:716-717` | LIVE |
| 16 | `max_lost_cycles` | `uint32` | `TrackLifecycleManager.cpp:726` | LIVE |
| 17 | `enable_imm_lifecycle` | `bool` | `SignalComponentFactory.cpp:91`, `RuntimeAssemblySupport.cpp:39/47/57` | LIVE |
| 18 | `model_count_hint` | `uint32` | `SessionToExecutionMapper.cpp:28-31` (生成 IMM 系数向量) | LIVE |

#### 3.3.6 DecisionControlConfig (`decision_control`) — 4 字段，全部 LIVE

| # | 字段 | 类型 | 消费位置 | 状态 |
|---|------|------|---------|------|
| 19 | `lpi_hold_cycles_after_request` | `uint32` | `ControlReducer.cpp:378/456` | LIVE |
| 20 | `eccm_hold_cycles_after_request` | `uint32` | `ControlReducer.cpp:380/478` | LIVE |
| 21 | `lpi_cooldown_cycles_after_release` | `uint32` | `ControlReducer.cpp:382/472` | LIVE |
| 22 | `eccm_cooldown_cycles_after_release` | `uint32` | `ControlReducer.cpp:384/494` | LIVE |

---

### 3.4 Environment 域 (`ArEnvironmentConfig`)

**6 字段，全部 LIVE，无死配置。**

| # | 字段 | 类型 | 消费位置 | 状态 |
|---|------|------|---------|------|
| 1 | `atmospheric_physics.enable_physical_model` | `bool` | `DetectionExecution.cpp:46` | LIVE |
| 2 | `atmospheric_physics.pressure_hpa` | `float` | `DetectionExecution.cpp:51`, `AtmosphericTypes.h:40` | LIVE |
| 3 | `atmospheric_physics.temperature_k` | `float` | `DetectionExecution.cpp:52`, `AtmosphericTypes.h:39` | LIVE |
| 4 | `atmospheric_physics.relative_humidity` | `float` | `DetectionExecution.cpp:53`, `AtmosphericTypes.h:41` | LIVE |
| 5 | `vegetation_scatter_physics.cover_profile` | `VegetationCoverProfile` | `PropagationModel.cpp:110` | LIVE |
| 6 | `vegetation_scatter_physics.enable_physical_model` | `bool` | `PropagationModel.cpp:161` | LIVE |

> **架构观察**: Environment 域有意绕过 `SessionToExecutionMapper` / `InternalExecutionConfig`，走独立路径 `EnvironmentService → EnvironmentSnapshot → 信号管线`。这是正确的设计 — 环境域有独立的冻结快照机制，不应混入执行配置。

---

## 四、决策架构深度研究

### 4.1 当前决策数据流

```
周期 N 执行:
  SignalPipeline.RunCycle()
    → 产出 DecisionInputFrame (航迹、干扰观测、质量指标)

  TacticalCoordinator.Evaluate(decision_frame, state_store)
    → ThreatAssessmentEvaluator: 威胁分类，产出 LpiSourceInfo
    → LpiEvaluator: 若检测到高威胁侦察平台，产出 LPI 提案
    → EccmEvaluator: 若存在 RF 干扰观测，产出 ECCM 提案
    → 返回 TacticalDecisionResult { proposals, classifications }

  ArController 存储:
    pending_internal_proposals = decision_result.proposals
    latest_decision_observation = { input_frame, active_control_profile }

── 周期间隔：外部模块可调用 SubmitExternalDecision() ──

周期 N+1 准备阶段:
  ApplyPendingDecisionControl():
    if has_pending_external_decision:
      selected_proposals = pending_external_decision.proposals    // 外部提案整体替换
    else:
      selected_proposals = pending_internal_proposals             // 使用原生提案

    ControlReducer.Reduce(previous_profile, selected_proposals)
      → 按优先级排序 → 验证 → 应用 LPI/ECCM 指令 → 管理 hold/cooldown → 解决跨域冲突
      → 产出 ArControlProfile

    ControlProfileEffects.ApplyControlProfileToConfig(profile, config)
      → 直接修改 ExecutionConfig 的运行时参数
```

### 4.2 关键发现：整体替换而非合并

**这是最核心的架构问题。** 在 `ArController::ApplyPendingDecisionControl()` 中：

```cpp
const auto& selected = has_pending_external_decision
    ? pending_external_decision.proposals
    : pending_internal_proposals;
```

外部决策 **完全替换** 原生决策，而非合并或覆盖。这意味着：
- 如果外部模块只提交了 LPI 提案，所有原生 ECCM 提案将被丢弃
- 外部模块必须重新提出它想要保留的一切，包括原生系统已检测到的必要反干扰措施
- 这不是 "外部覆盖特定参数" 的模型，而是 "外部接管全部决策权" 的模型

### 4.3 `SubmitExternalDecision` 的反直觉问题

用户的直觉是正确的。当前设计存在以下反直觉层面：

#### 问题 1：命名与语义不匹配

外部决策被称为 "proposal"（提案），与原生 `TacticalCoordinator` 的输出使用完全相同的 `TacticalProposal` 类型。但实际上，外部模块是被授权的决策者，不是 "建议者"。"提案" 暗示可被拒绝，但外部决策在替换模型下是权威性的（除了 cooldown 限制）。

#### 问题 2：hold/cooldown 对外部决策同样生效

`ControlReducer` 不区分决策来源。如果原生系统刚释放 ECCM 且 cooldown 计时器激活，外部模块的 ECCM 提案同样会被 cooldown 拒绝。外部模块无法覆盖 cooldown。这违反了 "外部权威" 的直觉。

#### 问题 3：外部模块无法设置 "无操作" 或 "域级基线"

如果外部模块只想控制 LPI 而不想干预 ECCM，它必须提交空的 ECCM 提案集来显式声明 "ECCM 域由我控制，但我不激活它"。如果不提交 ECCM 部分，原生 ECCM 提案会被丢弃（因为整体替换）。这要求外部模块理解所有域。

#### 问题 4：观测/响应协议耦合周期时序

外部模块必须在下一个 `Step` 调用之前响应特定的 `DecisionObservation`（由 `cycle_index` + `batch_id` 标识）。如果外部模块响应慢，原生决策自动生效。这意味着 "外部控制" 本质上是尽力而为的、延迟敏感的，可能不符合 "外部决策应为权威" 的预期。

#### 问题 5：优先级字段对外部决策基本无效

由于外部决策完全替换原生决策，同一时刻只有一种来源的提案。`ControlReducer` 内的优先级排序对外部提案几乎没有实际意义（除非同一提交内有冲突的指令类型，而这被验证为重复而拒绝）。

#### 问题 6：`ArControlProfile` 无溯源标记

一旦 `ControlReducer` 产出 profile，没有字段标识哪些指令来自原生、哪些来自外部。`last_applied_decision_source` 字段仅在控制器层面跟踪周期级来源（internal/external），但 profile 本身不含指令级溯源。这增加了调试和审计难度。

### 4.4 关于 "LPI/ECCM 决策控制放在 Policy 域" 的分析

用户认为 Policy 域中的 `DecisionControlPolicyConfig`（hold/cooldown 周期数）位置不太自然。分析如下：

**当前归属的合理性**：hold/cooldown 是决策行为的策略约束 — "决策激活后保持多久"、"释放后冷却多久"。这确实是一种 "策略"，放在 Policy 域有一定道理。

**反直觉之处**：
- `DecisionControlPolicyConfig` 控制的是决策引擎的行为时序，而非信号处理或检测参数
- 它与 `DetectionPolicyConfig`、`TrackingPolicyConfig` 等 "本体参数" 性质不同 — 后者控制物理/算法行为，前者控制决策状态机
- 从外部注入者的视角，hold/cooldown 是决策引擎的内部实现细节，不应暴露为顶层配置

### 4.5 决策架构改进 — ✅ 已实施方案 B

**方案 B：分离原生/外部路径** 已实施并合入。核心变更：

1. 外部决策不再走 `TacticalProposal`，改为通过 `ExternalDecisionOverride` 回调直接操作 `ArControlProfile`
2. `ControlReducer` 分为两个阶段：原生归约（含 hold/cooldown）+ 外部覆盖（无 hold/cooldown）
3. 外部模块接收当前 `ArControlProfile` 作为输入，返回修改后的 `ArControlProfile`
4. hold/cooldown 仅约束原生路径，外部覆盖完全绕过
5. `ControlDirective`、`ControlDirectiveType`、`ControlDirectiveSource`、`TacticalProposal` 已从公共 API 移入内部
6. 旧 `ExternalDecisionResponse` 类型和 `SubmitExternalDecision(ExternalDecisionResponse)` 已删除
7. 外部覆盖无 cycle/batch 时序耦合

---

## 五、总结与行动建议

### 5.1 配置审计结论

- **82 个字段全部存活**，配置体系健康
- 2 个死配置已处理：
  - `receiver.polarization` — ✅ 已移除
  - `detection.minimum_detection_margin_db` — ✅ 已激活（可靠性裕量门限）

### 5.2 决策架构结论

方案 B 已实施，核心矛盾已解决：外部决策从 "提案" 提升为 "覆盖"，通过回调直接操作 `ArControlProfile`，
不再与原生提案共享类型或管线。原生决策链路的 hold/cooldown 完整性不受外部覆盖影响。
