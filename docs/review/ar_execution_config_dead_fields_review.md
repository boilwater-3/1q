# InternalExecutionConfig 字段映射覆盖审计

Status: draft

> 审计日期：2026-07-30
> 审计范围：`src/airborne_radar/config/InternalExecutionConfig.h` 全部字段
> 目的：识别未被映射层（`MapSessionToExecution` / `ApplyRuntimePatch`）填充的字段，确认其实际写入路径

## 审计结论速览

| 类别 | 字段数 | 说明 |
|------|--------|------|
| 映射层正常写入 | 12 | sensor_enabled, decision_control, detection.*, association.*, tracking.policy, tracking.engineering, lifecycle.policy, lifecycle.engineering, imm_model_noise_diff_coeffs |
| 运行时由其他模块写入 | 3 | platform_attitude_deg, enable_anti_vgpo_acceleration_bound, enable_anti_false_target_discrimination |
| 仅映射层 clear()，读时生成默认值 | 2 | imm_initial_weights, imm_transition_probability |
| **仅使用硬编码默认值，映射层从未写入** | **7** | kalman_noise_diff_coeff, anti_vgpo_max_acceleration_mps2, track_pool_initial_chunk, track_pool_max_chunks, control_profile_effects, imm_activation_policy, track_pool_thread_safety_mode |

## 字段逐项审计

### 1. `platform_attitude_deg` (DetectionExecutionConfig)

| 属性 | 值 |
|------|-----|
| 类型 | `config::PlatformAttitudeDeg` |
| 默认值 | `{}` |
| 映射层写入 | **无** |
| 其他写入 | `SignalPipeline::UpdatePlatformAttitude()` (`SignalPipeline.cpp:279`) — 每周期运行时更新 |
| 读取 | `DetectionExecution::Run()` (`DetectionExecution.cpp:216`) — 传给 `BeamControlResolver::Resolve` |
| **结论** | **保留**。运行时由 signal pipeline 每周期写入，非配置映射职责。建议在字段声明处添加注释说明写入路径。 |

### 2. `kalman_noise_diff_coeff` (TrackingExecutionConfig)

| 属性 | 值 |
|------|-----|
| 类型 | `float` |
| 默认值 | `1.0f` |
| 映射层写入 | **无** |
| 其他写入 | **无** |
| 读取 | `SignalComponentFactory.cpp:51,65,159` — 传给关联器和预测器配置；`RuntimeAssemblySupport.cpp:167` — 运行时组装签名 |
| **结论** | **标记 TODO**。当前仅使用默认值 1.0f，从未被任何配置路径覆盖。如果是内部算法常量，应从配置结构中移除并改为常量；如果是预留可配置项，应在映射层添加写入路径。 |

### 3. `enable_anti_vgpo_acceleration_bound` (InternalExecutionConfig)

| 属性 | 值 |
|------|-----|
| 类型 | `bool` |
| 默认值 | `false` |
| 映射层写入 | **无** |
| 其他写入 | `ControlProfileEffects::ApplyControlProfileToConfig()` (`ControlProfileEffects.cpp:109`) — 每周期从 `session::ArControlProfile` 写入 |
| 读取 | `SignalComponentFactory.cpp:31,33` — 传给生命周期管理器 |
| **结论** | **保留**。运行时由控制策略评估模块写入，非配置映射职责。建议添加注释说明写入路径。 |

### 4. `anti_vgpo_max_acceleration_mps2` (InternalExecutionConfig)

| 属性 | 值 |
|------|-----|
| 类型 | `double` |
| 默认值 | `100.0` |
| 映射层写入 | **无** |
| 其他写入 | **无** |
| 读取 | `SignalComponentFactory.cpp:34` — 三元表达式中当 `enable_anti_vgpo_acceleration_bound` 为 true 时使用 |
| **结论** | **标记 TODO**。当前仅使用默认值 100.0，从未被任何配置路径覆盖。与 `enable_anti_vgpo_acceleration_bound` 配套使用，但该开关由控制策略模块写入，而此阈值始终为默认值。应考虑：(a) 由控制策略模块一并写入；(b) 改为常量；(c) 在映射层添加写入路径。 |

### 5. `enable_anti_false_target_discrimination` (InternalExecutionConfig)

| 属性 | 值 |
|------|-----|
| 类型 | `bool` |
| 默认值 | `false` |
| 映射层写入 | **无** |
| 其他写入 | `ControlProfileEffects::ApplyControlProfileToConfig()` (`ControlProfileEffects.cpp:111`) — 每周期从 `session::ArControlProfile` 写入 |
| 读取 | `SignalComponentFactory.cpp:29` — 传给生命周期管理器 |
| **结论** | **保留**。运行时由控制策略评估模块写入，非配置映射职责。建议添加注释说明写入路径。 |

### 6. `imm_initial_weights` (LifecycleExecutionConfig)

| 属性 | 值 |
|------|-----|
| 类型 | `std::vector<float>` |
| 默认值 | `{}` (空) |
| 映射层写入 | `MappingTransforms.h:61` — 仅 `clear()`（当 IMM 禁用时清空） |
| 其他写入 | **无** |
| 读取 | `ImmMatrixDefaults.cpp:77,81,83,89` — 空时生成均匀分布默认值，非空时使用用户值 |
| **结论** | **保留但添加注释**。当前为"可选覆盖"模式：提供值则使用，空则生成默认值。但从未有任何路径向此字段写入值。如果是预留的高级配置入口，应在文档中说明；如果不需要自定义，可考虑移除并始终使用默认生成。 |

### 7. `imm_transition_probability` (LifecycleExecutionConfig)

| 属性 | 值 |
|------|-----|
| 类型 | `std::vector<float>` |
| 默认值 | `{}` (空) |
| 映射层写入 | `MappingTransforms.h:62` — 仅 `clear()`（当 IMM 禁用时清空） |
| 其他写入 | **无** |
| 读取 | `ImmMatrixDefaults.cpp:39,46,48,56` — 空时生成均匀转移概率默认值，非空时使用用户值 |
| **结论** | **保留但添加注释**。与 `imm_initial_weights` 同属"可选覆盖"模式。当前无路径写入自定义转移概率矩阵。 |

### 8. `track_pool_initial_chunk` (LifecycleExecutionConfig)

| 属性 | 值 |
|------|-----|
| 类型 | `std::size_t` |
| 默认值 | `64` |
| 映射层写入 | **无** |
| 其他写入 | **无** |
| 读取 | `SignalComponentFactory.cpp:81` — 传给 `BoostTrackPool` 构造函数 |
| **结论** | **标记 TODO**。仅使用默认值 64，从未被覆盖。如果是性能调优常量，应从配置结构中移除；如果是可配置项，应在映射层或 runtime patch 中添加写入路径。 |

### 9. `track_pool_max_chunks` (LifecycleExecutionConfig)

| 属性 | 值 |
|------|-----|
| 类型 | `std::size_t` |
| 默认值 | `256` |
| 映射层写入 | **无** |
| 其他写入 | **无** |
| 读取 | `SignalComponentFactory.cpp:82` — 传给 `BoostTrackPool` 构造函数 |
| **结论** | **标记 TODO**。与 `track_pool_initial_chunk` 同属 track pool 容量参数，仅使用默认值。 |

### 10. `control_profile_effects` (InternalExecutionConfig)

| 属性 | 值 |
|------|-----|
| 类型 | `ControlProfileEffectsConfig` |
| 默认值 | `{sidelobe_level_reduction_db=6.0, adaptive_beam_gain_boost_db=2.0, adaptive_beamwidth_scale=0.60, lpi_beamwidth_scale=0.75}` |
| 映射层写入 | **无** |
| 其他写入 | **无** |
| 读取 | `ControlProfileEffects.cpp:63` — const 引用读取 |
| **结论** | **标记 TODO**。仅使用硬编码默认值。这些参数描述控制策略的天线/波束增益效果，从未被配置或运行时路径覆盖。如果当前阶段是固定常量，应从配置结构中移除并改为 `constexpr`；如果是预留可配置项，应添加映射路径。 |

### 11. `imm_activation_policy` (LifecycleExecutionConfig)

| 属性 | 值 |
|------|-----|
| 类型 | `signal::tracking::ImmActivationPolicy` |
| 默认值 | `kConfirmedTracksOnly` |
| 映射层写入 | **无** |
| 其他写入 | **无** |
| 读取 | `SignalComponentFactory.cpp:27` — 传给生命周期管理器 |
| **结论** | **标记 TODO**。仅使用默认值 `kConfirmedTracksOnly`。如果是固定策略，应从配置结构中移除；如果是可配置项，应在映射层添加写入路径。 |

### 12. `track_pool_thread_safety_mode` (LifecycleExecutionConfig)

| 属性 | 值 |
|------|-----|
| 类型 | `signal::tracking::TrackPoolThreadSafetyMode` |
| 默认值 | `kSingleThreadNoLock` |
| 映射层写入 | **无** |
| 其他写入 | **无** |
| 读取 | `SignalComponentFactory.cpp:28` — 传给生命周期管理器；`RuntimeAssemblySupport.cpp:41` — 运行时组装签名 |
| **结论** | **标记 TODO**。仅使用默认值 `kSingleThreadNoLock`。线程安全模式通常由部署环境决定（单线程 vs 多线程），应考虑在 `ArSessionConfig` 或 runtime patch 中暴露配置入口。 |

## 建议行动

### 需要添加写入路径（7 个字段）

| 字段 | 建议 |
|------|------|
| `kalman_noise_diff_coeff` | 在 `ArPolicyConfig::TrackingConfig` 中添加此字段，通过映射层写入；或改为 `constexpr` 常量 |
| `anti_vgpo_max_acceleration_mps2` | 由 `ControlProfileEffects::ApplyControlProfileToConfig()` 一并写入，与 `enable_anti_vgpo_acceleration_bound` 配套 |
| `track_pool_initial_chunk` | 在 `ArPolicyConfig::LifecycleConfig` 中添加此字段，或改为 `constexpr` 常量 |
| `track_pool_max_chunks` | 同上 |
| `control_profile_effects` | 在 `ArPolicyConfig` 中添加 `ControlProfileEffectsConfig` 子配置，或改为 `constexpr` 常量 |
| `imm_activation_policy` | 在 `ArPolicyConfig::LifecycleConfig` 中添加此字段，或改为 `constexpr` 常量 |
| `track_pool_thread_safety_mode` | 在 `ArSessionConfig` 中暴露配置入口（部署相关），或改为 `constexpr` 常量 |

### 需要添加注释（3 个字段）

| 字段 | 注释内容 |
|------|----------|
| `platform_attitude_deg` | "由 SignalPipeline::UpdatePlatformAttitude() 每周期写入，不经过 MapSessionToExecution" |
| `enable_anti_vgpo_acceleration_bound` | "由 ControlProfileEffects::ApplyControlProfileToConfig() 每周期从 ArControlProfile 写入" |
| `enable_anti_false_target_discrimination` | "同上" |

### 需要评估保留（2 个字段）

| 字段 | 评估 |
|------|------|
| `imm_initial_weights` | 当前为"可选覆盖"模式但无路径写入。如果不需要自定义初始权重，移除此字段，让 `ImmMatrixDefaults` 始终生成默认值 |
| `imm_transition_probability` | 同上 |
