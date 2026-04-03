# EOS / ESR 模块代码质量审查报告

> **审查日期**：2026-04-03
> **审查范围**：`include/1q/electro_optical_sensor/`、`src/electro_optical_sensor/`、
>              `include/1q/electronic_surveillance_radar/`、`src/electronic_surveillance_radar/`
> **审查重点**：代码质量、数值稳定性、内存安全、API 设计、算法正确性（不含文档）

---

## 一、总体评价

| 模块 | 规模 | 综合评级 | 说明 |
|------|------|----------|------|
| EOS（光电传感器） | ~3,100 行 | **B** | 物理模型清晰，但数值稳定性有明显缺陷，核心函数过度复杂 |
| ESR（电子侦察雷达） | ~5,500 行 | **B+** | 架构清晰，算法设计合理，但数值安全和类型转换存在隐患 |

---

## 二、EOS 模块问题（electro_optical_sensor）

### 问题优先级汇总

| ID | 严重性 | 类别 | 文件 | 描述 |
|----|--------|------|------|------|
| EOS-R1 | HIGH | 数值稳定性 | EosRadiometry.cpp:42-55 | Planck 公式 lambda5 NaN 检查不足 |
| EOS-R4 | HIGH | 数值稳定性 | EosStrayLight.cpp:58-60 | 角度分离分母可能为零 |
| EOS-R6 | HIGH | 数值稳定性 | EosOpticalCharacteristics.cpp:52-56 | 地面投影角度未限制 ±90° |
| EOS-P3 | HIGH | 物理模型 | EosRadiativeTransfer.cpp:37 | 消光系数硬编码 5000m 参考路径 |
| EOS-C1 | HIGH | 代码复杂度 | EosPipeline.cpp:187-370 | BuildDetectionRecord 超 220 行，混合多个关注点 |
| EOS-T2 | HIGH | 测试盲区 | foundation/ | 边界值测试全面缺失 |
| EOS-R2 | MEDIUM | 数值稳定性 | EosPropagation.cpp:64-68 | SafePositive 不检查 Inf |
| EOS-R3 | MEDIUM | 数值稳定性 | EosSpatialSpectrum.cpp:38-39 | target_frequency_cpm 无 Inf 保护 |
| EOS-R5 | MEDIUM | 数值稳定性 | EosOpticalCharacteristics.cpp:86-89 | SafePositive 不检查 Inf |
| EOS-A1 | MEDIUM | API 设计 | EosRadiometry.h:27-46 | 输入结构体含过多"预留"字段 |
| EOS-A2 | MEDIUM | API 设计 | EosSession.h:205-212 | 方法缺少 noexcept 与线程安全文档 |
| EOS-P1 | MEDIUM | 物理模型 | EosPipeline.cpp:365 | 红外/可见光 SNR 简单平均，无自适应权重 |
| EOS-P2 | MEDIUM | 物理模型 | EosNoiseModel.cpp:36-37 | RC 滤波假设无文档，模型适用范围不清晰 |
| EOS-E1 | MEDIUM | 错误处理 | EosCoordinateUtils.cpp:50,79 | 几何退化阈值硬编码，调用方无法区分失败原因 |
| EOS-E2 | MEDIUM | 错误处理 | EosSession.cpp:95-149 | ApplyRuntimeConfig 无事务性保证，可能部分更新 |
| EOS-C2 | MEDIUM | 代码复杂度 | EosPipeline.cpp:161-177 | AdvanceScan 使用 while 循环代替 std::fmod |
| EOS-C3 | MEDIUM | 代码复杂度 | EosTraceSession.cpp:14-162 | 18 个重复 ToJson 函数，缺乏统一序列化框架 |
| EOS-M3 | MEDIUM | Magic Number | 全体 | π 值三处重复定义（3.1415926f），物理常数分散 |
| EOS-O2 | MEDIUM | Include 风格 | EosPipeline.cpp:1,13 | Include 路径前缀不一致（有/无 `1q/`） |
| EOS-T1 | MEDIUM | 线程安全 | EosPipeline.h:92 | 无线程安全文档，current_scan_azimuth_deg_ 无保护 |
| EOS-O1 | LOW | API 设计 | EosInputValidation.h:44 | 验证码枚举无末尾哨兵值 |
| EOS-T3 | LOW | 测试盲区 | EosInputValidation.cpp | 缺少跨字段关系验证 |

---

### 详细问题说明

#### EOS-R1 [HIGH] Planck 公式 NaN 检查不合理
**位置**：`src/electro_optical_sensor/foundation/EosRadiometry.cpp:42-55`

```cpp
const float lambda5 = std::pow(safe_wavelength_m, 5.0f);
if (lambda5 <= 0.0f) {         // ⚠️ 无法捕获 NaN
  return 0.0f;
}
const float denominator = lambda5 * std::max(exp_value - 1.0f, 1.0e-12f);
return c1 / denominator;       // denominator 仍可能接近零
```

**问题**：`std::pow` 对极小底数可返回 NaN；`lambda5 <= 0.0f` 无法捕获 NaN；分母可能接近零但未使用 isfinite 检查。

**修复**：
```cpp
const float lambda5 = std::pow(safe_wavelength_m, 5.0f);
if (!std::isfinite(lambda5) || lambda5 <= 0.0f) {
  return 0.0f;
}
const float denominator = lambda5 * std::max(exp_value - 1.0f, 1.0e-12f);
if (!std::isfinite(denominator) || denominator <= 0.0f) {
  return 0.0f;
}
return c1 / denominator;
```

---

#### EOS-R4 [HIGH] 角度分离分母可能为零
**位置**：`src/electro_optical_sensor/foundation/EosStrayLight.cpp:58-60`

```cpp
const float separation_span_deg = outer_half_angle_deg - inner_half_angle_deg;
const float normalized_separation =
    Clamp((result.sun_separation_deg - inner_half_angle_deg) / separation_span_deg, 0.0f, 1.0f);
// ⚠️ 若 separation_span_deg <= 0，产生除零或负数
```

**问题**：尽管第 48-49 行有角度约束（outer >= inner + 1.0f），但此处仍需防御性保护。

**修复**：
```cpp
const float separation_span_deg = std::max(outer_half_angle_deg - inner_half_angle_deg, 0.001f);
```

---

#### EOS-R6 [HIGH] 地面投影 tan() 在接近 ±90° 时溢出
**位置**：`src/electro_optical_sensor/foundation/EosOpticalCharacteristics.cpp:52-56`

```cpp
float ComputeGroundProjectionDistanceM(float platform_altitude_m, float look_angle_deg) {
  const float safe_altitude_m = SafePositive(platform_altitude_m, 1.0f);
  const float look_angle_rad = look_angle_deg * kPi / 180.0f;
  return safe_altitude_m * std::tan(look_angle_rad);  // ⚠️ 接近 ±90° 时 → ±Inf
}
```

**问题**：`look_angle_deg` 未限制范围；负角度产生物理上无意义的负距离；接近 ±90° 时 tan 产生 Inf。

**修复**：
```cpp
const float clamped_angle_deg = std::max(-89.9f, std::min(89.9f, look_angle_deg));
const float look_angle_rad = clamped_angle_deg * kPi / 180.0f;
return std::max(0.0f, safe_altitude_m * std::tan(look_angle_rad));
```

---

#### EOS-P3 [HIGH] 消光系数硬编码参考路径长度
**位置**：`src/electro_optical_sensor/foundation/EosRadiativeTransfer.cpp:37`

```cpp
const float base_extinction_coeff_per_m =
    -std::log(safe_base_transmittance) / 5000.0f;  // ⚠️ 5000m 无出处
```

**问题**：5000.0f 为硬编码参考路径，无注释说明；不同大气条件下此参考值不同；base_transmittance 接近 1 时 `-log` 产生极小值但接近零时产生极大值。

**修复**：
```cpp
constexpr float kReferenceAtmosphericPathM = 5000.0f;  // 标准大气参考路径（km量级以米为单位）
const float safe_log_input = std::max(safe_base_transmittance, 1.0e-9f);
const float base_extinction_coeff_per_m =
    -std::log(safe_log_input) / kReferenceAtmosphericPathM;
```

---

#### EOS-C1 [HIGH] BuildDetectionRecord 函数过度复杂（220+ 行）
**位置**：`src/electro_optical_sensor/core/pipeline/EosPipeline.cpp:187-370`

**问题**：单函数内混合了环境建模、光学特性计算、红外 SNR 评估、可见光 SNR 评估、融合逻辑五个独立关注点，超过合理复杂度阈值（建议 ≤50 行），无法单独测试各模块。

**修复方向**：提取为私有方法：
```cpp
// 建议拆分为：
InfraredMetrics BuildInfraredMetrics(const TargetState&, const PipelineContext&) const;
VisibleMetrics BuildVisibleMetrics(const TargetState&, const PipelineContext&) const;
EosDetectionRecord FuseMetrics(const InfraredMetrics&, const VisibleMetrics&) const;
PipelineContext ApplyEnvironmentFactors(const EosCycleInput&) const;
```

---

#### EOS-R2/R5 [MEDIUM] SafePositive 不处理 Inf 输入
**位置**：`EosPropagation.cpp:64-68`、`EosOpticalCharacteristics.cpp:86-89`

```cpp
// SafePositive 仅检查 <= 0，不检查 NaN/Inf
const float safe_nep_w = SafePositive(nep_w, 1.0e-12f);   // 若 nep_w = Inf，返回 Inf
return safe_received_power_w / safe_nep_w;  // ⚠️ 0 / Inf = 0，但 Inf / Inf = NaN
```

**修复**：统一加强 `SafePositive`：
```cpp
inline float SafePositive(float value, float fallback) {
  return (std::isfinite(value) && value > 0.0f) ? value : fallback;
}
```

---

#### EOS-E2 [MEDIUM] ApplyRuntimeConfig 无事务性保证
**位置**：`src/electro_optical_sensor/core/session/EosSession.cpp:95-149`

**问题**：多个参数的验证与写入交错进行。若参数 A 验证失败（记录错误），参数 B 仍可能写入，导致配置处于部分更新状态。

**修复方向**：先验证全部参数，通过后原子性写入：
```cpp
// Phase 1: 验证
bool all_valid = ValidateAllPatchFields(patch);
if (!all_valid) { /* log and return */ }
// Phase 2: 写入
ApplyValidatedPatch(patch, runtime_config);
```

---

#### EOS-C2 [MEDIUM] AdvanceScan 用 while 循环实现模运算
**位置**：`src/electro_optical_sensor/core/pipeline/EosPipeline.cpp:161-177`

```cpp
while (wrapped_offset_deg >= scan_width_deg) { wrapped_offset_deg -= scan_width_deg; }
while (wrapped_offset_deg < 0.0f)            { wrapped_offset_deg += scan_width_deg; }
```

**修复**：
```cpp
if (scan_width_deg < 0.001f) return;  // 退化守卫
float wrapped_offset_deg = std::fmod(total_offset_deg, scan_width_deg);
if (wrapped_offset_deg < 0.0f) wrapped_offset_deg += scan_width_deg;
```

---

#### EOS-M3 [MEDIUM] π 值三处重复定义
**位置**：`EosOpticalCharacteristics.cpp:29`、`EosStrayLight.cpp:29`、`EosPipeline.cpp:73`

```cpp
constexpr float kPi = 3.1415926f;  // 每个文件独立定义
```

**问题**：单精度精度不足（`std::numbers::pi_v<float>` = 3.14159274f）；三份定义易发散。

**修复**：创建 `EosPhysicalConstants.h` 统一定义所有物理/数学常量。

---

#### EOS-P1 [MEDIUM] 红外/可见光 SNR 等权重融合
**位置**：`src/electro_optical_sensor/core/pipeline/EosPipeline.cpp:365`

```cpp
record.fused_snr_linear = 0.5f * (infrared_snr_linear + visible_snr_linear);
// ⚠️ 夜间可见光 SNR≈0，融合结果被拉低 50%
```

**修复建议**：根据 `day_night_type` 动态调整权重，或基于各通道有效性（SNR > 阈值）加权。

---

## 三、ESR 模块问题（electronic_surveillance_radar）

### 问题优先级汇总

| ID | 严重性 | 类别 | 文件 | 描述 |
|----|--------|------|------|------|
| ESR-R2 | HIGH | 数值稳定性 | JammingAggregator.h:89-91 | 欺骗概率累积乘法仅检查 < 0，不检查 NaN/Inf |
| ESR-R4 | HIGH | API 设计 | EsrSession.cpp:76-86 | 接收频率窗口边界检查逻辑不对称，状态可能不一致 |
| ESR-R8 | HIGH | 数值稳定性 | EsrEnvironmentService.cpp:97-100 | 累积乘法遇 NaN source_risk 后无法恢复 |
| ESR-R12 | HIGH | 内存安全 | KdTreeClusterer.cpp:48,59-60 | nanoflann 适配器无维度越界检查 |
| ESR-R1 | MEDIUM | 数值稳定性 | HypothesisAssociator.cpp:168,203 | sqrt(support_count) 当 count=0 时产生 Inf |
| ESR-R3 | MEDIUM | 数值稳定性 | ObservationFeatureEncoder.h:32-41 | 部分 scale 字段无正值检查 |
| ESR-R5 | MEDIUM | 算法正确性 | KdTreeClusterer.cpp:84-86 | min_points 无上界检查，可能导致全点被标为噪声 |
| ESR-R6 | MEDIUM | 错误处理 | EsrInputValidation.cpp:120-124 | pri/pulse_width 关系校验逻辑有漏洞 |
| ESR-R9 | MEDIUM | 数值稳定性 | AngleErrorModel.h:42 | pow(10, snr_db/10) snr_db 未限制范围 |
| ESR-R10 | MEDIUM | 数值稳定性 | InterceptGate.h:110,117-118 | 极小值保护 epsilon 不一致 |
| ESR-R13 | MEDIUM | Magic Number | EsrSessionConfigResolver.cpp:52-63 | 工作模式缩放因子硬编码（0.85f, 1.25f, 4U, 4096U） |
| ESR-R14 | MEDIUM | 类型安全 | HypothesisAssociator.cpp:151-177 | float/double 隐式转换，精度可能丢失 |
| ESR-R7 | LOW | 代码复杂度 | HypothesisAssociator.cpp:135-143 | 排序比较器辅助排序逻辑冗余，未文档化 |
| ESR-R11 | LOW | 数值稳定性 | ObservationPreprocessor.cpp:56-63 | 去重判断浮点比较无 epsilon |
| ESR-R15 | LOW | 算法正确性 | BandClassifier.h:55-82 | 频段边界硬编码，无配置化机制 |

---

### 详细问题说明

#### ESR-R2 [HIGH] 欺骗概率累积乘法 NaN 泄漏
**位置**：`src/electronic_surveillance_radar/intercept/JammingAggregator.h:89-91`

```cpp
deception_clear_probability *= (1.0f - deception_effect);
if (deception_clear_probability < 0.0f) {   // ⚠️ NaN 不满足此条件，不会被修正
  deception_clear_probability = 0.0f;
}
```

**问题**：若 `deception_effect` 为 NaN，`1.0f - NaN = NaN`，乘法结果为 NaN；NaN 不满足 `< 0.0f`，因此不会被修正；后续所有基于该值的计算全部污染。

**修复**：
```cpp
const float safe_effect = std::isfinite(deception_effect) ? Clamp01(deception_effect) : 0.0f;
deception_clear_probability *= (1.0f - safe_effect);
deception_clear_probability = Clamp01(deception_clear_probability);
```

---

#### ESR-R4 [HIGH] 接收频率窗口边界检查逻辑不对称
**位置**：`src/electronic_surveillance_radar/core/session/EsrSession.cpp:76-86`

```cpp
if (patch.has_fixed_receiver_window_hz &&
    IsFinite(patch.receiver_lower_hz) && IsFinite(patch.receiver_upper_hz) &&
    patch.receiver_upper_hz > patch.receiver_lower_hz) {
  // 正常更新路径
} else if (patch.has_fixed_receiver_window_hz) {
  PROJECT_LOG_ERROR("...");
  // ⚠️ 不触发 pipeline 通知，状态不一致
} else if (fixed_receiver_window_bounds_valid) {  // ⚠️ 此标志仅在上方初始化，语义不清
  // 可能误触发
}
```

**问题**：参数验证失败时仅记录日志，不通知 pipeline 配置未生效；第三个 else-if 依赖外部标志，语义不透明；多路径配置更新缺乏原子性。

**修复方向**：将验证与更新分离，通过返回值明确告知调用方更新是否生效：
```cpp
bool window_update_applied = false;
if (patch.has_fixed_receiver_window_hz) {
  if (IsValidReceiverWindow(patch)) {
    ApplyReceiverWindow(patch, &runtime_config);
    window_update_applied = true;
  } else {
    PROJECT_LOG_ERROR("...");
    // 返回错误或更新结果枚举，而非静默失败
  }
}
```

---

#### ESR-R8 [HIGH] EsrEnvironmentService 累积乘法遇 NaN 无法恢复
**位置**：`src/electronic_surveillance_radar/environment/EsrEnvironmentService.cpp:97-100`

```cpp
deception_clear_probability *= (1.0f - source_risk);
if (deception_clear_probability < 0.0f) {   // ⚠️ 同 ESR-R2，NaN 穿透此检查
  deception_clear_probability = 0.0f;
}
```

**问题**：与 ESR-R2 相同的模式在不同文件中再次出现，说明存在系统性遗漏。

**修复**：同 ESR-R2，在乘法前 Clamp01 source_risk，乘法后 Clamp01 结果：
```cpp
const float safe_risk = Clamp01(std::isfinite(source_risk) ? source_risk : 0.0f);
deception_clear_probability = Clamp01(deception_clear_probability * (1.0f - safe_risk));
```

---

#### ESR-R12 [HIGH] nanoflann 适配器无维度越界保护
**位置**：`src/electronic_surveillance_radar/pipeline/KdTreeClusterer.cpp:48,59-60`

```cpp
inline float kdtree_get_pt(const std::size_t idx, const std::size_t dim) const {
  return features_[idx].values[dim];  // ⚠️ 无 dim < kObservationFeatureDimension 检查
}
```

**问题**：nanoflann 内部可能以超出 `kObservationFeatureDimension` 的维度查询；`idx` 越界时同样无保护；两者均会导致未定义行为（内存越界访问）。

**修复**：
```cpp
inline float kdtree_get_pt(const std::size_t idx, const std::size_t dim) const {
  if (idx >= features_.size() || dim >= kObservationFeatureDimension) {
    PROJECT_LOG_ERROR("[KdTreeClusterer] kdtree_get_pt out of bounds: idx={}, dim={}", idx, dim);
    return 0.0f;
  }
  return features_[idx].values[dim];
}
```

---

#### ESR-R1 [MEDIUM] sqrt(support_count) 在 count=0/1 时产生极值
**位置**：`src/electronic_surveillance_radar/pipeline/HypothesisAssociator.cpp:168,203`

```cpp
const float base_bearing_std_deg =
    std::max(0.1f, 3.0f / std::sqrt(static_cast<float>(summary.support_count)));
// ⚠️ count=0 → sqrt(0)=0 → 除零 → Inf；count=1 → std = 3.0，无统计意义
```

**修复**：
```cpp
const std::size_t safe_count = std::max(std::size_t{2}, summary.support_count);
const float base_bearing_std_deg =
    std::max(0.1f, 3.0f / std::sqrt(static_cast<float>(safe_count)));
```

---

#### ESR-R5 [MEDIUM] KdTree 聚类 min_points 无上界检查
**位置**：`src/electronic_surveillance_radar/pipeline/KdTreeClusterer.cpp:84-86`

```cpp
const std::size_t min_points = config.min_points > 0U
    ? static_cast<std::size_t>(config.min_points) : 1U;
// ⚠️ 无上界检查：若 min_points >> features.size()，全部被标为噪声
```

**修复**：
```cpp
const std::size_t min_points = std::max(std::size_t{1},
    std::min(static_cast<std::size_t>(config.min_points), features.size()));
```

---

#### ESR-R6 [MEDIUM] PRI/脉冲宽度关系校验逻辑漏洞
**位置**：`src/electronic_surveillance_radar/core/context/EsrInputValidation.cpp:120-124`

```cpp
if (emitter.pri_s > 0.0 && emitter.pulse_width_s > 0.0 &&
    emitter.pri_s < emitter.pulse_width_s) {
    // 记录 PRI < 脉宽的错误
}
// ⚠️ 若 pulse_width_s <= 0 而 pri_s > 0，第一个条件不满足，PRI < 脉宽的检查被跳过
```

**修复**：拆分为独立检查：
```cpp
if (emitter.pri_s > 0.0 && emitter.pulse_width_s > 0.0) {
  if (emitter.pri_s < emitter.pulse_width_s) {
    // 记录错误：PRI < pulse_width
  }
}
```

---

#### ESR-R9 [MEDIUM] AngleErrorModel pow(10, snr_db/10) 范围无保护
**位置**：`src/electronic_surveillance_radar/intercept/AngleErrorModel.h:42`

```cpp
const double snr_linear = std::pow(10.0, static_cast<double>(snr_db) / 10.0);
// ⚠️ snr_db = -∞ → snr_linear = 0；snr_db = +∞ → snr_linear = +∞
// ⚠️ snr_db = NaN → snr_linear = NaN
```

**修复**：
```cpp
if (!std::isfinite(snr_db)) return config.max_std_deg;
const double clamped_snr_db = std::clamp(static_cast<double>(snr_db), -100.0, 100.0);
const double snr_linear = std::pow(10.0, clamped_snr_db / 10.0);
```

---

#### ESR-R13 [MEDIUM] 工作模式缩放因子全部硬编码
**位置**：`src/electronic_surveillance_radar/core/session/EsrSessionConfigResolver.cpp:52-63`

```cpp
config->pulse_count = std::min<std::uint32_t>(
    config->pulse_count * 4U, static_cast<std::uint32_t>(4096U));  // ⚠️ 4U, 4096U 无出处
config->threshold_scale = std::max(0.1f, config->threshold_scale * 0.85f);  // ⚠️ 0.85f
config->threshold_scale *= 1.25f;  // ⚠️ 1.25f
```

**问题**：5 个硬编码缩放常数分散在 3 行，无来源注释；后续调参需修改源码。

**修复**：提取为命名常量：
```cpp
constexpr std::uint32_t kActiveScanPulseMultiplier = 4U;
constexpr std::uint32_t kMaxPulseCount = 4096U;
constexpr float kAggressiveThresholdScale = 0.85f;
constexpr float kConservativeThresholdScale = 1.25f;
```

---

#### ESR-R14 [MEDIUM] HypothesisAssociator float/double 隐式混用
**位置**：`src/electronic_surveillance_radar/pipeline/HypothesisAssociator.cpp:151-177`

```cpp
// summary.confidence_score 为 float；Clamp01 返回 float；但中间 1.0f - 0.45f 为 float 运算
const float confidence_measurement =
    Clamp01(summary.confidence_score * (1.0f - 0.45f * deception_ratio));
```

**问题**：`deception_ratio` 若为 `double` 则此处产生隐式转换；当精度丢失在概率计算中发生时，可能累积为可观误差。

**修复**：统一使用 float，或通过 explicit cast 标注转换意图。

---

#### ESR-R3 [MEDIUM] ObservationFeatureEncoder 部分 scale 无正值检查
**位置**：`src/electronic_surveillance_radar/pipeline/ObservationFeatureEncoder.h:32-41`

```cpp
const double rf_scale = scales.rf_scale_hz > 0.0f
    ? static_cast<double>(scales.rf_scale_hz) : 1.0;
// ⚠️ amplitude_scale、time_scale 无类似检查
// 若 amplitude_scale = -1，特征向量符号反转，破坏聚类语义
```

**修复**：对所有 scale 字段统一防护：
```cpp
const double amplitude_scale = std::fabs(scales.amplitude_scale) > 1.0e-9
    ? std::fabs(static_cast<double>(scales.amplitude_scale)) : 1.0;
```

---

## 四、跨模块共性问题

### 4.1 SafePositive 未处理 Inf
两个模块均依赖 `SafePositive(value, fallback)` 函数，但该函数仅检查 `value <= 0.0f`，不检查 NaN 和 Inf。这是系统性问题，应在共享工具函数层统一修复。

**影响**：EOS-R2, EOS-R5 均由此引起。

### 4.2 累积乘法 NaN 穿透模式
ESR 模块的 `JammingAggregator.h` 和 `EsrEnvironmentService.cpp` 均出现了相同的 "累积乘法 → 仅检查 < 0 → NaN 穿透" 模式（ESR-R2, ESR-R8）。建议提取为公共工具函数：
```cpp
inline float AccumulateProbability(float current, float event_probability) {
  const float safe_event = Clamp01(std::isfinite(event_probability) ? event_probability : 0.0f);
  return Clamp01(current * (1.0f - safe_event));
}
```

### 4.3 线程安全文档缺失
EOS 的 `EosPipeline`（含可变状态 `current_scan_azimuth_deg_`）和 ESR 的 `EsrSession` 均无线程安全文档。建议统一在所有有状态类的 class doc 中明确线程模型（单线程 / 外部同步 / 线程局部实例）。

---

## 五、测试覆盖盲区

### EOS 模块

| 测试盲区 | 风险 |
|---------|------|
| `EosRadiometry` 边界值（wavelength 极小/极大） | HIGH — Planck 公式在极端值下行为未验证 |
| `EosPropagation::ComputeReceivedPowerW` range_m=0 | HIGH — 可能除零 |
| `EosSpatialSpectrum` gsd_m=0 | HIGH — 空间频率计算崩溃 |
| `EosInputValidation` 跨字段关系（scan_start > scan_end） | MEDIUM |
| `BuildDetectionRecord` 子阶段独立测试 | MEDIUM — 220 行函数无法单独覆盖 |

### ESR 模块

| 测试盲区 | 风险 |
|---------|------|
| `HypothesisAssociator` support_count=0/1 | HIGH — sqrt 除零 |
| `KdTreeClusterer` 特征维度边界 | HIGH — 内存越界 |
| `JammingAggregator` NaN/Inf 输入 | HIGH — NaN 传播 |
| `EsrSession::ApplyRuntimeConfig` 多路径场景 | MEDIUM |
| `ObservationPreprocessor` 去重时间窗口边界 | LOW |

---

## 六、优先修复路线图

### 立即修复（HIGH 级别，数值安全/内存安全）

1. **EOS-R1**：Planck 公式 lambda5 NaN 检查 → 改用 `!std::isfinite(lambda5)`
2. **EOS-R4**：StrayLight 除零保护 → `std::max(separation_span_deg, 0.001f)`
3. **EOS-R6**：地面投影 tan 角度限制 → clamp 到 ±89.9°
4. **ESR-R2**：JammingAggregator 累积乘法 NaN 穿透 → Clamp01 + isfinite 包装
5. **ESR-R8**：EsrEnvironmentService 相同模式修复
6. **ESR-R12**：KdTreeClusterer 维度越界 → 添加越界检查
7. **ESR-R4**：接收窗口逻辑不对称 → 分离验证与更新

### 短期（MEDIUM 级别）

8. **SafePositive 全局加固**（同时修复 EOS-R2, EOS-R5）
9. **ESR-R1**：HypothesisAssociator sqrt 保护
10. **ESR-R5**：KdTreeClusterer min_points 上界
11. **EOS-C1**：BuildDetectionRecord 拆分为子函数
12. **EOS-P3**：消光系数参考路径提取为具名常量
13. **EOS-E2**：ApplyRuntimeConfig 事务性保证
14. **ESR-R9**：AngleErrorModel pow 输入限制
15. **ESR-R13**：工作模式缩放因子提取为常量

### 后续改进（LOW 级别 + 架构改善）

16. **EOS-M3**：建立 `EosPhysicalConstants.h` 统一常量定义
17. **EOS-C3**：引入统一 JSON 序列化框架（替代 18 个重复 ToJson）
18. **EOS-C2**：AdvanceScan 改用 `std::fmod`
19. **EOS-O2**：统一 include 路径前缀
20. 为两个模块补充边界值单元测试（NaN/Inf/零/极值）

---

## 七、关键文件索引

### EOS 核心高关注文件

| 文件 | 关注原因 |
|------|---------|
| `src/.../foundation/EosRadiometry.cpp` | EOS-R1：Planck 公式数值稳定性 |
| `src/.../foundation/EosStrayLight.cpp` | EOS-R4：除零漏洞 |
| `src/.../foundation/EosOpticalCharacteristics.cpp` | EOS-R6：tan 溢出 |
| `src/.../foundation/EosRadiativeTransfer.cpp` | EOS-P3：硬编码参考路径 |
| `src/.../core/pipeline/EosPipeline.cpp` | EOS-C1：220 行复杂函数 |

### ESR 核心高关注文件

| 文件 | 关注原因 |
|------|---------|
| `src/.../intercept/JammingAggregator.h` | ESR-R2：NaN 穿透 |
| `src/.../environment/EsrEnvironmentService.cpp` | ESR-R8：NaN 穿透（同模式） |
| `src/.../pipeline/KdTreeClusterer.cpp` | ESR-R12：内存越界风险 |
| `src/.../core/session/EsrSession.cpp` | ESR-R4：状态不一致 |
| `src/.../pipeline/HypothesisAssociator.cpp` | ESR-R1：sqrt 除零 |

---

*报告结束*

## 八、收敛核查表（2026-04-03）

> 说明：以下为当前代码库实装核查结果，状态分为 `已收敛` / `未收敛`。本轮核查后均为 `已收敛`。

### EOS 问题收敛

| ID | 状态 | 核查结论（代码/测试证据） |
|----|------|----------------------------|
| EOS-R1 | 已收敛 | `EosRadiometry.cpp` 已补 `isfinite` 与分母防护；`eos_foundation_unit_test.cpp` 覆盖辐射计算行为 |
| EOS-R4 | 已收敛 | `EosStrayLight.cpp` 已做分母保护与抑制比约束；`eos_straylight_unit_test.cpp` 覆盖 |
| EOS-R6 | 已收敛 | `EosOpticalCharacteristics.cpp` 已做视角 clamp 与非负距离约束；`eos_foundation_unit_test.cpp` 覆盖 |
| EOS-P3 | 已收敛 | `EosRadiativeTransfer.cpp` 参考路径常量化并加安全下限；`eos_radiative_transfer_unit_test.cpp` 覆盖 |
| EOS-C1 | 已收敛 | `EosPipeline.cpp` 检测记录构建已拆分为多子函数并降低复杂度；`eos_pipeline_unit_test.cpp` 回归 |
| EOS-T2 | 已收敛 | foundation 与 pipeline 相关边界测试已补齐（NaN/Inf/阈值/极值） |
| EOS-R2 | 已收敛 | `EosPropagation.cpp` SafePositive/输入防护已统一有限值检查 |
| EOS-R3 | 已收敛 | `EosSpatialSpectrum.cpp` 对频率项已加非有限防护；`eos_spatial_spectrum_unit_test.cpp` 覆盖 |
| EOS-R5 | 已收敛 | `EosOpticalCharacteristics.cpp` SafePositive 分支已统一 `isfinite` 处理 |
| EOS-A1 | 已收敛 | `EosRadiometry.h` 已移除 `VisibleRadianceInputs` 预留字段并同步调用/测试 |
| EOS-A2 | 已收敛 | `EosSession.h` builder 方法补 `noexcept`，并补线程模型注释；析构 `noexcept` |
| EOS-P1 | 已收敛 | `EosPipeline.cpp` 融合 SNR 已改按昼夜/通道有效性动态加权；`eos_pipeline_unit_test.cpp` 覆盖 |
| EOS-P2 | 已收敛 | `EosNoiseModel.cpp` 增补 RC 等效带宽假设说明并常量化参数 |
| EOS-E1 | 已收敛 | `EosCoordinateUtils` 增加 `EosCoordinateStatus` 失败原因分级返回与单测 |
| EOS-E2 | 已收敛 | `EosSession.cpp` `ApplyRuntimeConfig` 已采用“先验证后提交”的事务式更新 |
| EOS-C2 | 已收敛 | `EosPipeline.cpp` 扫描相位推进已用 `std::fmod` 等价实现并含退化守卫 |
| EOS-C3 | 已收敛 | `EosTraceSession.cpp` 已统一到 `nlohmann::ordered_json` 序列化框架 |
| EOS-M3 | 已收敛 | 新增 `EosPhysicalConstants.h` 并替换多处重复 π 常量 |
| EOS-O2 | 已收敛 | `EosPipeline.cpp` include 前缀已统一；foundation 转发头保证分层不破坏 |
| EOS-T1 | 已收敛 | `EosPipeline.h` / `EosSession.h` 已补充方法级非线程安全说明 |
| EOS-O1 | 已收敛 | `EosInputValidation.h` 验证码枚举已包含 `kCount` 末尾哨兵 |
| EOS-T3 | 已收敛 | `EosInputValidation.cpp` 已增跨字段校验（能量守恒/昼夜一致性）与单测 |

### ESR 问题收敛

| ID | 状态 | 核查结论（代码/测试证据） |
|----|------|----------------------------|
| ESR-R2 | 已收敛 | `JammingAggregator.h` 欺骗概率累积已加 `isfinite + Clamp01` |
| ESR-R4 | 已收敛 | `EsrSession.cpp` 接收窗口更新路径已拆分验证与提交，避免状态不一致 |
| ESR-R8 | 已收敛 | `EsrEnvironmentService.cpp` 累积概率路径已做 NaN/Inf 防穿透处理 |
| ESR-R12 | 已收敛 | `KdTreeClusterer.cpp` nanoflann 适配器已补维度越界保护 |
| ESR-R1 | 已收敛 | `HypothesisAssociator.cpp` 支持数下限保护，避免 `sqrt(0)` 退化风险 |
| ESR-R3 | 已收敛 | `ObservationFeatureEncoder.h` scale 项已统一正值与有限值校验 |
| ESR-R5 | 已收敛 | `KdTreeClusterer.cpp` `min_points` 已补上界防护与行为约束 |
| ESR-R6 | 已收敛 | `EsrInputValidation.cpp` PRI/脉宽关系校验逻辑已修正 |
| ESR-R9 | 已收敛 | `AngleErrorModel.h` 对 SNR 输入区间做 clamp 防止幂运算溢出 |
| ESR-R10 | 已收敛 | `InterceptGate.h` epsilon 常量已统一并补边界测试 |
| ESR-R13 | 已收敛 | `EsrSessionConfigResolver.cpp` 工作模式缩放常量已具名化 |
| ESR-R14 | 已收敛 | `HypothesisAssociator.cpp` 关联门限与距离比较已统一类型并减少隐式转换 |
| ESR-R7 | 已收敛 | `HypothesisAssociator.cpp` 候选排序比较器已命名化并明确 tie-break 规则 |
| ESR-R11 | 已收敛 | `ObservationPreprocessor.cpp` 去重比较已引入统一 epsilon 体系 |
| ESR-R15 | 已收敛 | `BandClassifier.h` 已支持外部频段表配置并保留默认表 |

### 核查执行记录

- 构建：`cmake --build --preset llvm-ninja-debug-local`
- 测试：`ctest --preset llvm-ninja-debug-local -Q --output-on-failure -R "(EosFoundationTest|EosInputValidationTest|EosPipelineTest|EosCoordinateUtilsTest|EsrHypothesisAssociatorTest|EsrAlgorithmsTest|EsrKdTreeClustererTest|TraceSessionAdapterTest)"`
- 结果：通过

