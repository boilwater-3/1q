/**
 * @file ArProfileConstants.h
 * @brief 机载雷达语义档位常量表。
 *
 * 将原 ArSessionConfigBuilder 的 Profile 枚举翻译结果提取为预定义结构体常量，
 * 用户直接赋值到目标字段即可（"Profile 覆盖"语义不再存在——赋值即最终决定）。
 *
 * 命名约定：k<档位名><子域类型>。仅收录对 struct 默认值产生有效覆盖的档位；
 * 与默认值相同的 no-op 档位（kGenericAirborneXBand / kStandard / kBalanced /
 * kDisabled）不提供常量，struct 默认值即为该档位。
 *
 * 跨域档位（kRobustAntiJamming）拆为多个常量，分别赋值到对应子域。
 */

#ifndef ONEQ_AIRBORNE_RADAR_CONFIG_AR_PROFILE_CONSTANTS_H_
#define ONEQ_AIRBORNE_RADAR_CONFIG_AR_PROFILE_CONSTANTS_H_

#include <cmath>

#include "1q/airborne_radar/config/ArHardwareConfig.h"
#include "1q/airborne_radar/config/ArPolicyConfig.h"

namespace airborne_radar {
namespace config {
namespace profiles {

// =============================================================================
// 硬件档位（赋给 config.hardware，类型 detection::DetectionConfig）
// =============================================================================

/** @brief 远程高功率档位：5 MW、9.3 GHz，适用于远距探测场景。 */
const detection::DetectionConfig kLongRangeHighPowerHardware = [] {
  detection::DetectionConfig c{};
  c.transmitter.peak_power_w = 5.0e6f;
  c.transmitter.maximum_peak_power_w = 6.0e6f;
  c.transmitter.frequency_hz = 9.3e9f;
  // plan 首项必须与 frequency_hz 的浮点表示严格相等（校验要求 plan 包含初始载频），
  // 故从 frequency_hz 转换而非直接写 double 字面量（9.3e9f != 9.3e9，差 256 Hz）。
  c.transmitter.frequency_plan_hz = {static_cast<double>(c.transmitter.frequency_hz)};
  c.transmitter.bandwidth_hz = 3.0e6f;
  c.transmitter.pulse_width_s = 18e-6f;
  c.transmitter.prf_hz = 220.0f;
  c.antenna.main_beam_gain_db = 38.0f;
  c.receiver.noise_figure_db = 3.0f;
  return c;
}();

/** @brief 轻型低截获概率档位：350 kW、10 GHz，适用于隐蔽探测场景。 */
const detection::DetectionConfig kLightweightLpiHardware = [] {
  detection::DetectionConfig c{};
  c.transmitter.peak_power_w = 3.5e5f;
  c.transmitter.frequency_hz = 10.0e9f;
  c.transmitter.frequency_plan_hz = {10.0e9};
  c.transmitter.bandwidth_hz = 8.0e6f;
  c.transmitter.pulse_width_s = 8e-6f;
  c.transmitter.prf_hz = 600.0f;
  c.antenna.main_beam_gain_db = 31.0f;
  c.antenna.nominal_az_beamwidth_deg = 5.0f;
  c.antenna.nominal_el_beamwidth_deg = 5.0f;
  c.receiver.noise_figure_db = 5.0f;
  return c;
}();

// =============================================================================
// 探测意图档位（赋给 config.policy.detection，类型 detection::ArDetectionPolicyConfig）
// =============================================================================

/** @brief 探测优先：放宽门限并增加积累，优先发现目标。 */
const detection::ArDetectionPolicyConfig kDetectionPriorityDetection = [] {
  detection::ArDetectionPolicyConfig c{};
  c.pulse_count = 16;
  c.pfa = 2e-6f;
  c.minimum_snr_db = -12.0f;
  c.minimum_detection_margin_db = -100.0f;
  return c;
}();

/** @brief 航迹稳定优先：收紧门限并减少积累，优先抑制虚警。 */
const detection::ArDetectionPolicyConfig kTrackStabilityPriorityDetection = [] {
  detection::ArDetectionPolicyConfig c{};
  c.pulse_count = 8;
  c.pfa = 5e-7f;
  c.minimum_snr_db = -8.0f;
  c.minimum_detection_margin_db = -20.0f;
  return c;
}();

// =============================================================================
// 天线方向图档位（赋给 config.hardware.antenna.pattern，类型 detection::AntennaPatternConfig）
// =============================================================================

/** @brief 低旁瓣方向图：压低旁瓣与后瓣电平。 */
const detection::AntennaPatternConfig kLowSidelobeAntennaPattern = [] {
  detection::AntennaPatternConfig c{};
  c.max_sidelobe_level_db = -30.0f;
  c.backlobe_level_db = -42.0f;
  return c;
}();

/** @brief 宽覆盖方向图：抛物线主瓣、放宽旁瓣、容忍扫描损失。 */
const detection::AntennaPatternConfig kWideCoverageAntennaPattern = [] {
  detection::AntennaPatternConfig c{};
  c.model_type = detection::AntennaPatternModelType::kParabolicMainLobe;
  c.max_sidelobe_level_db = -18.0f;
  c.max_scan_loss_db = 8.0f;
  return c;
}();

// =============================================================================
// RCS 融合档位（赋给 config.hardware.rcs_physics，类型 detection::RcsPhysicsConfig）
// =============================================================================

/** @brief 保守融合：物理 RCS 占比 0.25。 */
const detection::RcsPhysicsConfig kConservativeRcsPhysics = [] {
  detection::RcsPhysicsConfig c{};
  c.enable_physical_rcs = true;
  c.physics_mix_ratio = 0.25f;
  return c;
}();

/** @brief 增强融合：物理 RCS 占比 0.60，圆柱模型权重 0.65。 */
const detection::RcsPhysicsConfig kEnhancedRcsPhysics = [] {
  detection::RcsPhysicsConfig c{};
  c.enable_physical_rcs = true;
  c.physics_mix_ratio = 0.60f;
  c.cylinder_weight = 0.65f;
  return c;
}();

// =============================================================================
// 跟踪策略档位（赋给 config.policy.tracking / config.policy.association）
// =============================================================================

/** @brief 快速关联：低量测噪声假设、丢失周期快速衰减。 */
const tracking::TrackingConfig kFastAssociationTracking = [] {
  tracking::TrackingConfig c{};
  c.kalman_measurement_noise_std = 6.0f;
  c.speed_decay_ratio_on_loss = 0.95f;
  c.rcs_decay_ratio_on_loss = 0.92f;
  return c;
}();

/** @brief 鲁棒抗干扰跟踪：宽量测噪声假设、丢失周期快速衰减。 */
const tracking::TrackingConfig kRobustAntiJammingTracking = [] {
  tracking::TrackingConfig c{};
  c.kalman_measurement_noise_std = 12.0f;
  c.speed_decay_ratio_on_loss = 0.95f;
  c.rcs_decay_ratio_on_loss = 0.92f;
  return c;
}();

/** @brief 鲁棒抗干扰关联：放宽归一化距离关联门限。 */
const tracking::AssociationConfig kRobustAntiJammingAssociation = [] {
  tracking::AssociationConfig c{};
  c.distance_gate_sigma = std::sqrt(12.0f);
  return c;
}();

// =============================================================================
// 生命周期档位（赋给 config.policy.lifecycle，类型 lifecycle::LifecycleConfig）
// =============================================================================

/** @brief 快速确认：1 次命中确认、1 次丢失标记、3 周期删除。 */
const lifecycle::LifecycleConfig kFastConfirmLifecycle = [] {
  lifecycle::LifecycleConfig c{};
  c.confirm_hits = 1U;
  c.max_miss_before_lost = 1U;
  c.max_lost_cycles = 3U;
  return c;
}();

/** @brief 高持久：3 次命中确认、容忍 3 次丢失、保留 8 周期。 */
const lifecycle::LifecycleConfig kHighPersistenceLifecycle = [] {
  lifecycle::LifecycleConfig c{};
  c.confirm_hits = 3U;
  c.max_miss_before_lost = 3U;
  c.max_lost_cycles = 8U;
  return c;
}();

}  // namespace profiles
}  // namespace config
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_CONFIG_AR_PROFILE_CONSTANTS_H_
