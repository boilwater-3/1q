/**
 * @file InternalSignalPipelineConfig.h
 * @brief 定义 SignalPipeline 内部扩展配置。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_INTERNAL_SIGNAL_PIPELINE_CONFIG_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_INTERNAL_SIGNAL_PIPELINE_CONFIG_H_

#include <cstddef>
#include <vector>

#include "airborne_radar/config/engineering/SignalEngineeringConfig.h"
#include "airborne_radar/signal/pipeline/config/SignalPipelineRuntimeTypes.h"
#include "airborne_radar/signal/tracking/LifecycleConfig.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

enum class InternalSignalPipelineProfile {
  kBaseline = 0,
  kTrackingPreset,
  kRobustPreset,
};

inline InternalSignalPipelineProfile ResolveInternalProfileFromPublicConfig(
    const SignalPipelineConfig& public_config) {
  if (public_config.lifecycle.policy_profile == config::LifecyclePolicyProfile::kHighPersistence ||
      public_config.tracking.policy_profile == config::TrackingPolicyProfile::kRobustAntiJamming) {
    return InternalSignalPipelineProfile::kRobustPreset;
  }
  if (public_config.tracking.policy_profile == config::TrackingPolicyProfile::kFastAssociation ||
      public_config.detection.intent_profile == config::DetectionIntentProfile::kDetectionPriority) {
    return InternalSignalPipelineProfile::kTrackingPreset;
  }
  return InternalSignalPipelineProfile::kBaseline;
}

struct JammingEffectsConfig {
  float confidence_weight_min{0.25f};
  float heuristic_base_penalty_db{0.8f};
  float heuristic_power_penalty_slope{0.18f};
  float heuristic_noise_sidelobe_penalty{0.9f};
  float heuristic_noise_frontlobe_ratio{0.4f};
  float heuristic_noise_js_slope{0.4f};
  float heuristic_deception_freq_penalty{1.8f};
  float heuristic_deception_prf_penalty{1.2f};
  float heuristic_repeater_prf_penalty{1.0f};
  float heuristic_repeater_freq_penalty{0.8f};
  float heuristic_unknown_freq_penalty{1.1f};
  float heuristic_unknown_prf_penalty{0.9f};
  float heuristic_unknown_sidelobe_penalty{0.6f};
  float heuristic_unknown_frontlobe_penalty{0.2f};
  float association_scale_max{2.5f};
  float tracking_noise_scale_max{2.0f};
  float measurement_noise_scale_max{1.8f};
  float deception_association_step{0.18f};
  float deception_tracking_step{0.12f};
  float deception_measurement_step{0.10f};
  float repeater_association_step{0.12f};
  float repeater_tracking_step{0.15f};
  float repeater_measurement_step{0.08f};
  float noise_measurement_step{0.06f};
  float unknown_association_step{0.08f};
  float unknown_tracking_step{0.08f};
  float unknown_measurement_step{0.05f};
  float covariance_inflation_max{2.5f};
  float covariance_deception_inflation_step{0.20f};
  float covariance_repeater_inflation_step{0.16f};
  float covariance_noise_inflation_step{0.08f};
  float covariance_unknown_inflation_step{0.10f};
};

struct ControlProfileEffectsConfig {
  float sidelobe_level_reduction_db{6.0f};
  float adaptive_beam_gain_boost_db{2.0f};
  float adaptive_beamwidth_scale{0.60f};
  float lpi_beamwidth_scale{0.75f};
  float lpi_beam_signal_gain_db{1.0f};
  float adaptive_beam_signal_gain_db{1.5f};
  float eccm_speed_decay_bonus{0.05f};
  float eccm_rcs_decay_bonus{0.08f};
};

struct SignalLifecycleInternalConfig {
  tracking::ImmActivationPolicy imm_activation_policy{
      tracking::ImmActivationPolicy::kConfirmedTracksOnly};
  tracking::TrackPoolThreadSafetyMode track_pool_thread_safety_mode{
      tracking::TrackPoolThreadSafetyMode::kSingleThreadNoLock};
  std::vector<float> imm_model_noise_diff_coeffs;
  std::vector<float> imm_initial_weights;
  std::vector<float> imm_transition_probability;
  std::size_t lifecycle_track_pool_initial_chunk{64};
  std::size_t lifecycle_track_pool_max_chunks{256};
};

struct SignalAssociationInternalConfig {
  float unassigned_cost{9.0f};
};

struct SignalTrackingInternalConfig {
  float kalman_noise_diff_coeff{1.0f};
  float speed_decay_ratio_on_loss{0.90f};
  float rcs_decay_ratio_on_loss{0.85f};
};

struct ResolvedDetectionConfig {
  config::engineering::DetectionConfig engineering{};
};

struct ResolvedTrackingConfig {
  config::engineering::TrackingConfig engineering{};
};

struct ResolvedLifecycleConfig {
  config::engineering::LifecycleRuntimeConfig engineering{};
};

struct ResolvedBeamControlConfig {
  model::PlatformAttitudeDeg platform_attitude_deg{};
};

struct InternalSignalPipelineConfig {
  SignalAssociationInternalConfig association{};
  SignalTrackingInternalConfig tracking{};
  SignalLifecycleInternalConfig lifecycle{};
  JammingEffectsConfig jamming_effects{};
  ControlProfileEffectsConfig control_profile_effects{};
  ResolvedDetectionConfig detection{};
  ResolvedTrackingConfig tracking_runtime{};
  ResolvedLifecycleConfig lifecycle_runtime{};
  ResolvedBeamControlConfig beam_control{};
};

inline config::engineering::AntennaPatternConfig ResolveAntennaPatternEngineering(
    const config::AntennaPatternConfig& semantic_pattern) {
  config::engineering::AntennaPatternConfig pattern;
  pattern.boresight_offset_deg = semantic_pattern.boresight_offset_deg;
  if (semantic_pattern.profile == config::AntennaPatternProfile::kLowSidelobe) {
    pattern.max_sidelobe_level_db = -30.0f;
    pattern.backlobe_level_db = -42.0f;
    return pattern;
  }
  if (semantic_pattern.profile == config::AntennaPatternProfile::kWideCoverage) {
    pattern.model_type = config::engineering::AntennaPatternModelType::kParabolicMainLobe;
    pattern.max_sidelobe_level_db = -18.0f;
    pattern.max_scan_loss_db = 8.0f;
  }
  return pattern;
}

inline config::engineering::DetectionConfig ResolveDetectionEngineering(
    const config::SignalDetectionConfig& semantic_detection) {
  config::engineering::DetectionConfig resolved;
  resolved.enable_physics_detection = semantic_detection.enable_physics_detection;
  resolved.antenna.pattern = ResolveAntennaPatternEngineering(semantic_detection.antenna_pattern);

  switch (semantic_detection.hardware_profile) {
    case config::RadarHardwareProfile::kLongRangeHighPower:
      resolved.transmitter.peak_power_w = 5.0e6f;
      resolved.transmitter.frequency_hz = 9.3e9f;
      resolved.transmitter.bandwidth_hz = 3.0e6f;
      resolved.transmitter.pulse_width_s = 18e-6f;
      resolved.transmitter.prf_hz = 220.0f;
      resolved.antenna.main_beam_gain_db = 38.0f;
      resolved.receiver.noise_figure_db = 3.0f;
      break;
    case config::RadarHardwareProfile::kLightweightLpi:
      resolved.transmitter.peak_power_w = 3.5e5f;
      resolved.transmitter.frequency_hz = 10.0e9f;
      resolved.transmitter.bandwidth_hz = 8.0e6f;
      resolved.transmitter.pulse_width_s = 8e-6f;
      resolved.transmitter.prf_hz = 600.0f;
      resolved.antenna.main_beam_gain_db = 31.0f;
      resolved.antenna.nominal_az_beamwidth_deg = 5.0f;
      resolved.antenna.nominal_el_beamwidth_deg = 5.0f;
      resolved.receiver.noise_figure_db = 5.0f;
      break;
    case config::RadarHardwareProfile::kGenericAirborneXBand:
    default:
      break;
  }

  switch (semantic_detection.intent_profile) {
    case config::DetectionIntentProfile::kDetectionPriority:
      resolved.pulse_count = 16;
      resolved.detection_policy.cfar_pfa = 2e-6f;
      resolved.detection_policy.min_snr_db = -12.0f;
      break;
    case config::DetectionIntentProfile::kTrackStabilityPriority:
      resolved.pulse_count = 8;
      resolved.detection_policy.cfar_pfa = 5e-7f;
      resolved.detection_policy.min_snr_db = -8.0f;
      break;
    case config::DetectionIntentProfile::kBalanced:
    default:
      break;
  }

  switch (semantic_detection.rcs_fusion_profile) {
    case config::RcsFusionProfile::kConservative:
      resolved.rcs_physics.enable_physical_rcs = true;
      resolved.rcs_physics.physics_mix_ratio = 0.25f;
      break;
    case config::RcsFusionProfile::kEnhanced:
      resolved.rcs_physics.enable_physical_rcs = true;
      resolved.rcs_physics.physics_mix_ratio = 0.60f;
      resolved.rcs_physics.cylinder_weight = 0.65f;
      break;
    case config::RcsFusionProfile::kDisabled:
    default:
      break;
  }

  switch (semantic_detection.intent_profile) {
    case config::DetectionIntentProfile::kDetectionPriority:
      resolved.min_detection_margin_db = -100.0f;
      break;
    case config::DetectionIntentProfile::kTrackStabilityPriority:
      resolved.min_detection_margin_db = -20.0f;
      break;
    case config::DetectionIntentProfile::kBalanced:
    default:
      resolved.min_detection_margin_db = -2.0f;
      break;
  }
  return resolved;
}

inline config::engineering::TrackingConfig ResolveTrackingEngineering(
    const config::SignalTrackingConfig& semantic_tracking) {
  config::engineering::TrackingConfig resolved;
  resolved.enable_kalman_filter = semantic_tracking.enable_tracking_filter;
  switch (semantic_tracking.policy_profile) {
    case config::TrackingPolicyProfile::kFastAssociation:
      resolved.kalman_measurement_noise_std = 6.0f;
      resolved.kalman_update_backend = config::engineering::KalmanUpdateBackend::kStandardKfJoseph;
      break;
    case config::TrackingPolicyProfile::kRobustAntiJamming:
      resolved.kalman_measurement_noise_std = 12.0f;
      resolved.kalman_update_backend = config::engineering::KalmanUpdateBackend::kUdKf;
      break;
    case config::TrackingPolicyProfile::kBalanced:
    default:
      break;
  }
  return resolved;
}

inline config::engineering::LifecycleRuntimeConfig ResolveLifecycleEngineering(
    const config::SignalLifecycleConfig& semantic_lifecycle) {
  config::engineering::LifecycleRuntimeConfig resolved;
  resolved.enable_imm_lifecycle = semantic_lifecycle.enable_imm_fusion;
  switch (semantic_lifecycle.policy_profile) {
    case config::LifecyclePolicyProfile::kFastConfirm:
      resolved.lifecycle_config.confirm_hits = 1U;
      resolved.lifecycle_config.max_miss_before_lost = 1U;
      resolved.lifecycle_config.max_lost_cycles = 3U;
      break;
    case config::LifecyclePolicyProfile::kHighPersistence:
      resolved.lifecycle_config.confirm_hits = 3U;
      resolved.lifecycle_config.max_miss_before_lost = 3U;
      resolved.lifecycle_config.max_lost_cycles = 8U;
      break;
    case config::LifecyclePolicyProfile::kBalanced:
    default:
      break;
  }
  return resolved;
}

inline InternalSignalPipelineConfig BuildBaselineInternalSignalPipelineConfig(
    const SignalPipelineConfig& public_config) {
  InternalSignalPipelineConfig internal;
  internal.detection.engineering = ResolveDetectionEngineering(public_config.detection);
  internal.tracking_runtime.engineering = ResolveTrackingEngineering(public_config.tracking);
  internal.lifecycle_runtime.engineering = ResolveLifecycleEngineering(public_config.lifecycle);
  return internal;
}

inline void ApplyInternalProfileTuning(InternalSignalPipelineProfile profile,
                                       InternalSignalPipelineConfig* internal_config) {
  if (internal_config == nullptr) {
    return;
  }
  if (profile == InternalSignalPipelineProfile::kTrackingPreset ||
      profile == InternalSignalPipelineProfile::kRobustPreset) {
    internal_config->tracking.speed_decay_ratio_on_loss = 0.95f;
    internal_config->tracking.rcs_decay_ratio_on_loss = 0.92f;
  }
  if (profile == InternalSignalPipelineProfile::kRobustPreset) {
    internal_config->association.unassigned_cost = 12.0f;
  }
}

inline void ApplyInternalImmDefaults(bool enable_imm_lifecycle,
                                     InternalSignalPipelineConfig* internal_config) {
  if (internal_config == nullptr || !enable_imm_lifecycle) {
    return;
  }
  internal_config->lifecycle.imm_model_noise_diff_coeffs = std::vector<float>{0.5f, 4.0f};
}

inline InternalSignalPipelineConfig BuildInternalSignalPipelineConfig(
    const SignalPipelineConfig& public_config) {
  InternalSignalPipelineConfig internal_config =
      BuildBaselineInternalSignalPipelineConfig(public_config);
  ApplyInternalProfileTuning(ResolveInternalProfileFromPublicConfig(public_config), &internal_config);
  ApplyInternalImmDefaults(internal_config.lifecycle_runtime.engineering.enable_imm_lifecycle,
                           &internal_config);
  return internal_config;
}

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_INTERNAL_SIGNAL_PIPELINE_CONFIG_H_
