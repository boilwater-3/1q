/**
 * @file InternalPipelineConfig.h
 * @brief 定义机载雷达流水线内部扩展配置。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_INTERNAL_PIPELINE_CONFIG_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_INTERNAL_PIPELINE_CONFIG_H_

#include <cstddef>
#include <vector>

#include "airborne_radar/config/internal/ExpertToEngineeringMapping.h"
#include "airborne_radar/config/engineering/SignalEngineeringConfig.h"
#include "airborne_radar/signal/pipeline/config/SignalPipelineRuntimeTypes.h"
#include "airborne_radar/signal/tracking/LifecycleConfig.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

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

struct InternalPipelineConfig {
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

inline InternalPipelineConfig BuildBaselineInternalPipelineConfig(
    const PipelineConfig& public_config) {
  InternalPipelineConfig internal;
  internal.detection.engineering =
      config::internal::ResolveDetectionEngineering(public_config.expert.detection);
  internal.tracking_runtime.engineering =
      config::internal::ResolveTrackingEngineering(public_config.expert.tracking);
  internal.lifecycle_runtime.engineering =
      config::internal::ResolveLifecycleEngineering(public_config.expert.lifecycle);
  internal.tracking.speed_decay_ratio_on_loss =
      public_config.expert.tracking.speed_decay_ratio_on_loss;
  internal.tracking.rcs_decay_ratio_on_loss =
      public_config.expert.tracking.rcs_decay_ratio_on_loss;
  internal.association.unassigned_cost = public_config.expert.association.unassigned_cost;
  return internal;
}

inline void ApplyInternalImmDefaults(bool enable_imm_lifecycle,
                                     InternalPipelineConfig* internal_config) {
  if (internal_config == nullptr || !enable_imm_lifecycle) {
    return;
  }
  internal_config->lifecycle.imm_model_noise_diff_coeffs = std::vector<float>{0.5f, 4.0f};
}

inline InternalPipelineConfig BuildInternalPipelineConfig(
    const PipelineConfig& public_config) {
  InternalPipelineConfig internal_config =
      BuildBaselineInternalPipelineConfig(public_config);
  ApplyInternalImmDefaults(internal_config.lifecycle_runtime.engineering.enable_imm_lifecycle,
                           &internal_config);
  return internal_config;
}

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_INTERNAL_PIPELINE_CONFIG_H_
