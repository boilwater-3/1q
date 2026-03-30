/**
 * @file InternalSignalPipelineConfig.h
 * @brief 定义 SignalPipeline 内部扩展配置。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_INTERNAL_SIGNAL_PIPELINE_CONFIG_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_INTERNAL_SIGNAL_PIPELINE_CONFIG_H_

#include <cstddef>
#include <vector>

#include "airborne_radar/signal/pipeline/SignalPipelineRuntimeTypes.h"
#include "airborne_radar/signal/tracking/LifecycleConfig.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

/**
 * @brief JammingEffectsConfig 描述干扰效应建模的内部整定参数。
 */
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

/**
 * @brief ControlProfileEffectsConfig 描述控制轮廓对运行时配置的内部修正参数。
 */
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

/**
 * @brief SignalLifecycleInternalConfig 描述生命周期装配的内部参数。
 */
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

/**
 * @brief SignalAssociationInternalConfig 描述关联域内部整定参数。
 */
struct SignalAssociationInternalConfig {
  float unassigned_cost{9.0f};
};

/**
 * @brief SignalTrackingInternalConfig 描述跟踪域内部整定参数。
 */
struct SignalTrackingInternalConfig {
  float kalman_noise_diff_coeff{1.0f};
  float speed_decay_ratio_on_loss{0.90f};
  float rcs_decay_ratio_on_loss{0.85f};
};

/**
 * @brief InternalSignalPipelineConfig 汇聚 public config 之外的内部整定参数。
 */
struct InternalSignalPipelineConfig {
  SignalAssociationInternalConfig association{};
  SignalTrackingInternalConfig tracking{};
  SignalLifecycleInternalConfig lifecycle{};
  JammingEffectsConfig jamming_effects{};
  ControlProfileEffectsConfig control_profile_effects{};
};

/**
 * @brief 根据 public SignalPipelineConfig 构建默认内部扩展配置。
 */
inline InternalSignalPipelineConfig BuildInternalSignalPipelineConfig(
    const SignalPipelineConfig& public_config) {
  InternalSignalPipelineConfig internal_config;
  if (public_config.lifecycle.lifecycle_config.confirm_hits >= 2U ||
      public_config.tracking.kalman_measurement_noise_std >= 3.0f) {
    internal_config.tracking.speed_decay_ratio_on_loss = 0.95f;
    internal_config.tracking.rcs_decay_ratio_on_loss = 0.92f;
  }
  if (public_config.lifecycle.lifecycle_config.max_miss_before_lost >= 3U ||
      public_config.lifecycle.lifecycle_config.max_lost_cycles >= 8U) {
    internal_config.association.unassigned_cost = 12.0f;
  }
  if (public_config.lifecycle.enable_imm_lifecycle) {
    internal_config.lifecycle.imm_model_noise_diff_coeffs = std::vector<float>{0.5f, 4.0f};
  }
  return internal_config;
}

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_INTERNAL_SIGNAL_PIPELINE_CONFIG_H_
