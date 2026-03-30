/**
 * @file InternalSignalPipelineConfig.h
 * @brief 定义 SignalPipeline 内部扩展配置。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_INTERNAL_SIGNAL_PIPELINE_CONFIG_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_INTERNAL_SIGNAL_PIPELINE_CONFIG_H_

#include <cmath>
#include <cstddef>
#include <vector>

#include "airborne_radar/common/config/SignalPipelinePresetSemantics.h"
#include "airborne_radar/signal/pipeline/SignalPipelineRuntimeTypes.h"
#include "airborne_radar/signal/tracking/LifecycleConfig.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

/**
 * @brief InternalSignalPipelineProfile 描述 internal tuning 的预设语义档位。
 */
enum class InternalSignalPipelineProfile {
  kBaseline = 0,
  kTrackingPreset,
  kRobustPreset,
};

namespace detail {

inline bool NearlyEqual(float lhs, float rhs) { return std::fabs(lhs - rhs) <= 1e-5f; }

inline bool EqualsEulerAnglesDeg(const common::config::EulerAnglesDeg& lhs,
                                 const common::config::EulerAnglesDeg& rhs) {
  return NearlyEqual(lhs.yaw_deg, rhs.yaw_deg) && NearlyEqual(lhs.pitch_deg, rhs.pitch_deg) &&
         NearlyEqual(lhs.roll_deg, rhs.roll_deg);
}

inline bool EqualsAzimuthElevationDeg(const common::config::AzimuthElevationDeg& lhs,
                                      const common::config::AzimuthElevationDeg& rhs) {
  return NearlyEqual(lhs.az_deg, rhs.az_deg) && NearlyEqual(lhs.el_deg, rhs.el_deg);
}

inline bool EqualsAzimuthElevationLimitsDeg(const common::config::AzimuthElevationLimitsDeg& lhs,
                                            const common::config::AzimuthElevationLimitsDeg& rhs) {
  return NearlyEqual(lhs.az_min_deg, rhs.az_min_deg) && NearlyEqual(lhs.az_max_deg, rhs.az_max_deg) &&
         NearlyEqual(lhs.el_min_deg, rhs.el_min_deg) && NearlyEqual(lhs.el_max_deg, rhs.el_max_deg);
}

inline bool EqualsCommandedBeamwidthDeg(const common::config::CommandedBeamwidthDeg& lhs,
                                        const common::config::CommandedBeamwidthDeg& rhs) {
  return NearlyEqual(lhs.commanded_az_beamwidth_deg, rhs.commanded_az_beamwidth_deg) &&
         NearlyEqual(lhs.commanded_el_beamwidth_deg, rhs.commanded_el_beamwidth_deg);
}

inline bool EqualsRadarOrientationConfig(const common::config::RadarOrientationConfig& lhs,
                                         const common::config::RadarOrientationConfig& rhs) {
  return EqualsEulerAnglesDeg(lhs.mount_angles_deg, rhs.mount_angles_deg) &&
         EqualsAzimuthElevationDeg(lhs.scan_center_deg, rhs.scan_center_deg) &&
         EqualsAzimuthElevationLimitsDeg(lhs.mechanical_scan_limits_deg,
                                         rhs.mechanical_scan_limits_deg) &&
         EqualsAzimuthElevationLimitsDeg(lhs.electronic_scan_limits_deg,
                                         rhs.electronic_scan_limits_deg) &&
         lhs.scan_start_position == rhs.scan_start_position &&
         lhs.scan_sequence == rhs.scan_sequence && lhs.work_sub_mode == rhs.work_sub_mode &&
         EqualsAzimuthElevationDeg(lhs.dwell_center_deg, rhs.dwell_center_deg) &&
         lhs.commanded_beamwidth_enabled == rhs.commanded_beamwidth_enabled &&
         EqualsCommandedBeamwidthDeg(lhs.commanded_beamwidth_deg, rhs.commanded_beamwidth_deg) &&
         lhs.stabilization_mode == rhs.stabilization_mode;
}

inline bool EqualsAntennaPatternConfig(const common::config::AntennaPatternConfig& lhs,
                                       const common::config::AntennaPatternConfig& rhs) {
  return lhs.model_type == rhs.model_type &&
         NearlyEqual(lhs.max_sidelobe_level_db, rhs.max_sidelobe_level_db) &&
         NearlyEqual(lhs.backlobe_level_db, rhs.backlobe_level_db) &&
         NearlyEqual(lhs.scan_loss_coeff_db_per_deg2, rhs.scan_loss_coeff_db_per_deg2) &&
         NearlyEqual(lhs.max_scan_loss_db, rhs.max_scan_loss_db) &&
         EqualsAzimuthElevationDeg(lhs.boresight_offset_deg, rhs.boresight_offset_deg);
}

inline bool EqualsTransmitterConfig(const common::config::TransmitterConfig& lhs,
                                    const common::config::TransmitterConfig& rhs) {
  return NearlyEqual(lhs.peak_power_w, rhs.peak_power_w) &&
         NearlyEqual(lhs.frequency_hz, rhs.frequency_hz) &&
         NearlyEqual(lhs.bandwidth_hz, rhs.bandwidth_hz) &&
         NearlyEqual(lhs.pulse_width_s, rhs.pulse_width_s) &&
         NearlyEqual(lhs.prf_hz, rhs.prf_hz) &&
         NearlyEqual(lhs.transmit_loss_db, rhs.transmit_loss_db);
}

inline bool EqualsAntennaConfig(const common::config::AntennaConfig& lhs,
                                const common::config::AntennaConfig& rhs) {
  return NearlyEqual(lhs.main_beam_gain_db, rhs.main_beam_gain_db) &&
         NearlyEqual(lhs.nominal_az_beamwidth_deg, rhs.nominal_az_beamwidth_deg) &&
         NearlyEqual(lhs.nominal_el_beamwidth_deg, rhs.nominal_el_beamwidth_deg) &&
         EqualsAntennaPatternConfig(lhs.pattern, rhs.pattern) &&
         lhs.enable_directional_pattern == rhs.enable_directional_pattern;
}

inline bool EqualsReceiverConfig(const common::config::ReceiverConfig& lhs,
                                 const common::config::ReceiverConfig& rhs) {
  return NearlyEqual(lhs.noise_figure_db, rhs.noise_figure_db) &&
         NearlyEqual(lhs.receive_loss_db, rhs.receive_loss_db);
}

inline bool EqualsDetectionPolicy(const common::config::DetectionPolicy& lhs,
                                  const common::config::DetectionPolicy& rhs) {
  return NearlyEqual(lhs.cfar_pfa, rhs.cfar_pfa) && NearlyEqual(lhs.min_snr_db, rhs.min_snr_db);
}

inline bool EqualsRadarSystemConfig(const common::config::RadarSystemConfig& lhs,
                                    const common::config::RadarSystemConfig& rhs) {
  return EqualsTransmitterConfig(lhs.transmitter, rhs.transmitter) &&
         EqualsAntennaConfig(lhs.antenna, rhs.antenna) &&
         EqualsReceiverConfig(lhs.receiver, rhs.receiver) &&
         EqualsDetectionPolicy(lhs.detection, rhs.detection);
}

inline bool EqualsSignalDetectionConfig(const common::config::SignalDetectionConfig& lhs,
                                        const common::config::SignalDetectionConfig& rhs) {
  return lhs.enable_physics_detection == rhs.enable_physics_detection &&
         EqualsRadarSystemConfig(lhs.radar_system, rhs.radar_system) &&
         NearlyEqual(lhs.min_detection_margin_db, rhs.min_detection_margin_db) &&
         lhs.pulse_count == rhs.pulse_count;
}

inline bool EqualsSignalBeamControlConfig(const common::config::SignalBeamControlConfig& lhs,
                                          const common::config::SignalBeamControlConfig& rhs) {
  return EqualsRadarOrientationConfig(lhs.radar_orientation, rhs.radar_orientation) &&
         EqualsEulerAnglesDeg(lhs.platform_attitude_deg, rhs.platform_attitude_deg);
}

inline bool EqualsSignalTrackingConfig(const common::config::SignalTrackingConfig& lhs,
                                       const common::config::SignalTrackingConfig& rhs) {
  return lhs.enable_kalman_filter == rhs.enable_kalman_filter &&
         NearlyEqual(lhs.kalman_measurement_noise_std, rhs.kalman_measurement_noise_std);
}

inline bool EqualsLifecycleConfig(const common::config::LifecycleConfig& lhs,
                                  const common::config::LifecycleConfig& rhs) {
  return lhs.confirm_hits == rhs.confirm_hits &&
         lhs.max_miss_before_lost == rhs.max_miss_before_lost &&
         lhs.max_lost_cycles == rhs.max_lost_cycles;
}

inline bool EqualsSignalLifecycleConfigIgnoringImm(
    const common::config::SignalLifecycleConfig& lhs,
    const common::config::SignalLifecycleConfig& rhs) {
  return lhs.enable_auto_lifecycle_manager == rhs.enable_auto_lifecycle_manager &&
         EqualsLifecycleConfig(lhs.lifecycle_config, rhs.lifecycle_config);
}

inline bool MatchesProfileSignatureIgnoringImm(const SignalPipelineConfig& public_config,
                                               const SignalPipelineConfig& signature) {
  return EqualsSignalDetectionConfig(public_config.detection, signature.detection) &&
         EqualsSignalBeamControlConfig(public_config.beam_control, signature.beam_control) &&
         EqualsSignalTrackingConfig(public_config.tracking, signature.tracking) &&
         EqualsSignalLifecycleConfigIgnoringImm(public_config.lifecycle, signature.lifecycle);
}

}  // namespace detail

/**
 * @brief 解析 public 配置对应的 internal preset profile。
 */
inline InternalSignalPipelineProfile ResolveInternalProfileFromPublicConfig(
    const SignalPipelineConfig& public_config) {
  const SignalPipelineConfig robust_signature =
      common::config::internal::BuildHighRobustnessPresetConfig();
  if (detail::MatchesProfileSignatureIgnoringImm(public_config, robust_signature)) {
    return InternalSignalPipelineProfile::kRobustPreset;
  }

  const SignalPipelineConfig tracking_signature =
      common::config::internal::BuildTrackingMissionPresetConfig();
  if (detail::MatchesProfileSignatureIgnoringImm(public_config, tracking_signature)) {
    return InternalSignalPipelineProfile::kTrackingPreset;
  }

  return InternalSignalPipelineProfile::kBaseline;
}

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

inline InternalSignalPipelineConfig BuildBaselineInternalSignalPipelineConfig() {
  return InternalSignalPipelineConfig();
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
  if (internal_config == nullptr) {
    return;
  }
  if (!enable_imm_lifecycle) {
    return;
  }
  internal_config->lifecycle.imm_model_noise_diff_coeffs = std::vector<float>{0.5f, 4.0f};
}

/**
 * @brief 根据 public SignalPipelineConfig 构建默认内部扩展配置。
 */
inline InternalSignalPipelineConfig BuildInternalSignalPipelineConfig(
    const SignalPipelineConfig& public_config) {
  InternalSignalPipelineConfig internal_config = BuildBaselineInternalSignalPipelineConfig();
  ApplyInternalProfileTuning(ResolveInternalProfileFromPublicConfig(public_config), &internal_config);
  ApplyInternalImmDefaults(public_config.lifecycle.enable_imm_lifecycle, &internal_config);
  return internal_config;
}

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_INTERNAL_SIGNAL_PIPELINE_CONFIG_H_
