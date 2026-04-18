/**
 * @file ExpertToEngineeringMapping.h
 * @brief 定义 expert 配置到 engineering 配置的内部映射函数。
 */

#ifndef AIRBORNE_RADAR_SRC_CONFIG_INTERNAL_EXPERT_TO_ENGINEERING_MAPPING_H_
#define AIRBORNE_RADAR_SRC_CONFIG_INTERNAL_EXPERT_TO_ENGINEERING_MAPPING_H_

#include "1q/airborne_radar/config/expert/ExpertPipelineConfig.h"
#include "airborne_radar/config/engineering/SignalEngineeringConfig.h"

namespace airborne_radar {
namespace config {
namespace internal {

inline engineering::AntennaPatternModelType ResolveExpertAntennaPatternModelType(
    expert::AntennaPatternModelType model_type) {
  switch (model_type) {
    case expert::AntennaPatternModelType::kParabolicMainLobe:
      return engineering::AntennaPatternModelType::kParabolicMainLobe;
    case expert::AntennaPatternModelType::kCosinePower:
      return engineering::AntennaPatternModelType::kCosinePower;
    case expert::AntennaPatternModelType::kGaussianMainLobe:
    default:
      return engineering::AntennaPatternModelType::kGaussianMainLobe;
  }
}

inline engineering::KalmanUpdateBackend ResolveExpertKalmanUpdateBackend(
    expert::KalmanUpdateBackend backend) {
  switch (backend) {
    case expert::KalmanUpdateBackend::kUdKf:
      return engineering::KalmanUpdateBackend::kUdKf;
    case expert::KalmanUpdateBackend::kSrif:
      return engineering::KalmanUpdateBackend::kSrif;
    case expert::KalmanUpdateBackend::kStandardKfJoseph:
    default:
      return engineering::KalmanUpdateBackend::kStandardKfJoseph;
  }
}

inline engineering::DetectionConfig ResolveDetectionEngineering(
    const expert::DetectionConfig& expert_detection) {
  engineering::DetectionConfig resolved;
  resolved.enable_physics_detection = expert_detection.enable_physics_detection;
  resolved.transmitter.peak_power_w = expert_detection.transmitter.peak_power_w;
  resolved.transmitter.frequency_hz = expert_detection.transmitter.frequency_hz;
  resolved.transmitter.bandwidth_hz = expert_detection.transmitter.bandwidth_hz;
  resolved.transmitter.pulse_width_s = expert_detection.transmitter.pulse_width_s;
  resolved.transmitter.prf_hz = expert_detection.transmitter.prf_hz;
  resolved.transmitter.transmit_loss_db = expert_detection.transmitter.transmit_loss_db;
  resolved.antenna.main_beam_gain_db = expert_detection.antenna.main_beam_gain_db;
  resolved.antenna.nominal_az_beamwidth_deg =
      expert_detection.antenna.nominal_az_beamwidth_deg;
  resolved.antenna.nominal_el_beamwidth_deg =
      expert_detection.antenna.nominal_el_beamwidth_deg;
  resolved.antenna.pattern.model_type =
      ResolveExpertAntennaPatternModelType(expert_detection.antenna.pattern.model_type);
  resolved.antenna.pattern.max_sidelobe_level_db =
      expert_detection.antenna.pattern.max_sidelobe_level_db;
  resolved.antenna.pattern.backlobe_level_db =
      expert_detection.antenna.pattern.backlobe_level_db;
  resolved.antenna.pattern.scan_loss_coeff_db_per_deg2 =
      expert_detection.antenna.pattern.scan_loss_coeff_db_per_deg2;
  resolved.antenna.pattern.max_scan_loss_db =
      expert_detection.antenna.pattern.max_scan_loss_db;
  resolved.antenna.pattern.boresight_offset_deg =
      expert_detection.antenna.pattern.boresight_offset_deg;
  resolved.antenna.enable_directional_pattern =
      expert_detection.antenna.enable_directional_pattern;
  resolved.receiver.noise_figure_db = expert_detection.receiver.noise_figure_db;
  resolved.receiver.receive_loss_db = expert_detection.receiver.receive_loss_db;
  resolved.detection_policy.cfar_pfa = expert_detection.detection_policy.cfar_pfa;
  resolved.detection_policy.min_snr_db = expert_detection.detection_policy.min_snr_db;
  resolved.rcs_physics.enable_physical_rcs = expert_detection.rcs_physics.enable_physical_rcs;
  resolved.rcs_physics.frequency_hz = expert_detection.rcs_physics.frequency_hz;
  resolved.rcs_physics.physics_mix_ratio = expert_detection.rcs_physics.physics_mix_ratio;
  resolved.rcs_physics.cylinder_weight = expert_detection.rcs_physics.cylinder_weight;
  resolved.rcs_physics.min_equivalent_radius_m =
      expert_detection.rcs_physics.min_equivalent_radius_m;
  resolved.rcs_physics.max_equivalent_radius_m =
      expert_detection.rcs_physics.max_equivalent_radius_m;
  resolved.rcs_physics.min_rcs_m2 = expert_detection.rcs_physics.min_rcs_m2;
  resolved.rcs_physics.max_rcs_m2 = expert_detection.rcs_physics.max_rcs_m2;
  resolved.rcs_physics.bistatic_psi_offset_deg =
      expert_detection.rcs_physics.bistatic_psi_offset_deg;
  resolved.min_detection_margin_db = expert_detection.min_detection_margin_db;
  resolved.pulse_count = expert_detection.pulse_count;
  return resolved;
}

inline engineering::TrackingConfig ResolveTrackingEngineering(
    const expert::TrackingConfig& expert_tracking) {
  engineering::TrackingConfig resolved;
  resolved.enable_kalman_filter = expert_tracking.enable_kalman_filter;
  resolved.kalman_measurement_noise_std = expert_tracking.kalman_measurement_noise_std;
  resolved.kalman_update_backend =
      ResolveExpertKalmanUpdateBackend(expert_tracking.kalman_update_backend);
  return resolved;
}

inline engineering::LifecycleRuntimeConfig ResolveLifecycleEngineering(
    const expert::LifecycleConfig& expert_lifecycle) {
  engineering::LifecycleRuntimeConfig resolved;
  resolved.lifecycle_config.confirm_hits = expert_lifecycle.confirm_hits;
  resolved.lifecycle_config.max_miss_before_lost = expert_lifecycle.max_miss_before_lost;
  resolved.lifecycle_config.max_lost_cycles = expert_lifecycle.max_lost_cycles;
  resolved.enable_imm_lifecycle = expert_lifecycle.enable_imm_lifecycle;
  return resolved;
}

}  // namespace internal
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_CONFIG_INTERNAL_EXPERT_TO_ENGINEERING_MAPPING_H_
