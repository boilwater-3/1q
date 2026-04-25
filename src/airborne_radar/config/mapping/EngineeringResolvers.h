/**
 * @file EngineeringResolvers.h
 * @brief 定义四域配置子类型到 engineering 配置的共享映射函数。
 */

#ifndef AIRBORNE_RADAR_SRC_CONFIG_MAPPING_ENGINEERING_RESOLVERS_H_
#define AIRBORNE_RADAR_SRC_CONFIG_MAPPING_ENGINEERING_RESOLVERS_H_

#include "1q/airborne_radar/config/RadarHardwareConfig.h"
#include "1q/airborne_radar/config/RadarPolicyConfig.h"
#include "airborne_radar/config/SignalEngineeringConfig.h"

namespace airborne_radar {
namespace config {
namespace mapping {

inline engineering::AntennaPatternModelType ResolveAntennaPatternModelType(
    AntennaPatternModelType model_type) {
  switch (model_type) {
    case AntennaPatternModelType::kParabolicMainLobe:
      return engineering::AntennaPatternModelType::kParabolicMainLobe;
    case AntennaPatternModelType::kCosinePower:
      return engineering::AntennaPatternModelType::kCosinePower;
    case AntennaPatternModelType::kGaussianMainLobe:
    default:
      return engineering::AntennaPatternModelType::kGaussianMainLobe;
  }
}

inline engineering::KalmanUpdateBackend ResolveKalmanUpdateBackend(
    KalmanUpdateBackend backend) {
  switch (backend) {
    case KalmanUpdateBackend::kUdKf:
      return engineering::KalmanUpdateBackend::kUdKf;
    case KalmanUpdateBackend::kSrif:
      return engineering::KalmanUpdateBackend::kSrif;
    case KalmanUpdateBackend::kEkf:
      return engineering::KalmanUpdateBackend::kEkf;
    case KalmanUpdateBackend::kStandardKfJoseph:
    default:
      return engineering::KalmanUpdateBackend::kStandardKfJoseph;
  }
}

inline engineering::DetectionConfig ResolveDetectionEngineering(
    const DetectionConfig& detection) {
  engineering::DetectionConfig resolved;
  resolved.enable_physics_detection = detection.enable_physics_detection;
  resolved.transmitter.peak_power_w = detection.transmitter.peak_power_w;
  resolved.transmitter.frequency_hz = detection.transmitter.frequency_hz;
  resolved.transmitter.bandwidth_hz = detection.transmitter.bandwidth_hz;
  resolved.transmitter.pulse_width_s = detection.transmitter.pulse_width_s;
  resolved.transmitter.prf_hz = detection.transmitter.prf_hz;
  resolved.transmitter.transmit_loss_db = detection.transmitter.transmit_loss_db;
  resolved.antenna.main_beam_gain_db = detection.antenna.main_beam_gain_db;
  resolved.antenna.nominal_az_beamwidth_deg =
      detection.antenna.nominal_az_beamwidth_deg;
  resolved.antenna.nominal_el_beamwidth_deg =
      detection.antenna.nominal_el_beamwidth_deg;
  resolved.antenna.pattern.model_type =
      ResolveAntennaPatternModelType(detection.antenna.pattern.model_type);
  resolved.antenna.pattern.max_sidelobe_level_db =
      detection.antenna.pattern.max_sidelobe_level_db;
  resolved.antenna.pattern.backlobe_level_db =
      detection.antenna.pattern.backlobe_level_db;
  resolved.antenna.pattern.scan_loss_coeff_db_per_deg2 =
      detection.antenna.pattern.scan_loss_coeff_db_per_deg2;
  resolved.antenna.pattern.max_scan_loss_db =
      detection.antenna.pattern.max_scan_loss_db;
  resolved.antenna.pattern.boresight_offset_deg =
      detection.antenna.pattern.boresight_offset_deg;
  resolved.antenna.enable_directional_pattern =
      detection.antenna.enable_directional_pattern;
  resolved.receiver.noise_figure_db = detection.receiver.noise_figure_db;
  resolved.receiver.receive_loss_db = detection.receiver.receive_loss_db;
  resolved.detection_policy.cfar_pfa = detection.detection_policy.cfar_pfa;
  resolved.detection_policy.min_snr_db = detection.detection_policy.min_snr_db;
  resolved.rcs_physics.enable_physical_rcs = detection.rcs_physics.enable_physical_rcs;
  resolved.rcs_physics.frequency_hz = detection.rcs_physics.frequency_hz;
  resolved.rcs_physics.physics_mix_ratio = detection.rcs_physics.physics_mix_ratio;
  resolved.rcs_physics.cylinder_weight = detection.rcs_physics.cylinder_weight;
  resolved.rcs_physics.min_equivalent_radius_m =
      detection.rcs_physics.min_equivalent_radius_m;
  resolved.rcs_physics.max_equivalent_radius_m =
      detection.rcs_physics.max_equivalent_radius_m;
  resolved.rcs_physics.min_rcs_m2 = detection.rcs_physics.min_rcs_m2;
  resolved.rcs_physics.max_rcs_m2 = detection.rcs_physics.max_rcs_m2;
  resolved.rcs_physics.bistatic_psi_offset_deg =
      detection.rcs_physics.bistatic_psi_offset_deg;
  resolved.min_detection_margin_db = detection.min_detection_margin_db;
  resolved.pulse_count = detection.pulse_count;
  return resolved;
}

inline engineering::TrackingConfig ResolveTrackingEngineering(
    const TrackingConfig& tracking) {
  engineering::TrackingConfig resolved;
  resolved.enable_kalman_filter = tracking.enable_kalman_filter;
  resolved.kalman_measurement_noise_std = tracking.kalman_measurement_noise_std;
  resolved.kalman_update_backend =
      ResolveKalmanUpdateBackend(tracking.kalman_update_backend);
  return resolved;
}

inline engineering::LifecycleRuntimeConfig ResolveLifecycleEngineering(
    const LifecycleConfig& lifecycle) {
  engineering::LifecycleRuntimeConfig resolved;
  resolved.lifecycle_config.confirm_hits = lifecycle.confirm_hits;
  resolved.lifecycle_config.max_miss_before_lost = lifecycle.max_miss_before_lost;
  resolved.lifecycle_config.max_lost_cycles = lifecycle.max_lost_cycles;
  resolved.enable_imm_lifecycle = lifecycle.enable_imm_lifecycle;
  return resolved;
}

}  // namespace mapping
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_CONFIG_MAPPING_ENGINEERING_RESOLVERS_H_
