/**
 * @file SignalEngineeringConfig.h
 * @brief 定义 Signal 子系统内部工程参数类型（非公开 API）。
 */

#ifndef AIRBORNE_RADAR_SRC_CONFIG_ENGINEERING_SIGNAL_ENGINEERING_CONFIG_H_
#define AIRBORNE_RADAR_SRC_CONFIG_ENGINEERING_SIGNAL_ENGINEERING_CONFIG_H_

#include <cstdint>

#include "1q/airborne_radar/config/RadarHardwareConfig.h"
#include "1q/airborne_radar/config/RadarPolicyConfig.h"
#include "1q/airborne_radar/model/RadarOrientationConfig.h"

namespace airborne_radar {
namespace config {
namespace engineering {

using AntennaPatternModelType = detection::AntennaPatternModelType;
using KalmanUpdateBackend = tracking::KalmanUpdateBackend;

struct TransmitterConfig {
  float peak_power_w{1e6f};
  float frequency_hz{3e9f};
  float bandwidth_hz{4.5e6f};
  float pulse_width_s{13e-6f};
  float prf_hz{300.0f};
  float transmit_loss_db{3.5f};
};

struct AntennaPatternConfig {
  AntennaPatternModelType model_type{
      AntennaPatternModelType::kGaussianMainLobe};
  float max_sidelobe_level_db{-20.0f};
  float backlobe_level_db{-35.0f};
  float scan_loss_coeff_db_per_deg2{0.0f};
  float max_scan_loss_db{6.0f};
  model::AzimuthElevationDeg boresight_offset_deg{};
};

struct AntennaConfig {
  float main_beam_gain_db{35.0f};
  float nominal_az_beamwidth_deg{4.0f};
  float nominal_el_beamwidth_deg{4.0f};
  engineering::AntennaPatternConfig pattern{};
  bool enable_directional_pattern{false};
};

struct ReceiverConfig {
  float noise_figure_db{4.0f};
  float receive_loss_db{2.0f};
};

struct DetectionPolicy {
  float cfar_pfa{1e-6f};
  float min_snr_db{-10.0f};
};

struct RcsPhysicsConfig {
  bool enable_physical_rcs{false};
  float frequency_hz{0.0f};
  float physics_mix_ratio{0.0f};
  float cylinder_weight{0.5f};
  float min_equivalent_radius_m{0.05f};
  float max_equivalent_radius_m{5.0f};
  float min_rcs_m2{0.01f};
  float max_rcs_m2{1000.0f};
  float bistatic_psi_offset_deg{5.0f};
};

struct DetectionConfig {
  bool enable_physics_detection{false};
  TransmitterConfig transmitter{};
  AntennaConfig antenna{};
  ReceiverConfig receiver{};
  DetectionPolicy detection_policy{};
  RcsPhysicsConfig rcs_physics{};
  float min_detection_margin_db{-2.0f};
  int pulse_count{10};
};

struct TrackingConfig {
  bool enable_kalman_filter{true};
  float kalman_measurement_noise_std{10.0f};
  KalmanUpdateBackend kalman_update_backend{
      KalmanUpdateBackend::kStandardKfJoseph};
  float speed_decay_ratio_on_loss{1.0f};
  float rcs_decay_ratio_on_loss{1.0f};
};

struct LifecycleConfig {
  std::uint32_t confirm_hits{3};
  std::uint32_t max_miss_before_lost{2};
  std::uint32_t max_lost_cycles{5};
};

struct LifecycleRuntimeConfig {
  LifecycleConfig lifecycle_config{};
  bool enable_imm_lifecycle{false};
};

}  // namespace engineering
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_CONFIG_ENGINEERING_SIGNAL_ENGINEERING_CONFIG_H_