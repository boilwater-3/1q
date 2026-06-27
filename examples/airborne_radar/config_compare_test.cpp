#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>

#include "1q/airborne_radar/airborne_radar.hpp"
#include "1q/coordinate/types.h"
#include "config_loader.h"

namespace ar = airborne_radar;
namespace ar_config = airborne_radar::config;
namespace ar_env = airborne_radar::config;
namespace ar_model = airborne_radar::session;
namespace ar_session = airborne_radar::session;

namespace {

ar_config::AzimuthElevationDeg MakeAzEl(float az_deg, float el_deg) {
  ar_config::AzimuthElevationDeg v;
  v.az_deg = az_deg;
  v.el_deg = el_deg;
  return v;
}

/// Original hardcoded builder from git history.
ar_config::RadarSessionConfig MakeWideAreaSearchConfig() {
  ar_config::RadarSessionConfig config =
      ar_config::RadarSessionConfigBuilder()
          .Detection()
          .EnablePhysicsDetection(false)
          .WithHardwareProfile(ar_config::profiles::RadarHardwareProfile::kLongRangeHighPower)
          .WithDetectionIntentProfile(
              ar_config::profiles::DetectionIntentProfile::kDetectionPriority)
          .WithAntennaPatternProfile(ar_config::profiles::AntennaPatternProfile::kStandard)
          .End()
          .Tracking()
          .EnableTrackingFilter(true)
          .WithTrackingPolicyProfile(ar_config::profiles::TrackingPolicyProfile::kFastAssociation)
          .End()
          .Lifecycle()
          .WithLifecyclePolicyProfile(ar_config::profiles::LifecyclePolicyProfile::kFastConfirm)
          .End()
          .Environment()
          .WithJammingSensitivityProfile(ar_env::JammingSensitivityProfile::kBalanced)
          .End()
          .Build();
  config.mission.orientation.work_mode = ar_config::RadarWorkMode::kTas;
  config.mission.orientation.scan_center_deg = MakeAzEl(0.0f, 0.0f);
  return config;
}

int failed = 0;
#define CHECK_EQ(a, b, name)                                                    \
  do {                                                                          \
    if (std::abs((a) - (b)) > 0.0001f) {                                        \
      std::cerr << "  FAIL " << (name) << ": " << (a) << " != " << (b) << "\n"; \
      ++failed;                                                                 \
    }                                                                           \
  } while (0)

#define CHECK_BOOL(a, b, name)                                                  \
  do {                                                                          \
    if ((a) != (b)) {                                                           \
      std::cerr << "  FAIL " << (name) << ": " << (a) << " != " << (b) << "\n"; \
      ++failed;                                                                 \
    }                                                                           \
  } while (0)

#define CHECK_INT(a, b, name)                                                   \
  do {                                                                          \
    if ((a) != (b)) {                                                           \
      std::cerr << "  FAIL " << (name) << ": " << (a) << " != " << (b) << "\n"; \
      ++failed;                                                                 \
    }                                                                           \
  } while (0)

void CheckAzEl(const ar_config::AzimuthElevationDeg& a, const ar_config::AzimuthElevationDeg& b,
               const char* prefix) {
  CHECK_EQ(a.az_deg, b.az_deg, (std::string(prefix) + ".az_deg").c_str());
  CHECK_EQ(a.el_deg, b.el_deg, (std::string(prefix) + ".el_deg").c_str());
}

void CheckAzElLimits(const ar_config::AzimuthElevationLimitsDeg& a,
                     const ar_config::AzimuthElevationLimitsDeg& b, const char* prefix) {
  CHECK_EQ(a.az_min_deg, b.az_min_deg, (std::string(prefix) + ".az_min_deg").c_str());
  CHECK_EQ(a.az_max_deg, b.az_max_deg, (std::string(prefix) + ".az_max_deg").c_str());
  CHECK_EQ(a.el_min_deg, b.el_min_deg, (std::string(prefix) + ".el_min_deg").c_str());
  CHECK_EQ(a.el_max_deg, b.el_max_deg, (std::string(prefix) + ".el_max_deg").c_str());
}

void CheckCmdBeamwidth(const ar_config::CommandedBeamwidthDeg& a,
                       const ar_config::CommandedBeamwidthDeg& b, const char* prefix) {
  CHECK_EQ(a.commanded_az_beamwidth_deg, b.commanded_az_beamwidth_deg,
           (std::string(prefix) + ".commanded_az_beamwidth_deg").c_str());
  CHECK_EQ(a.commanded_el_beamwidth_deg, b.commanded_el_beamwidth_deg,
           (std::string(prefix) + ".commanded_el_beamwidth_deg").c_str());
}

void CheckTransmitter(const ar_config::detection::TransmitterConfig& a,
                      const ar_config::detection::TransmitterConfig& b) {
  CHECK_EQ(a.peak_power_w, b.peak_power_w, "transmitter.peak_power_w");
  CHECK_EQ(a.frequency_hz, b.frequency_hz, "transmitter.frequency_hz");
  CHECK_EQ(a.bandwidth_hz, b.bandwidth_hz, "transmitter.bandwidth_hz");
  CHECK_EQ(a.pulse_width_s, b.pulse_width_s, "transmitter.pulse_width_s");
  CHECK_EQ(a.prf_hz, b.prf_hz, "transmitter.prf_hz");
  CHECK_EQ(a.transmit_loss_db, b.transmit_loss_db, "transmitter.transmit_loss_db");
}

void CheckAntennaPattern(const ar_config::detection::AntennaPatternConfig& a,
                         const ar_config::detection::AntennaPatternConfig& b) {
  CHECK_INT(static_cast<int>(a.model_type), static_cast<int>(b.model_type),
            "antenna_pattern.model_type");
  CHECK_EQ(a.max_sidelobe_level_db, b.max_sidelobe_level_db,
           "antenna_pattern.max_sidelobe_level_db");
  CHECK_EQ(a.backlobe_level_db, b.backlobe_level_db, "antenna_pattern.backlobe_level_db");
  CHECK_EQ(a.scan_loss_coeff_db_per_deg2, b.scan_loss_coeff_db_per_deg2,
           "antenna_pattern.scan_loss_coeff_db_per_deg2");
  CHECK_EQ(a.max_scan_loss_db, b.max_scan_loss_db, "antenna_pattern.max_scan_loss_db");
  CheckAzEl(a.boresight_offset_deg, b.boresight_offset_deg, "antenna_pattern.boresight_offset_deg");
}

void CheckAntenna(const ar_config::detection::AntennaConfig& a,
                  const ar_config::detection::AntennaConfig& b) {
  CHECK_EQ(a.main_beam_gain_db, b.main_beam_gain_db, "antenna.main_beam_gain_db");
  CHECK_EQ(a.nominal_az_beamwidth_deg, b.nominal_az_beamwidth_deg,
           "antenna.nominal_az_beamwidth_deg");
  CHECK_EQ(a.nominal_el_beamwidth_deg, b.nominal_el_beamwidth_deg,
           "antenna.nominal_el_beamwidth_deg");
  CheckAntennaPattern(a.pattern, b.pattern);
  CHECK_BOOL(a.enable_directional_pattern, b.enable_directional_pattern,
             "antenna.enable_directional_pattern");
}

void CheckReceiver(const ar_config::detection::ReceiverConfig& a,
                   const ar_config::detection::ReceiverConfig& b) {
  CHECK_EQ(a.noise_figure_db, b.noise_figure_db, "receiver.noise_figure_db");
  CHECK_EQ(a.receive_loss_db, b.receive_loss_db, "receiver.receive_loss_db");
}

void CheckDetectionPolicy(const ar_config::detection::DetectionPolicyConfig& a,
                          const ar_config::detection::DetectionPolicyConfig& b) {
  CHECK_EQ(a.cfar_pfa, b.cfar_pfa, "detection_policy.cfar_pfa");
  CHECK_EQ(a.min_snr_db, b.min_snr_db, "detection_policy.min_snr_db");
}

void CheckRcsPhysics(const ar_config::detection::RcsPhysicsConfig& a,
                     const ar_config::detection::RcsPhysicsConfig& b) {
  CHECK_BOOL(a.enable_physical_rcs, b.enable_physical_rcs, "rcs.enable_physical_rcs");
  CHECK_EQ(a.frequency_hz, b.frequency_hz, "rcs.frequency_hz");
  CHECK_EQ(a.physics_mix_ratio, b.physics_mix_ratio, "rcs.physics_mix_ratio");
  CHECK_EQ(a.cylinder_weight, b.cylinder_weight, "rcs.cylinder_weight");
  CHECK_EQ(a.min_equivalent_radius_m, b.min_equivalent_radius_m, "rcs.min_equivalent_radius_m");
  CHECK_EQ(a.max_equivalent_radius_m, b.max_equivalent_radius_m, "rcs.max_equivalent_radius_m");
  CHECK_EQ(a.min_rcs_m2, b.min_rcs_m2, "rcs.min_rcs_m2");
  CHECK_EQ(a.max_rcs_m2, b.max_rcs_m2, "rcs.max_rcs_m2");
  CHECK_EQ(a.bistatic_psi_offset_deg, b.bistatic_psi_offset_deg, "rcs.bistatic_psi_offset_deg");
}

void CompareConfigs(const ar_config::RadarSessionConfig& a,
                    const ar_config::RadarSessionConfig& b) {
  std::cout << "=== Comparing SessionConfig ===\n";

  // hardware
  const auto& da = a.hardware;
  const auto& db = b.hardware;
  CHECK_BOOL(da.enable_physics_detection, db.enable_physics_detection,
             "detection.enable_physics_detection");
  CheckTransmitter(da.transmitter, db.transmitter);
  CheckAntenna(da.antenna, db.antenna);
  CheckReceiver(da.receiver, db.receiver);
  CheckDetectionPolicy(da.detection_policy, db.detection_policy);
  CheckRcsPhysics(da.rcs_physics, db.rcs_physics);
  CHECK_EQ(da.min_detection_margin_db, db.min_detection_margin_db,
           "detection.min_detection_margin_db");
  CHECK_INT(da.pulse_count, db.pulse_count, "detection.pulse_count");

  // mission.orientation
  const auto& oa = a.mission.orientation;
  const auto& ob = b.mission.orientation;
  CHECK_EQ(oa.mount_angles_deg.yaw_deg, ob.mount_angles_deg.yaw_deg,
           "orientation.mount_angles.yaw");
  CHECK_EQ(oa.mount_angles_deg.pitch_deg, ob.mount_angles_deg.pitch_deg,
           "orientation.mount_angles.pitch");
  CHECK_EQ(oa.mount_angles_deg.roll_deg, ob.mount_angles_deg.roll_deg,
           "orientation.mount_angles.roll");
  CheckAzEl(oa.scan_center_deg, ob.scan_center_deg, "orientation.scan_center");
  CheckAzElLimits(oa.mechanical_scan_limits_deg, ob.mechanical_scan_limits_deg,
                  "orientation.mechanical_scan_limits");
  CheckAzElLimits(oa.electronic_scan_limits_deg, ob.electronic_scan_limits_deg,
                  "orientation.electronic_scan_limits");
  CHECK_INT(static_cast<int>(oa.scan_start_position), static_cast<int>(ob.scan_start_position),
            "orientation.scan_start_position");
  CHECK_INT(static_cast<int>(oa.scan_sequence), static_cast<int>(ob.scan_sequence),
            "orientation.scan_sequence");
  CHECK_INT(static_cast<int>(oa.work_mode), static_cast<int>(ob.work_mode),
            "orientation.work_mode");
  CHECK_BOOL(oa.commanded_beamwidth_enabled, ob.commanded_beamwidth_enabled,
             "orientation.commanded_beamwidth_enabled");
  CheckCmdBeamwidth(oa.commanded_beamwidth_deg, ob.commanded_beamwidth_deg,
                    "orientation.commanded_beamwidth_deg");
  CHECK_INT(static_cast<int>(oa.stabilization_mode), static_cast<int>(ob.stabilization_mode),
            "orientation.stabilization_mode");

  // policy subtree
  // beam_control.pointing
  CheckCmdBeamwidth(a.policy.beam_control.pointing.nominal_beamwidth_deg,
                    b.policy.beam_control.pointing.nominal_beamwidth_deg,
                    "beam_control.pointing.nominal_beamwidth");

  // beam_control.scheduler
  CHECK_INT(a.policy.beam_control.scheduler.azimuth_step_count_hint,
            b.policy.beam_control.scheduler.azimuth_step_count_hint,
            "beam_control.scheduler.azimuth_step_count_hint");
  CHECK_INT(a.policy.beam_control.scheduler.elevation_step_count_hint,
            b.policy.beam_control.scheduler.elevation_step_count_hint,
            "beam_control.scheduler.elevation_step_count_hint");
  CHECK_BOOL(a.policy.beam_control.scheduler.prefer_dense_tas_sampling,
             b.policy.beam_control.scheduler.prefer_dense_tas_sampling,
             "beam_control.scheduler.prefer_dense_tas_sampling");

  // association
  CHECK_EQ(a.policy.association.unassigned_cost, b.policy.association.unassigned_cost,
           "association.unassigned_cost");

  // tracking
  CHECK_BOOL(a.policy.tracking.enable_kalman_filter, b.policy.tracking.enable_kalman_filter,
             "tracking.enable_kalman_filter");
  CHECK_EQ(a.policy.tracking.kalman_measurement_noise_std,
           b.policy.tracking.kalman_measurement_noise_std, "tracking.kalman_measurement_noise_std");
  CHECK_INT(static_cast<int>(a.policy.tracking.kalman_update_backend),
            static_cast<int>(b.policy.tracking.kalman_update_backend),
            "tracking.kalman_update_backend");
  CHECK_EQ(a.policy.tracking.speed_decay_ratio_on_loss, b.policy.tracking.speed_decay_ratio_on_loss,
           "tracking.speed_decay_ratio_on_loss");
  CHECK_EQ(a.policy.tracking.rcs_decay_ratio_on_loss, b.policy.tracking.rcs_decay_ratio_on_loss,
           "tracking.rcs_decay_ratio_on_loss");

  // lifecycle
  CHECK_INT(a.policy.lifecycle.confirm_hits, b.policy.lifecycle.confirm_hits,
            "lifecycle.confirm_hits");
  CHECK_INT(a.policy.lifecycle.max_miss_before_lost, b.policy.lifecycle.max_miss_before_lost,
            "lifecycle.max_miss_before_lost");
  CHECK_INT(a.policy.lifecycle.max_lost_cycles, b.policy.lifecycle.max_lost_cycles,
            "lifecycle.max_lost_cycles");
  CHECK_BOOL(a.policy.lifecycle.enable_imm_lifecycle, b.policy.lifecycle.enable_imm_lifecycle,
             "lifecycle.enable_imm_lifecycle");
  CHECK_INT(a.policy.lifecycle.model_count_hint, b.policy.lifecycle.model_count_hint,
            "lifecycle.model_count_hint");

  // environment
  CHECK_BOOL(a.environment.scenario_config.atmospheric_physics.enable_physical_model,
             b.environment.scenario_config.atmospheric_physics.enable_physical_model,
             "atmos.enable_physical_model");
  CHECK_EQ(a.environment.scenario_config.atmospheric_physics.pressure_hpa,
           b.environment.scenario_config.atmospheric_physics.pressure_hpa, "atmos.pressure_hpa");
  CHECK_EQ(a.environment.scenario_config.atmospheric_physics.temperature_k,
           b.environment.scenario_config.atmospheric_physics.temperature_k, "atmos.temperature_k");
  CHECK_EQ(a.environment.scenario_config.atmospheric_physics.relative_humidity,
           b.environment.scenario_config.atmospheric_physics.relative_humidity,
           "atmos.relative_humidity");

  CHECK_BOOL(a.environment.scenario_config.atmospheric_context.has_simulation_unix_seconds,
             b.environment.scenario_config.atmospheric_context.has_simulation_unix_seconds,
             "atmos_ctx.has_simulation_unix_seconds");
  CHECK_INT(a.environment.scenario_config.atmospheric_context.simulation_unix_seconds,
            b.environment.scenario_config.atmospheric_context.simulation_unix_seconds,
            "atmos_ctx.simulation_unix_seconds");
  CHECK_EQ(a.environment.scenario_config.atmospheric_context.solar_flux_f107a,
           b.environment.scenario_config.atmospheric_context.solar_flux_f107a,
           "atmos_ctx.solar_flux_f107a");
  CHECK_EQ(a.environment.scenario_config.atmospheric_context.solar_flux_f107,
           b.environment.scenario_config.atmospheric_context.solar_flux_f107,
           "atmos_ctx.solar_flux_f107");
  CHECK_EQ(a.environment.scenario_config.atmospheric_context.geomagnetic_ap,
           b.environment.scenario_config.atmospheric_context.geomagnetic_ap,
           "atmos_ctx.geomagnetic_ap");

  CHECK_INT(
      static_cast<int>(a.environment.scenario_config.vegetation_scatter_physics.cover_profile),
      static_cast<int>(b.environment.scenario_config.vegetation_scatter_physics.cover_profile),
      "veg.cover_profile");
  CHECK_BOOL(a.environment.scenario_config.vegetation_scatter_physics.enable_physical_model,
             b.environment.scenario_config.vegetation_scatter_physics.enable_physical_model,
             "veg.enable_physical_model");

  // jamming_sensitivity_profile
  CHECK_INT(static_cast<int>(a.environment.jamming_sensitivity_profile),
            static_cast<int>(b.environment.jamming_sensitivity_profile),
            "jamming_sensitivity_profile");
}

}  // namespace

int main() {
  const ar_config::RadarSessionConfig builder_config = MakeWideAreaSearchConfig();

  ar_config::RadarSessionConfig file_config;
  {
    std::string error;
    if (!examples::LoadArSessionConfigFromFile("configs/airborne_radar.json", &file_config,
                                               &error)) {
      std::cerr << "FAIL: could not load JSON config: " << error << "\n";
      return 1;
    }
  }

  CompareConfigs(builder_config, file_config);

  if (failed == 0) {
    std::cout << "PASS: all fields match\n";
    return 0;
  }
  std::cerr << "FAILED: " << failed << " field(s) differ\n";
  return 1;
}
