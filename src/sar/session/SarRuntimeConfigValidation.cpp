#include "sar/session/SarRuntimeConfigValidation.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>

#include "sar/imaging/SarFocusingSelector.h"
#include "sar/session/SarDiagnosticUtils.h"

#include "1q/sar/session/SarInputValidation.h"

namespace sar {
namespace session {

namespace {

bool HasValidL3Waypoints(const config::SarMissionConfig& mission) {
  if (mission.l3_waypoints.size() < 2U) {
    return false;
  }
  for (std::size_t index = 0U; index < mission.l3_waypoints.size(); ++index) {
    const config::SarWaypointConfig& waypoint = mission.l3_waypoints[index];
    if (!std::isfinite(waypoint.time_from_session_start_s) ||
        !std::isfinite(waypoint.latitude_deg) || !std::isfinite(waypoint.longitude_deg) ||
        !std::isfinite(waypoint.altitude_m) || waypoint.time_from_session_start_s < 0.0 ||
        (index > 0U && waypoint.time_from_session_start_s <=
                           mission.l3_waypoints[index - 1U].time_from_session_start_s)) {
      return false;
    }
  }
  return mission.l3_waypoints.front().time_from_session_start_s == 0.0;
}

}  // namespace

bool ValidateRuntimeConfigForStep(const config::SarSessionConfig& config,
                                  bool has_external_raw_iq,
                                  SarCycleResult* result) {
  if (!session::AreSarHardwareAndMissionFieldsValid(config)) {
    RecordAbort(result, "invalid_config", "SAR runtime config contains invalid hardware/mission fields.");
    return false;
  }

  // 跨字段物理约束：LFM 脉冲的完整回波需要覆盖整个脉冲持续时间。
  if (config.hardware.pulse_width_s > 0.0) {
    const std::size_t waveform_samples = static_cast<std::size_t>(
        std::ceil(config.hardware.pulse_width_s * config.hardware.sample_rate_hz));
    if (waveform_samples > config.mission.range_sample_count) {
      RecordAbort(result, "sample_window_too_small_for_pulse",
                  "SAR range sample window (" + std::to_string(config.mission.range_sample_count) +
                      " samples) cannot hold the full LFM pulse (" +
                      std::to_string(waveform_samples) +
                      " samples = pulse_width_s * sample_rate_hz). Increase range_sample_count "
                      "or reduce pulse_width_s / sample_rate_hz.");
      return false;
    }
  }

  if (config.policy.enable_l1_rda_imaging &&
      imaging::ExceedsFocusingSizeLimit(config.mission.range_sample_count,
                                        config.mission.azimuth_pulse_count,
                                        imaging::kFocusingRdaSizeLimit)) {
    RecordAbort(result, "rda_size_gate",
                "SAR session RDA size exceeds current Phase 1 runtime gate; use smaller "
                "validation scenes until performance approval.");
    return false;
  }
  if (config.policy.enable_l1_rda_imaging &&
      !config.policy.enable_raw_echo_generation) {
    RecordAbort(result, "rda_requires_raw_echo",
                "SAR session RDA requires raw echo generation.");
    return false;
  }
  if (config.policy.enable_l2_motion_compensation &&
      (!config.policy.enable_l1_rda_imaging || !config.policy.enable_raw_echo_generation ||
       config.mission.l2_velocity_error_stddev_x_mps < 0.0 ||
       config.mission.l2_velocity_error_stddev_y_mps < 0.0 ||
       config.mission.l2_velocity_error_stddev_z_mps < 0.0)) {
    RecordAbort(result, "invalid_l2_motion_compensation_config",
                "SAR L2 motion compensation requires raw echo, RDA, and non-negative velocity "
                "errors.");
    return false;
  }
  if (config.policy.enable_l3_bp_imaging &&
      (!config.policy.enable_raw_echo_generation ||
       config.policy.enable_l1_rda_imaging || config.policy.enable_l2_motion_compensation ||
       (!has_external_raw_iq && !HasValidL3Waypoints(config.mission)))) {
    RecordAbort(result, "invalid_l3_bp_config",
                "SAR L3 BP requires raw echo, valid waypoints, and no L1/L2 path.");
    return false;
  }
  if (config.policy.enable_l3_bp_imaging &&
      imaging::ExceedsFocusingSizeLimit(config.mission.range_sample_count,
                                        config.mission.azimuth_pulse_count,
                                        imaging::kFocusingBackprojectionSizeLimit)) {
    RecordAbort(result, "l3_bp_size_gate",
                "SAR L3 BP size exceeds the approved 128x128 runtime gate.");
    return false;
  }
  return true;
}

}  // namespace session
}  // namespace sar
