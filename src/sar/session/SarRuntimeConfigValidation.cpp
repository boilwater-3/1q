#include "sar/session/SarRuntimeConfigValidation.h"

#include <cmath>
#include <cstdint>
#include <string>

namespace sar {
namespace session {

namespace {

constexpr std::uint32_t kMaxSessionRdaRangeSamples = 1024U;
constexpr std::uint32_t kMaxSessionRdaPulses = 1024U;
constexpr std::uint32_t kMaxSessionBpDimension = 128U;

void RecordAbort(SarCycleResult* result, const std::string& tag, const std::string& message) {
  result->has_error = true;
  result->abort_reason = tag;
  SarDiagnosticIssue issue;
  issue.severity = SarDiagnosticSeverity::kError;
  issue.code = "sar." + tag;
  issue.message = message;
  result->diagnostics.push_back(issue);
}

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
  if (config.hardware.bandwidth_hz <= 0.0 || config.hardware.sample_rate_hz <= 0.0 ||
      config.hardware.carrier_frequency_hz <= 0.0 ||
      config.hardware.pulse_repetition_frequency_hz <= 0.0 ||
      config.mission.platform_speed_mps <= 0.0 || config.mission.nominal_slant_range_m <= 0.0 ||
      config.mission.range_sample_count == 0U || config.mission.azimuth_pulse_count == 0U) {
    RecordAbort(result, "invalid_config", "SAR runtime config contains non-positive fields.");
    return false;
  }

  if (config.policy.enable_l1_rda_imaging &&
      (config.mission.range_sample_count > kMaxSessionRdaRangeSamples ||
       config.mission.azimuth_pulse_count > kMaxSessionRdaPulses)) {
    RecordAbort(result, "rda_size_gate",
                "SAR session RDA size exceeds current Phase 1 runtime gate; use smaller "
                "validation scenes until performance approval.");
    return false;
  }
  if (config.policy.enable_l1_rda_imaging && !config.policy.enable_raw_echo_generation) {
    RecordAbort(result, "rda_requires_raw_echo",
                "SAR session RDA requires raw echo generation in the current Phase 1 pipeline.");
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
      (!config.policy.enable_raw_echo_generation || !config.policy.enable_range_compression ||
       config.policy.enable_l1_rda_imaging || config.policy.enable_l2_motion_compensation ||
       (!has_external_raw_iq && !HasValidL3Waypoints(config.mission)))) {
    RecordAbort(result, "invalid_l3_bp_config",
                "SAR L3 BP requires raw echo, range compression, valid waypoints, and no L1/L2 "
                "path.");
    return false;
  }
  if (config.policy.enable_l3_bp_imaging &&
      (config.mission.range_sample_count > kMaxSessionBpDimension ||
       config.mission.azimuth_pulse_count > kMaxSessionBpDimension)) {
    RecordAbort(result, "l3_bp_size_gate",
                "SAR L3 BP size exceeds the approved 128x128 runtime gate.");
    return false;
  }
  return true;
}

}  // namespace session
}  // namespace sar

