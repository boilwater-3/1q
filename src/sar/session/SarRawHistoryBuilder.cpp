#include "sar/session/SarRawHistoryBuilder.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "sar/echo/SarEcho.h"
#include "sar/geometry/SarGeometry.h"
#include "sar/runtime/PulseRingBuffer.h"
#include "sar/session/SarDiagnosticUtils.h"
#include "sar/signal/SarWaveform.h"

namespace sar {
namespace session {

namespace {

constexpr double kEarthRadiusM = 6378137.0;

geometry::LocalPoint ToLocalPoint(double latitude_deg, double longitude_deg, double altitude_m,
                                  const config::SarMissionConfig& mission) {
  const double deg_to_rad = 3.141592653589793238462643383279502884 / 180.0;
  const double lat0_rad = mission.scene_center_latitude_deg * deg_to_rad;
  const double dlat_rad = (latitude_deg - mission.scene_center_latitude_deg) * deg_to_rad;
  const double dlon_rad = (longitude_deg - mission.scene_center_longitude_deg) * deg_to_rad;

  geometry::LocalPoint point;
  point.x_m = dlon_rad * std::cos(lat0_rad) * kEarthRadiusM;
  point.y_m = dlat_rad * kEarthRadiusM;
  point.z_m = altitude_m - mission.scene_center_altitude_m;
  return point;
}

double DbsmToSquareMeters(double dbsm) { return std::pow(10.0, dbsm / 10.0); }

bool CopyExternalPulseStates(const std::vector<SarRawIqFrame::PulseState>& public_states,
                             std::uint32_t expected_count,
                             std::deque<geometry::PlatformPulseState>* output) {
  if (output == nullptr || public_states.size() != expected_count) {
    return false;
  }
  std::deque<geometry::PlatformPulseState> converted;
  for (std::size_t index = 0U; index < public_states.size(); ++index) {
    const SarRawIqFrame::PulseState& public_state = public_states[index];
    if (!std::isfinite(public_state.time_s) || !std::isfinite(public_state.position_x_m) ||
        !std::isfinite(public_state.position_y_m) || !std::isfinite(public_state.position_z_m) ||
        !std::isfinite(public_state.velocity_x_mps) ||
        !std::isfinite(public_state.velocity_y_mps) ||
        !std::isfinite(public_state.velocity_z_mps) ||
        (index > 0U &&
         (public_state.pulse_id != public_states[index - 1U].pulse_id + 1U ||
          public_state.time_s <= public_states[index - 1U].time_s))) {
      return false;
    }
    geometry::PlatformPulseState internal_state;
    internal_state.pulse_id = public_state.pulse_id;
    internal_state.time_s = public_state.time_s;
    internal_state.position_m.x_m = public_state.position_x_m;
    internal_state.position_m.y_m = public_state.position_y_m;
    internal_state.position_m.z_m = public_state.position_z_m;
    internal_state.velocity_x_mps = public_state.velocity_x_mps;
    internal_state.velocity_y_mps = public_state.velocity_y_mps;
    internal_state.velocity_z_mps = public_state.velocity_z_mps;
    converted.push_back(internal_state);
  }
  *output = std::move(converted);
  return true;
}

std::vector<echo::PointTarget> BuildLocalTargets(const SarCycleInput& input,
                                                 const config::SarMissionConfig& mission) {
  std::vector<echo::PointTarget> targets;
  targets.reserve(input.point_targets.size());
  for (const SarPointTarget& target : input.point_targets) {
    echo::PointTarget local_target;
    local_target.position_m =
        ToLocalPoint(target.latitude_deg, target.longitude_deg, target.altitude_m, mission);
    local_target.rcs_m2 = DbsmToSquareMeters(target.radar_cross_section_dbsm);
    targets.push_back(local_target);
  }
  return targets;
}

// 生成本周期的理想与实际脉冲轨迹。L1 为基础匀速直线条带；
// L3 启用时用航路点折线覆盖实际轨迹（ideal 同步设为 actual）；
// L2 启用时在实际轨迹上叠加确定性高斯扰动，并衔接上一周期末端的位置误差。
// 返回 false 时已写入 result 错误诊断。
bool GenerateCycleTrajectory(const config::SarSessionConfig& config, const SarCycleInput& input,
                             std::uint64_t next_pulse_id, std::size_t pulse_count_to_generate,
                             const geometry::PlatformPulseState* previous_actual,
                             std::vector<geometry::PlatformPulseState>* ideal_pulses,
                             std::vector<geometry::PlatformPulseState>* actual_pulses,
                             SarCycleResult* result) {
  geometry::StraightStripmapTrackConfig track_config;
  track_config.start_position_m =
      ToLocalPoint(input.platform.latitude_deg, input.platform.longitude_deg,
                   input.platform.altitude_m, config.mission);
  track_config.start_position_m.x_m += static_cast<double>(next_pulse_id) *
                                       config.mission.platform_speed_mps /
                                       config.hardware.pulse_repetition_frequency_hz;
  track_config.velocity_x_mps = config.mission.platform_speed_mps;
  track_config.prf_hz = config.hardware.pulse_repetition_frequency_hz;
  track_config.first_pulse_id = next_pulse_id;
  track_config.pulse_count = static_cast<std::uint32_t>(pulse_count_to_generate);

  if (pulse_count_to_generate > 0U &&
      !geometry::GenerateStraightStripmapTrack(track_config, ideal_pulses)) {
    RecordAbort(result, "track_generation_failed", "SAR failed to generate L1 stripmap track.");
    return false;
  }
  *actual_pulses = *ideal_pulses;

  if (config.policy.enable_l3_bp_imaging && pulse_count_to_generate > 0U) {
    geometry::WaypointTrackConfig l3_config;
    l3_config.first_pulse_id = next_pulse_id;
    for (const config::SarWaypointConfig& waypoint : config.mission.l3_waypoints) {
      geometry::Waypoint local_waypoint;
      local_waypoint.time_s = waypoint.time_from_session_start_s;
      local_waypoint.position_m = ToLocalPoint(waypoint.latitude_deg, waypoint.longitude_deg,
                                               waypoint.altitude_m, config.mission);
      l3_config.waypoints.push_back(local_waypoint);
    }
    for (std::size_t index = 0U; index < pulse_count_to_generate; ++index) {
      l3_config.pulse_times_s.push_back(static_cast<double>(next_pulse_id + index) /
                                        config.hardware.pulse_repetition_frequency_hz);
    }
    if (!geometry::GenerateWaypointTrack(l3_config, actual_pulses)) {
      RecordAbort(result, "l3_waypoint_coverage",
                  "SAR L3 waypoints do not cover the required fixed-PRF pulse time range.");
      return false;
    }
    *ideal_pulses = *actual_pulses;
    result->diagnostics.push_back(
        MakeInfoDiagnostic(
            "sar.l3_trajectory",
            "SAR L3 waypoint trajectory generated=" + std::to_string(actual_pulses->size()) +
                ", first_time_s=" + std::to_string(actual_pulses->front().time_s) +
                ", last_time_s=" + std::to_string(actual_pulses->back().time_s)));
  } else if (config.policy.enable_l2_motion_compensation && !ideal_pulses->empty()) {
    geometry::PerturbedStripmapTrackConfig l2_config;
    l2_config.ideal = track_config;
    l2_config.velocity_error_stddev_x_mps = config.mission.l2_velocity_error_stddev_x_mps;
    l2_config.velocity_error_stddev_y_mps = config.mission.l2_velocity_error_stddev_y_mps;
    l2_config.velocity_error_stddev_z_mps = config.mission.l2_velocity_error_stddev_z_mps;
    l2_config.random_seed =
        config.mission.l2_random_seed + static_cast<std::uint32_t>(next_pulse_id);
    if (previous_actual != nullptr) {
      const double dt_s = 1.0 / config.hardware.pulse_repetition_frequency_hz;
      l2_config.initial_position_error_m.x_m =
          previous_actual->position_m.x_m + previous_actual->velocity_x_mps * dt_s -
          ideal_pulses->front().position_m.x_m;
      l2_config.initial_position_error_m.y_m =
          previous_actual->position_m.y_m + previous_actual->velocity_y_mps * dt_s -
          ideal_pulses->front().position_m.y_m;
      l2_config.initial_position_error_m.z_m =
          previous_actual->position_m.z_m + previous_actual->velocity_z_mps * dt_s -
          ideal_pulses->front().position_m.z_m;
    }
    geometry::TrajectoryErrorDiagnostics trajectory_diagnostics;
    if (!geometry::GeneratePerturbedStripmapTrack(l2_config, actual_pulses,
                                                  &trajectory_diagnostics)) {
      RecordAbort(result, "l2_track_generation_failed", "SAR failed to generate L2 trajectory.");
      return false;
    }
    result->diagnostics.push_back(MakeInfoDiagnostic(
        "sar.l2_trajectory", "SAR L2 trajectory max_position_error_m=" +
                                 std::to_string(trajectory_diagnostics.max_position_error_m) +
                                 ", rms_position_error_m=" +
                                 std::to_string(trajectory_diagnostics.rms_position_error_m)));
  }
  return true;
}

}  // namespace

bool HasExternalRawIq(const SarCycleInput& input) {
  return input.raw_iq.pulse_count != 0U || input.raw_iq.samples_per_pulse != 0U ||
         !input.raw_iq.i_values.empty() || !input.raw_iq.q_values.empty() ||
         !input.raw_iq.pulse_states.empty() || !input.raw_iq.ideal_pulse_states.empty();
}

bool BuildWaveformAndFilter(const config::SarSessionConfig& config, signal::LfmWaveform* waveform,
                            signal::ComplexVector* matched_filter) {
  signal::LfmWaveformConfig waveform_config;
  waveform_config.bandwidth_hz = config.hardware.bandwidth_hz;
  waveform_config.sample_rate_hz = config.hardware.sample_rate_hz;
  waveform_config.start_frequency_hz = 0.0;
  waveform_config.time_bandwidth_product =
      std::max(config.hardware.bandwidth_hz * config.hardware.pulse_width_s, 1.0);
  return signal::GenerateLfmWaveform(waveform_config, waveform) &&
         signal::BuildMatchedFilter(waveform->samples, matched_filter);
}

double EstimateRawHistorySnrDb(const signal::ComplexMatrix& history) {
  if (history.rows == 0U || history.cols == 0U ||
      history.values.size() != history.rows * history.cols) {
    return -std::numeric_limits<double>::infinity();
  }

  double peak_power = 0.0;
  double mean_power = 0.0;
  for (const signal::ComplexSample& sample : history.values) {
    const double power = std::norm(sample);
    peak_power = std::max(peak_power, power);
    mean_power += power;
  }
  mean_power /= static_cast<double>(history.values.size());
  if (peak_power <= 0.0 || mean_power <= 0.0 || !std::isfinite(peak_power) ||
      !std::isfinite(mean_power)) {
    return -std::numeric_limits<double>::infinity();
  }
  return 10.0 * std::log10(peak_power / mean_power);
}

bool BuildExternalRawIqHistory(const config::SarSessionConfig& config, const SarCycleInput& input,
                               signal::ComplexMatrix* history,
                               std::deque<geometry::PlatformPulseState>* ideal_trajectory_buffer,
                               std::deque<geometry::PlatformPulseState>* actual_trajectory_buffer,
                               SarCycleResult* result) {
  const std::size_t expected_value_count =
      static_cast<std::size_t>(config.mission.azimuth_pulse_count) *
      static_cast<std::size_t>(config.mission.range_sample_count);
  if (!config.policy.enable_l1_rda_imaging && !config.policy.enable_l3_bp_imaging) {
    RecordAbort(result, "external_raw_iq_requires_l1_rda",
                "External raw IQ requires L1 RDA or L3 BP.");
    return false;
  }
  if (input.raw_iq.pulse_count != config.mission.azimuth_pulse_count ||
      input.raw_iq.samples_per_pulse != config.mission.range_sample_count ||
      input.raw_iq.i_values.size() != expected_value_count ||
      input.raw_iq.q_values.size() != expected_value_count) {
    RecordAbort(result, "external_raw_iq_shape_mismatch",
                "External raw IQ shape must exactly match the configured complete aperture.");
    return false;
  }

  if ((config.policy.enable_l3_bp_imaging || config.policy.enable_l2_motion_compensation) &&
      input.raw_iq.pulse_states.size() != input.raw_iq.pulse_count) {
    RecordAbort(result, "external_raw_iq_trajectory_required",
                "External raw IQ BP/L2 requires one actual pulse state for every IQ row.");
    return false;
  }
  if (config.policy.enable_l2_motion_compensation &&
      input.raw_iq.ideal_pulse_states.size() != input.raw_iq.pulse_count) {
    RecordAbort(result, "external_raw_iq_ideal_trajectory_required",
                "External raw IQ L2 requires one ideal pulse state for every IQ row.");
    return false;
  }
  if (config.policy.enable_l1_rda_imaging && !config.policy.enable_l2_motion_compensation &&
      (!input.raw_iq.pulse_states.empty() || !input.raw_iq.ideal_pulse_states.empty())) {
    result->diagnostics.push_back(MakeInfoDiagnostic(
        "sar.external_raw_iq_trajectory_ignored",
        "External pulse states are ignored by L1 RDA when L2 motion compensation is disabled."));
  }

  signal::ComplexMatrix external_history;
  external_history.rows = input.raw_iq.pulse_count;
  external_history.cols = input.raw_iq.samples_per_pulse;
  external_history.values.reserve(expected_value_count);
  for (std::size_t index = 0U; index < expected_value_count; ++index) {
    const double i_value = input.raw_iq.i_values[index];
    const double q_value = input.raw_iq.q_values[index];
    if (!std::isfinite(i_value) || !std::isfinite(q_value)) {
      RecordAbort(result, "external_raw_iq_non_finite",
                  "External raw IQ contains a non-finite sample.");
      return false;
    }
    external_history.values.push_back(signal::ComplexSample(i_value, q_value));
  }
  if ((config.policy.enable_l3_bp_imaging || config.policy.enable_l2_motion_compensation) &&
      !CopyExternalPulseStates(input.raw_iq.pulse_states, input.raw_iq.pulse_count,
                               actual_trajectory_buffer)) {
    RecordAbort(result, "external_raw_iq_invalid_trajectory",
                "External actual pulse states must be finite, contiguous, and strictly time "
                "ordered.");
    return false;
  }
  if (config.policy.enable_l2_motion_compensation &&
      !CopyExternalPulseStates(input.raw_iq.ideal_pulse_states, input.raw_iq.pulse_count,
                               ideal_trajectory_buffer)) {
    actual_trajectory_buffer->clear();
    RecordAbort(result, "external_raw_iq_invalid_ideal_trajectory",
                "External ideal pulse states must be finite, contiguous, and strictly time "
                "ordered.");
    return false;
  }
  *history = std::move(external_history);
  result->diagnostics.push_back(
      MakeInfoDiagnostic("sar.external_raw_iq",
                         "SAR consumed external complete-aperture raw IQ pulses=" +
                             std::to_string(input.raw_iq.pulse_count) +
                             ", samples_per_pulse=" +
                             std::to_string(input.raw_iq.samples_per_pulse) +
                             (input.point_targets.empty() ? "." : "; point targets were ignored.")));
  return true;
}

bool BuildRawPulseHistory(const config::SarSessionConfig& config, const SarCycleInput& input,
                          const signal::ComplexVector& transmit_waveform,
                          runtime::PulseRingBuffer* pulse_buffer, std::uint64_t* next_pulse_id,
                          double* pulse_fraction_carry, signal::ComplexMatrix* history,
                          std::deque<geometry::PlatformPulseState>* ideal_trajectory_buffer,
                          std::deque<geometry::PlatformPulseState>* actual_trajectory_buffer,
                          SarCycleResult* result) {
  if (pulse_buffer == nullptr || next_pulse_id == nullptr || pulse_fraction_carry == nullptr ||
      ideal_trajectory_buffer == nullptr || actual_trajectory_buffer == nullptr) {
    RecordAbort(result, "pulse_buffer_unavailable", "SAR pulse ring buffer is unavailable.");
    return false;
  }

  const double requested_pulses =
      input.dt_sec * config.hardware.pulse_repetition_frequency_hz + *pulse_fraction_carry;
  std::size_t pulse_count_to_generate = static_cast<std::size_t>(std::floor(requested_pulses));
  *pulse_fraction_carry = requested_pulses - static_cast<double>(pulse_count_to_generate);
  if (pulse_buffer->size() < config.mission.azimuth_pulse_count) {
    pulse_count_to_generate = std::max(pulse_count_to_generate,
                                       config.mission.azimuth_pulse_count - pulse_buffer->size());
  }

  if (pulse_count_to_generate == 0U) {
    result->diagnostics.push_back(
        MakeInfoDiagnostic("sar.pulse_ring_buffer", "SAR pulse ring buffer reused latest aperture."));
  }

  std::vector<geometry::PlatformPulseState> ideal_pulses;
  std::vector<geometry::PlatformPulseState> actual_pulses;
  const geometry::PlatformPulseState* previous_actual =
      actual_trajectory_buffer->empty() ? nullptr : &actual_trajectory_buffer->back();
  if (!GenerateCycleTrajectory(config, input, *next_pulse_id, pulse_count_to_generate,
                               previous_actual, &ideal_pulses, &actual_pulses, result)) {
    return false;
  }

  const std::vector<echo::PointTarget> targets = BuildLocalTargets(input, config.mission);
  echo::RawEchoConfig echo_config;
  echo_config.sample_rate_hz = config.hardware.sample_rate_hz;
  echo_config.carrier_frequency_hz = config.hardware.carrier_frequency_hz;
  echo_config.range_sample_count = config.mission.range_sample_count;

  history->rows = actual_pulses.size();
  history->cols = config.mission.range_sample_count;
  history->values.assign(history->rows * history->cols, signal::ComplexSample(0.0, 0.0));

  std::size_t clipping_count = 0U;
  for (std::size_t row = 0U; row < actual_pulses.size(); ++row) {
    echo::RawEchoResult echo;
    if (!echo::GeneratePointTargetRawEcho(echo_config, actual_pulses[row], targets,
                                          transmit_waveform, &echo)) {
      RecordAbort(result, "raw_echo_failed", "SAR failed to generate point-target raw echo.");
      return false;
    }
    if (echo.has_clipping) {
      ++clipping_count;
    }
    runtime::PulseRecord record;
    record.pulse_id = *next_pulse_id;
    record.samples = echo.samples;
    if (!pulse_buffer->Push(record)) {
      RecordAbort(result, "pulse_buffer_push_failed",
                  "SAR failed to append raw pulse to ring buffer.");
      return false;
    }
    ++(*next_pulse_id);
    ideal_trajectory_buffer->push_back(ideal_pulses[row]);
    actual_trajectory_buffer->push_back(actual_pulses[row]);
    while (ideal_trajectory_buffer->size() > config.mission.azimuth_pulse_count) {
      ideal_trajectory_buffer->pop_front();
      actual_trajectory_buffer->pop_front();
    }
  }

  if (clipping_count > 0U) {
    result->diagnostics.push_back(MakeInfoDiagnostic(
        "sar.raw_echo_clipping",
        "SAR raw echo clipping observed in " + std::to_string(clipping_count) + " pulses."));
  }

  std::vector<runtime::PulseRecord> latest_pulses;
  if (!pulse_buffer->ReadLatest(config.mission.azimuth_pulse_count, &latest_pulses)) {
    RecordAbort(result, "pulse_history_unavailable",
                "SAR pulse ring buffer cannot provide a contiguous latest aperture.");
    return false;
  }

  history->rows = latest_pulses.size();
  history->cols = config.mission.range_sample_count;
  history->values.assign(history->rows * history->cols, signal::ComplexSample(0.0, 0.0));
  for (std::size_t row = 0U; row < latest_pulses.size(); ++row) {
    if (latest_pulses[row].samples.size() != history->cols) {
      RecordAbort(result, "pulse_sample_count_mismatch",
                  "SAR pulse ring buffer returned a pulse with unexpected range sample count.");
      return false;
    }
    for (std::size_t col = 0U; col < history->cols; ++col) {
      (*history)(row, col) = latest_pulses[row].samples[col];
    }
  }

  result->diagnostics.push_back(
      MakeInfoDiagnostic("sar.pulse_ring_buffer",
                         "SAR pulse ring buffer size=" + std::to_string(pulse_buffer->size()) +
                             ", generated=" + std::to_string(actual_pulses.size()) +
                             ", overflow=" +
                             (pulse_buffer->overflow_sticky() ? "true" : "false")));
  return true;
}

}  // namespace session
}  // namespace sar
