#include "1q/sar/session/SarSession.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "1q/sar/session/SarSessionFactory.h"
#include "sar/echo/SarEcho.h"
#include "sar/geometry/SarGeometry.h"
#include "sar/imaging/SarGbp.h"
#include "sar/imaging/SarImageQuality.h"
#include "sar/imaging/SarMotionCompensation.h"
#include "sar/imaging/SarRda.h"
#include "sar/runtime/PulseRingBuffer.h"
#include "sar/signal/SarWaveform.h"

namespace sar {
namespace session {

namespace {

constexpr double kEarthRadiusM = 6378137.0;
constexpr double kSpeedOfLightMps = 299792458.0;
constexpr std::uint32_t kMaxSessionRdaRangeSamples = 1024U;
constexpr std::uint32_t kMaxSessionRdaPulses = 1024U;
constexpr std::uint32_t kMaxSessionBpDimension = 128U;

bool HasRequestedUpdate(const config::SarRuntimeConfigPatch& patch) {
  return patch.has_enable_raw_echo_generation || patch.has_enable_range_compression ||
         patch.has_enable_l1_rda_imaging || patch.has_retain_raw_phase_history ||
         patch.has_retain_focused_image || patch.has_min_valid_snr_db;
}

void ApplyPatchToConfig(config::SarSessionConfig* config,
                        const config::SarRuntimeConfigPatch& patch) {
  if (patch.has_enable_raw_echo_generation) {
    config->policy.enable_raw_echo_generation = patch.enable_raw_echo_generation;
  }
  if (patch.has_enable_range_compression) {
    config->policy.enable_range_compression = patch.enable_range_compression;
  }
  if (patch.has_enable_l1_rda_imaging) {
    config->policy.enable_l1_rda_imaging = patch.enable_l1_rda_imaging;
  }
  if (patch.has_retain_raw_phase_history) {
    config->policy.retain_raw_phase_history = patch.retain_raw_phase_history;
  }
  if (patch.has_retain_focused_image) {
    config->policy.retain_focused_image = patch.retain_focused_image;
  }
  if (patch.has_min_valid_snr_db) {
    config->policy.min_valid_snr_db = patch.min_valid_snr_db;
  }
}

SarDiagnosticIssue MakeError(const char* code, const char* message) {
  SarDiagnosticIssue issue;
  issue.severity = SarDiagnosticSeverity::kError;
  issue.code = code;
  issue.message = message;
  return issue;
}

SarDiagnosticIssue MakeInfo(const char* code, const std::string& message) {
  SarDiagnosticIssue issue;
  issue.severity = SarDiagnosticSeverity::kInfo;
  issue.code = code;
  issue.message = message;
  return issue;
}

// 记录结构化中止错误：设置 has_error、以 tag 作为 abort_reason、追加 code 为
// "sar."+tag 的 Error 诊断。集中此三件套模式，确保 abort_reason 与 diagnostic code
// 始终一致，避免散落字符串字面量产生拼写漂移。
void RecordAbort(SarCycleResult* result, const std::string& tag, const std::string& message) {
  result->has_error = true;
  result->abort_reason = tag;
  SarDiagnosticIssue issue;
  issue.severity = SarDiagnosticSeverity::kError;
  issue.code = "sar." + tag;
  issue.message = message;
  result->diagnostics.push_back(std::move(issue));
}

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

bool ValidateRuntimeConfigForStep(const config::SarSessionConfig& config, bool has_external_raw_iq,
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

bool HasExternalRawIq(const SarCycleInput& input) {
  return input.raw_iq.pulse_count != 0U || input.raw_iq.samples_per_pulse != 0U ||
         !input.raw_iq.i_values.empty() || !input.raw_iq.q_values.empty() ||
         !input.raw_iq.pulse_states.empty() || !input.raw_iq.ideal_pulse_states.empty();
}

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
    result->diagnostics.push_back(MakeInfo(
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
      MakeInfo("sar.external_raw_iq",
               "SAR consumed external complete-aperture raw IQ pulses=" +
                   std::to_string(input.raw_iq.pulse_count) +
                   ", samples_per_pulse=" + std::to_string(input.raw_iq.samples_per_pulse) +
                   (input.point_targets.empty() ? "." : "; point targets were ignored.")));
  return true;
}

bool CopyFocusedImage(const signal::ComplexMatrix& source, SarFocusedImageSource image_source,
                      SarFocusedImage* output) {
  if (output == nullptr || source.rows == 0U || source.cols == 0U ||
      source.values.size() != source.rows * source.cols ||
      source.rows > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max()) ||
      source.cols > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())) {
    return false;
  }

  SarFocusedImage image;
  image.source = image_source;
  image.row_count = static_cast<std::uint32_t>(source.rows);
  image.column_count = static_cast<std::uint32_t>(source.cols);
  image.real_values.reserve(source.values.size());
  image.imaginary_values.reserve(source.values.size());
  for (const signal::ComplexSample& sample : source.values) {
    image.real_values.push_back(sample.real());
    image.imaginary_values.push_back(sample.imag());
  }
  *output = std::move(image);
  return true;
}

// 根据 policy 决定是否拷贝完整聚焦图像。retain=false 时仅写入占位元数据，
// 跳过大图拷贝；调用方仍可从 row_count/column_count/source 获知图像形状。
bool ExportFocusedImage(const config::SarPolicyConfig& policy,
                        const signal::ComplexMatrix& source, SarFocusedImageSource image_source,
                        SarFocusedImage* output) {
  if (output == nullptr) {
    return false;
  }
  if (!policy.retain_focused_image) {
    SarFocusedImage placeholder;
    placeholder.source = image_source;
    placeholder.row_count = static_cast<std::uint32_t>(source.rows);
    placeholder.column_count = static_cast<std::uint32_t>(source.cols);
    placeholder.is_placeholder = true;
    *output = std::move(placeholder);
    return true;
  }
  return CopyFocusedImage(source, image_source, output);
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
        MakeInfo("sar.l3_trajectory",
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
    result->diagnostics.push_back(MakeInfo(
        "sar.l2_trajectory", "SAR L2 trajectory max_position_error_m=" +
                                 std::to_string(trajectory_diagnostics.max_position_error_m) +
                                 ", rms_position_error_m=" +
                                 std::to_string(trajectory_diagnostics.rms_position_error_m)));
  }
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
        MakeInfo("sar.pulse_ring_buffer", "SAR pulse ring buffer reused latest aperture."));
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
    result->diagnostics.push_back(MakeInfo(
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
      MakeInfo("sar.pulse_ring_buffer",
               "SAR pulse ring buffer size=" + std::to_string(pulse_buffer->size()) +
                   ", generated=" + std::to_string(actual_pulses.size()) +
                   ", overflow=" + (pulse_buffer->overflow_sticky() ? "true" : "false")));
  return true;
}

}  // namespace

struct SarSession::Impl {
  explicit Impl(const config::SarSessionConfig& initial_config)
      : runtime_config(initial_config),
        raw_pulse_buffer(std::max<std::size_t>(initial_config.mission.azimuth_pulse_count, 1U)) {}

  config::SarSessionConfig runtime_config;
  runtime::PulseRingBuffer raw_pulse_buffer;
  std::deque<geometry::PlatformPulseState> ideal_trajectory_buffer;
  std::deque<geometry::PlatformPulseState> actual_trajectory_buffer;
  std::uint64_t next_pulse_id{0U};
  double pulse_fraction_carry{0.0};
  SarOutputFrame previous_output{};
  bool has_previous_output{false};
};

SarSession::SarSession(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

SarSession::SarSession() : impl_(new Impl(config::SarSessionConfig{})) {}

SarSession::~SarSession() noexcept = default;
SarSession::SarSession(SarSession&&) noexcept = default;
SarSession& SarSession::operator=(SarSession&&) noexcept = default;

SarSession SarSessionFactory::Create(const config::SarSessionConfig& config) {
  return SarSession(std::unique_ptr<SarSession::Impl>(new SarSession::Impl(config)));
}

SarOutputFrame SarSession::Step(const SarCycleInput& input) {
  return StepWithResult(input).output_frame;
}

SarCycleResult SarSession::StepWithResult(const SarCycleInput& input) {
  SarCycleResult result;
  result.input_cycle_index = input.cycle_index;
  result.output_frame.cycle_index = input.cycle_index;

  if (input.dt_sec <= 0.0) {
    RecordAbort(&result, "invalid_dt_sec", "SAR cycle dt_sec must be positive.");
    if (impl_->has_previous_output) {
      result.output_frame = impl_->previous_output;
      result.reused_previous_output = true;
    }
    return result;
  }

  result.output_frame.range_sample_count = impl_->runtime_config.mission.range_sample_count;
  result.output_frame.azimuth_pulse_count = impl_->runtime_config.mission.azimuth_pulse_count;
  result.output_frame.center_slant_range_m = impl_->runtime_config.mission.nominal_slant_range_m;
  result.output_frame.estimated_snr_db = 0.0;

  const bool has_external_raw_iq = HasExternalRawIq(input);
  if (!ValidateRuntimeConfigForStep(impl_->runtime_config, has_external_raw_iq, &result)) {
    if (impl_->has_previous_output) {
      result.output_frame = impl_->previous_output;
      result.reused_previous_output = true;
    }
    return result;
  }

  signal::LfmWaveform waveform;
  signal::ComplexVector matched_filter;
  if (!BuildWaveformAndFilter(impl_->runtime_config, &waveform, &matched_filter)) {
    RecordAbort(&result, "waveform_generation_failed", "SAR failed to generate LFM waveform.");
    return result;
  }

  signal::ComplexMatrix raw_history;
  if (impl_->runtime_config.policy.enable_raw_echo_generation) {
    if (has_external_raw_iq) {
      if (!BuildExternalRawIqHistory(impl_->runtime_config, input, &raw_history,
                                     &impl_->ideal_trajectory_buffer,
                                     &impl_->actual_trajectory_buffer, &result)) {
        return result;
      }
    } else {
      if (!BuildRawPulseHistory(
              impl_->runtime_config, input, waveform.samples, &impl_->raw_pulse_buffer,
              &impl_->next_pulse_id, &impl_->pulse_fraction_carry, &raw_history,
              &impl_->ideal_trajectory_buffer, &impl_->actual_trajectory_buffer, &result)) {
        return result;
      }
    }
    result.output_frame.completed_stage = SarProcessingStage::kRawEcho;
    result.output_frame.has_raw_echo = true;
  }
  if (impl_->runtime_config.policy.enable_range_compression) {
    result.output_frame.completed_stage = SarProcessingStage::kRangeCompression;
    result.output_frame.has_range_compressed_echo = true;
  }
  if (impl_->runtime_config.policy.enable_l1_rda_imaging) {
    signal::ComplexMatrix rda_input = raw_history;
    if (impl_->runtime_config.policy.enable_l2_motion_compensation) {
      if (impl_->ideal_trajectory_buffer.size() != raw_history.rows ||
          impl_->actual_trajectory_buffer.size() != raw_history.rows) {
        RecordAbort(&result, "l2_trajectory_history_mismatch",
                    "SAR L2 trajectory history does not match the latest raw aperture.");
        return result;
      }
      const std::vector<geometry::PlatformPulseState> ideal_trajectory(
          impl_->ideal_trajectory_buffer.begin(), impl_->ideal_trajectory_buffer.end());
      const std::vector<geometry::PlatformPulseState> actual_trajectory(
          impl_->actual_trajectory_buffer.begin(), impl_->actual_trajectory_buffer.end());
      imaging::FirstOrderMotionCompensationConfig compensation_config;
      compensation_config.sample_rate_hz = impl_->runtime_config.hardware.sample_rate_hz;
      compensation_config.carrier_frequency_hz =
          impl_->runtime_config.hardware.carrier_frequency_hz;
      compensation_config.reference_point_m.y_m =
          impl_->runtime_config.mission.nominal_slant_range_m;
      imaging::MotionCompensationDiagnostics compensation_diagnostics;
      if (!imaging::ApplyFirstOrderMotionCompensation(compensation_config, ideal_trajectory,
                                                      actual_trajectory, raw_history, &rda_input,
                                                      &compensation_diagnostics)) {
        RecordAbort(&result, "motion_compensation_failed", "SAR L2 motion compensation failed.");
        return result;
      }
      result.diagnostics.push_back(MakeInfo(
          "sar.motion_compensation",
          "SAR first-order motion compensation max_abs_range_error_m=" +
              std::to_string(compensation_diagnostics.max_abs_range_error_m) +
              ", rms_range_error_m=" + std::to_string(compensation_diagnostics.rms_range_error_m) +
              ", max_abs_envelope_shift_bins=" +
              std::to_string(compensation_diagnostics.max_abs_envelope_shift_bins)));
    }
    imaging::RdaConfig rda_config;
    rda_config.sample_rate_hz = impl_->runtime_config.hardware.sample_rate_hz;
    rda_config.carrier_frequency_hz = impl_->runtime_config.hardware.carrier_frequency_hz;
    rda_config.prf_hz = impl_->runtime_config.hardware.pulse_repetition_frequency_hz;
    rda_config.platform_velocity_mps = impl_->runtime_config.mission.platform_speed_mps;
    rda_config.reference_range_m = impl_->runtime_config.mission.nominal_slant_range_m;
    rda_config.rcmc_interpolation = imaging::RcmcInterpolation::kLinear;

    imaging::FocusedSarImage image;
    if (!imaging::FocusStripmapRda(rda_config, rda_input, matched_filter, &image)) {
      RecordAbort(&result, "rda_failed", "SAR RDA focus failed.");
      return result;
    }
    if (!ExportFocusedImage(impl_->runtime_config.policy, image.image,
                            SarFocusedImageSource::kL1Rda, &result.focused_image)) {
      RecordAbort(&result, "rda_public_image_export_failed",
                  "SAR RDA image could not be converted to the public focused-image payload.");
      return result;
    }
    const std::size_t peak_index = imaging::FindPeakIndex(image.image);
    result.output_frame.phase_reference_mode = SarPhaseReferenceMode::kCenterBroadside;
    result.output_frame.image_quality_mainlobe_method = SarMainlobeEstimationMethod::k3dB;
    result.output_frame.range_width_3db_bins = image.diagnostics.range_width_3db_bins;
    result.output_frame.azimuth_width_3db_bins = image.diagnostics.azimuth_width_3db_bins;
    result.output_frame.range_resolution_3db_m = image.diagnostics.range_resolution_3db_m;
    result.output_frame.azimuth_resolution_3db_m = image.diagnostics.azimuth_resolution_3db_m;
    result.output_frame.image_entropy_nats = image.diagnostics.image_entropy_nats;
    result.output_frame.image_contrast = image.diagnostics.image_contrast;
    result.output_frame.has_image_quality_metrics = true;
    result.output_frame.image_resolution_m_valid = image.diagnostics.resolution_m_valid;
    result.output_frame.phase_reference_applied = image.diagnostics.phase_reference_applied;
    result.diagnostics.push_back(MakeInfo(
        "sar.rda_peak",
        "SAR RDA peak index " + std::to_string(peak_index) +
            ", doppler_rate_hz_per_s=" + std::to_string(image.diagnostics.doppler_rate_hz_per_s) +
            ", azimuth_sample_spacing_m=" +
            std::to_string(image.diagnostics.azimuth_sample_spacing_m) +
            ", azimuth_phase_curvature_rad_per_pulse2=" +
            std::to_string(image.diagnostics.azimuth_phase_curvature_rad_per_pulse2) +
            ", azimuth_quadratic_phase_span_rad=" +
            std::to_string(image.diagnostics.azimuth_quadratic_phase_span_rad) +
            ", max_geometric_doppler_hz=" +
            std::to_string(image.diagnostics.max_geometric_doppler_hz) +
            ", doppler_nyquist_margin=" +
            std::to_string(image.diagnostics.doppler_nyquist_margin) +
            ", phase_reference_mode=" + image.diagnostics.phase_reference_mode +
            ", phase_reference_applied=" +
            std::to_string(image.diagnostics.phase_reference_applied ? 1 : 0) +
            ", range_width_3db_bins=" + std::to_string(image.diagnostics.range_width_3db_bins) +
            ", azimuth_width_3db_bins=" + std::to_string(image.diagnostics.azimuth_width_3db_bins) +
            ", range_resolution_3db_m=" +
            std::to_string(image.diagnostics.range_resolution_3db_m) +
            ", azimuth_resolution_3db_m=" +
            std::to_string(image.diagnostics.azimuth_resolution_3db_m) +
            ", image_entropy_nats=" + std::to_string(image.diagnostics.image_entropy_nats) +
            ", image_contrast=" + std::to_string(image.diagnostics.image_contrast)));
    result.output_frame.completed_stage = SarProcessingStage::kL1RdaImage;
    result.output_frame.has_l1_image = true;
  }
  if (impl_->runtime_config.policy.enable_l3_bp_imaging) {
    if (impl_->actual_trajectory_buffer.size() != raw_history.rows) {
      RecordAbort(&result, "l3_trajectory_history_mismatch",
                  "SAR L3 trajectory history does not match the latest raw aperture.");
      return result;
    }
    const std::vector<geometry::PlatformPulseState> actual_trajectory(
        impl_->actual_trajectory_buffer.begin(), impl_->actual_trajectory_buffer.end());
    imaging::GbpConfig bp_config;
    bp_config.sample_rate_hz = impl_->runtime_config.hardware.sample_rate_hz;
    bp_config.carrier_frequency_hz = impl_->runtime_config.hardware.carrier_frequency_hz;
    bp_config.grid.azimuth_pixel_count = impl_->runtime_config.mission.azimuth_pulse_count;
    bp_config.grid.range_pixel_count = impl_->runtime_config.mission.range_sample_count;
    bp_config.grid.azimuth_spacing_m = impl_->runtime_config.mission.platform_speed_mps /
                                       impl_->runtime_config.hardware.pulse_repetition_frequency_hz;
    bp_config.grid.range_spacing_m =
        kSpeedOfLightMps / (2.0 * impl_->runtime_config.hardware.sample_rate_hz);
    bp_config.grid.azimuth_start_m = -0.5 *
                                     static_cast<double>(bp_config.grid.azimuth_pixel_count - 1U) *
                                     bp_config.grid.azimuth_spacing_m;
    imaging::FocusedGbpImage image;
    if (!imaging::FocusSmallSceneBp(bp_config, actual_trajectory, raw_history, matched_filter,
                                    &image)) {
      RecordAbort(&result, "l3_bp_failed", "SAR L3 BP focus failed.");
      return result;
    }
    if (!ExportFocusedImage(impl_->runtime_config.policy, image.image,
                            SarFocusedImageSource::kL3Bp, &result.focused_image)) {
      RecordAbort(&result, "bp_public_image_export_failed",
                  "SAR BP image could not be converted to the public focused-image payload.");
      return result;
    }
    const imaging::ImageQualityMetrics quality = imaging::EvaluateImageQuality(image.image);
    result.output_frame.phase_reference_mode = SarPhaseReferenceMode::kNative;
    result.output_frame.image_quality_mainlobe_method = SarMainlobeEstimationMethod::k3dB;
    result.output_frame.range_width_3db_bins = quality.range_width_3db_bins;
    result.output_frame.azimuth_width_3db_bins = quality.azimuth_width_3db_bins;
    result.output_frame.image_entropy_nats = quality.entropy_nats;
    result.output_frame.image_contrast = quality.image_contrast;
    result.output_frame.has_image_quality_metrics = quality.valid;
    result.output_frame.image_resolution_m_valid = false;
    result.output_frame.phase_reference_applied = false;
    result.diagnostics.push_back(MakeInfo(
        "sar.bp_peak", "SAR BP peak_row=" + std::to_string(quality.peak_row) +
                           ", peak_col=" + std::to_string(quality.peak_col) +
                           ", image_entropy_nats=" + std::to_string(quality.entropy_nats)));
    result.diagnostics.push_back(
        MakeInfo("sar.bp_traversal", "SAR BP traversal=" + image.diagnostics.traversal_order));
    result.output_frame.completed_stage = SarProcessingStage::kL3BpImage;
    result.output_frame.has_l3_bp_image = true;
  }

  result.executed_this_cycle = true;
  impl_->previous_output = result.output_frame;
  impl_->has_previous_output = true;
  return result;
}

void SarSession::ApplyRuntimeConfig(const config::SarRuntimeConfigPatch& patch) {
  (void)TryApplyRuntimeConfig(patch);
}

bool SarSession::TryApplyRuntimeConfig(const config::SarRuntimeConfigPatch& patch) {
  if (!HasRequestedUpdate(patch)) {
    return false;
  }
  ApplyPatchToConfig(&impl_->runtime_config, patch);
  return true;
}

}  // namespace session
}  // namespace sar
