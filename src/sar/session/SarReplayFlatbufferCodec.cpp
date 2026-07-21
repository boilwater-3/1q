#include "SarReplayFlatbufferCodec.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "common/replay/ReplayFlatbufferCodecSupport.h"
#include "flatbuffers/flatbuffers.h"
#include "sar/session/generated/sar_replay_generated.h"
#include "sar/session/generated/sar_session_replay_generated.h"

namespace sar {
namespace session {
namespace {

flatbuffers::Offset<replay::SarPlatformState> BuildPlatformState(
    flatbuffers::FlatBufferBuilder& fbb, const SarPlatformState& value) {
  return replay::CreateSarPlatformState(fbb, value.time_s, value.latitude_deg, value.longitude_deg,
                                        value.altitude_m, value.velocity_north_mps,
                                        value.velocity_east_mps, value.velocity_down_mps,
                                        value.roll_deg, value.pitch_deg, value.yaw_deg);
}

SarPlatformState FromFbPlatformState(const replay::SarPlatformState* fb) {
  SarPlatformState out;
  if (!fb) {
    return out;
  }
  out.time_s = fb->time_s();
  out.latitude_deg = fb->latitude_deg();
  out.longitude_deg = fb->longitude_deg();
  out.altitude_m = fb->altitude_m();
  out.velocity_north_mps = fb->velocity_north_mps();
  out.velocity_east_mps = fb->velocity_east_mps();
  out.velocity_down_mps = fb->velocity_down_mps();
  out.roll_deg = fb->roll_deg();
  out.pitch_deg = fb->pitch_deg();
  out.yaw_deg = fb->yaw_deg();
  return out;
}

flatbuffers::Offset<replay::SarRawIqPulseState> BuildRawIqPulseState(
    flatbuffers::FlatBufferBuilder& fbb, const SarRawIqFrame::PulseState& value) {
  return replay::CreateSarRawIqPulseState(
      fbb, value.pulse_id, value.time_s, value.position_x_m, value.position_y_m,
      value.position_z_m, value.velocity_x_mps, value.velocity_y_mps, value.velocity_z_mps);
}

SarRawIqFrame::PulseState FromFbRawIqPulseState(const replay::SarRawIqPulseState* fb) {
  SarRawIqFrame::PulseState out;
  if (!fb) {
    return out;
  }
  out.pulse_id = fb->pulse_id();
  out.time_s = fb->time_s();
  out.position_x_m = fb->position_x_m();
  out.position_y_m = fb->position_y_m();
  out.position_z_m = fb->position_z_m();
  out.velocity_x_mps = fb->velocity_x_mps();
  out.velocity_y_mps = fb->velocity_y_mps();
  out.velocity_z_mps = fb->velocity_z_mps();
  return out;
}

flatbuffers::Offset<replay::SarRawIqFrame> BuildRawIqFrame(
    flatbuffers::FlatBufferBuilder& fbb, const SarRawIqFrame& value) {
  std::vector<flatbuffers::Offset<replay::SarRawIqPulseState>> pulse_states;
  pulse_states.reserve(value.pulse_states.size());
  for (const SarRawIqFrame::PulseState& pulse : value.pulse_states) {
    pulse_states.push_back(BuildRawIqPulseState(fbb, pulse));
  }
  std::vector<flatbuffers::Offset<replay::SarRawIqPulseState>> ideal_pulse_states;
  ideal_pulse_states.reserve(value.ideal_pulse_states.size());
  for (const SarRawIqFrame::PulseState& pulse : value.ideal_pulse_states) {
    ideal_pulse_states.push_back(BuildRawIqPulseState(fbb, pulse));
  }
  return replay::CreateSarRawIqFrame(
      fbb, value.pulse_count, value.samples_per_pulse, fbb.CreateVector(value.i_values),
      fbb.CreateVector(value.q_values), fbb.CreateVector(pulse_states),
      fbb.CreateVector(ideal_pulse_states));
}

bool FromFbRawIqFrame(const replay::SarRawIqFrame* fb, SarRawIqFrame* out) {
  if (!fb || !out || !fb->i_values() || !fb->q_values() || !fb->pulse_states() ||
      !fb->ideal_pulse_states()) {
    return false;
  }
  SarRawIqFrame decoded;
  decoded.pulse_count = fb->pulse_count();
  decoded.samples_per_pulse = fb->samples_per_pulse();
  decoded.i_values.assign(fb->i_values()->begin(), fb->i_values()->end());
  decoded.q_values.assign(fb->q_values()->begin(), fb->q_values()->end());
  decoded.pulse_states.reserve(fb->pulse_states()->size());
  for (const replay::SarRawIqPulseState* pulse : *fb->pulse_states()) {
    if (!pulse) {
      return false;
    }
    decoded.pulse_states.push_back(FromFbRawIqPulseState(pulse));
  }
  decoded.ideal_pulse_states.reserve(fb->ideal_pulse_states()->size());
  for (const replay::SarRawIqPulseState* pulse : *fb->ideal_pulse_states()) {
    if (!pulse) {
      return false;
    }
    decoded.ideal_pulse_states.push_back(FromFbRawIqPulseState(pulse));
  }
  *out = std::move(decoded);
  return true;
}

flatbuffers::Offset<replay::SarOutputFrame> BuildOutputFrame(flatbuffers::FlatBufferBuilder& fbb,
                                                             const SarOutputFrame& value) {
  return replay::CreateSarOutputFrame(
      fbb, value.cycle_index, static_cast<std::int32_t>(value.completed_stage),
      value.range_sample_count, value.azimuth_pulse_count, value.center_slant_range_m,
      value.estimated_snr_db, static_cast<std::int32_t>(value.phase_reference_mode),
      static_cast<std::int32_t>(value.image_quality_mainlobe_method), value.range_width_3db_bins,
      value.azimuth_width_3db_bins, value.range_resolution_3db_m, value.azimuth_resolution_3db_m,
      value.image_entropy_nats, value.image_contrast, value.has_raw_echo,
      value.has_range_compressed_echo, value.has_l1_image, value.has_l3_bp_image,
      value.has_image_quality_metrics, value.image_resolution_m_valid,
      value.phase_reference_applied);
}

void FromFbOutputFrame(const replay::SarOutputFrame* fb, SarOutputFrame* out) {
  if (!fb || !out) {
    return;
  }
  out->cycle_index = fb->cycle_index();
  out->completed_stage = static_cast<SarProcessingStage>(fb->completed_stage());
  out->range_sample_count = fb->range_sample_count();
  out->azimuth_pulse_count = fb->azimuth_pulse_count();
  out->center_slant_range_m = fb->center_slant_range_m();
  out->estimated_snr_db = fb->estimated_snr_db();
  out->phase_reference_mode = static_cast<SarPhaseReferenceMode>(fb->phase_reference_mode());
  out->image_quality_mainlobe_method =
      static_cast<SarMainlobeEstimationMethod>(fb->image_quality_mainlobe_method());
  out->range_width_3db_bins = fb->range_width_3db_bins();
  out->azimuth_width_3db_bins = fb->azimuth_width_3db_bins();
  out->range_resolution_3db_m = fb->range_resolution_3db_m();
  out->azimuth_resolution_3db_m = fb->azimuth_resolution_3db_m();
  out->image_entropy_nats = fb->image_entropy_nats();
  out->image_contrast = fb->image_contrast();
  out->has_raw_echo = fb->has_raw_echo();
  out->has_range_compressed_echo = fb->has_range_compressed_echo();
  out->has_l1_image = fb->has_l1_image();
  out->has_l3_bp_image = fb->has_l3_bp_image();
  out->has_image_quality_metrics = fb->has_image_quality_metrics();
  out->image_resolution_m_valid = fb->image_resolution_m_valid();
  out->phase_reference_applied = fb->phase_reference_applied();
}

flatbuffers::Offset<replay::SarHardwareConfig> BuildHardwareConfig(
    flatbuffers::FlatBufferBuilder& fbb, const config::SarHardwareConfig& value) {
  return replay::CreateSarHardwareConfig(
      fbb, value.carrier_frequency_hz, value.bandwidth_hz, value.pulse_width_s,
      value.pulse_repetition_frequency_hz, value.sample_rate_hz, value.peak_power_w,
      value.antenna_length_m, value.antenna_width_m, value.antenna_gain_db,
      value.receiver_noise_figure_db, value.system_loss_db);
}

void FromFbHardwareConfig(const replay::SarHardwareConfig* fb, config::SarHardwareConfig* out) {
  if (!fb || !out) {
    return;
  }
  out->carrier_frequency_hz = fb->carrier_frequency_hz();
  out->bandwidth_hz = fb->bandwidth_hz();
  out->pulse_width_s = fb->pulse_width_s();
  out->pulse_repetition_frequency_hz = fb->pulse_repetition_frequency_hz();
  out->sample_rate_hz = fb->sample_rate_hz();
  out->peak_power_w = fb->peak_power_w();
  out->antenna_length_m = fb->antenna_length_m();
  out->antenna_width_m = fb->antenna_width_m();
  out->antenna_gain_db = fb->antenna_gain_db();
  out->receiver_noise_figure_db = fb->receiver_noise_figure_db();
  out->system_loss_db = fb->system_loss_db();
}

flatbuffers::Offset<replay::SarMissionConfig> BuildMissionConfig(
    flatbuffers::FlatBufferBuilder& fbb, const config::SarMissionConfig& value) {
  std::vector<flatbuffers::Offset<replay::SarWaypointConfig>> waypoints;
  waypoints.reserve(value.l3_waypoints.size());
  for (const config::SarWaypointConfig& waypoint : value.l3_waypoints) {
    waypoints.push_back(replay::CreateSarWaypointConfig(
        fbb, waypoint.time_from_session_start_s, waypoint.latitude_deg, waypoint.longitude_deg,
        waypoint.altitude_m));
  }
  const auto waypoint_vector = fbb.CreateVector(waypoints);
  return replay::CreateSarMissionConfig(
      fbb, value.scene_center_latitude_deg, value.scene_center_longitude_deg,
      value.scene_center_altitude_m, value.nominal_slant_range_m, value.synthetic_aperture_time_s,
      value.platform_speed_mps, value.range_sample_count, value.azimuth_pulse_count,
      value.desired_ground_range_resolution_m, value.desired_azimuth_resolution_m,
      value.l2_velocity_error_stddev_x_mps, value.l2_velocity_error_stddev_y_mps,
      value.l2_velocity_error_stddev_z_mps, value.l2_random_seed, waypoint_vector);
}

void FromFbMissionConfig(const replay::SarMissionConfig* fb, config::SarMissionConfig* out) {
  if (!fb || !out) {
    return;
  }
  out->scene_center_latitude_deg = fb->scene_center_latitude_deg();
  out->scene_center_longitude_deg = fb->scene_center_longitude_deg();
  out->scene_center_altitude_m = fb->scene_center_altitude_m();
  out->nominal_slant_range_m = fb->nominal_slant_range_m();
  out->synthetic_aperture_time_s = fb->synthetic_aperture_time_s();
  out->platform_speed_mps = fb->platform_speed_mps();
  out->range_sample_count = fb->range_sample_count();
  out->azimuth_pulse_count = fb->azimuth_pulse_count();
  out->desired_ground_range_resolution_m = fb->desired_ground_range_resolution_m();
  out->desired_azimuth_resolution_m = fb->desired_azimuth_resolution_m();
  out->l2_velocity_error_stddev_x_mps = fb->l2_velocity_error_stddev_x_mps();
  out->l2_velocity_error_stddev_y_mps = fb->l2_velocity_error_stddev_y_mps();
  out->l2_velocity_error_stddev_z_mps = fb->l2_velocity_error_stddev_z_mps();
  out->l2_random_seed = fb->l2_random_seed();
  out->l3_waypoints.clear();
  if (fb->l3_waypoints()) {
    out->l3_waypoints.reserve(fb->l3_waypoints()->size());
    for (const replay::SarWaypointConfig* waypoint : *fb->l3_waypoints()) {
      config::SarWaypointConfig decoded;
      decoded.time_from_session_start_s = waypoint->time_from_session_start_s();
      decoded.latitude_deg = waypoint->latitude_deg();
      decoded.longitude_deg = waypoint->longitude_deg();
      decoded.altitude_m = waypoint->altitude_m();
      out->l3_waypoints.push_back(decoded);
    }
  }
}

flatbuffers::Offset<replay::SarPolicyConfig> BuildPolicyConfig(
    flatbuffers::FlatBufferBuilder& fbb, const config::SarPolicyConfig& value) {
  return replay::CreateSarPolicyConfig(
      fbb, value.enable_raw_echo_generation, value.enable_range_compression,
      value.enable_l1_rda_imaging, value.enable_diagnostics, value.retain_raw_phase_history,
      value.retain_focused_image, value.max_allowed_squint_angle_deg, value.minimum_snr_db,
      value.enable_l2_motion_compensation, value.enable_l3_bp_imaging);
}

void FromFbPolicyConfig(const replay::SarPolicyConfig* fb, config::SarPolicyConfig* out) {
  if (!fb || !out) {
    return;
  }
  out->enable_raw_echo_generation = fb->enable_raw_echo_generation();
  out->enable_range_compression = fb->enable_range_compression();
  out->enable_l1_rda_imaging = fb->enable_l1_rda_imaging();
  out->enable_diagnostics = fb->enable_diagnostics();
  out->retain_raw_phase_history = fb->retain_raw_phase_history();
  out->retain_focused_image = fb->retain_focused_image();
  out->max_allowed_squint_angle_deg = fb->max_allowed_squint_angle_deg();
  out->minimum_snr_db = fb->minimum_snr_db();
  out->enable_l2_motion_compensation = fb->enable_l2_motion_compensation();
  out->enable_l3_bp_imaging = fb->enable_l3_bp_imaging();
}

flatbuffers::Offset<replay::SarEnvironmentConfig> BuildEnvironmentConfig(
    flatbuffers::FlatBufferBuilder& fbb, const config::SarEnvironmentConfig& value) {
  return replay::CreateSarEnvironmentConfig(
      fbb, value.terrain_reference_altitude_m, value.atmospheric_loss_db_per_km,
      value.surface_backscatter_sigma0_db, value.use_flat_earth_geometry,
      value.enable_atmospheric_attenuation);
}

void FromFbEnvironmentConfig(const replay::SarEnvironmentConfig* fb,
                             config::SarEnvironmentConfig* out) {
  if (!fb || !out) {
    return;
  }
  out->terrain_reference_altitude_m = fb->terrain_reference_altitude_m();
  out->atmospheric_loss_db_per_km = fb->atmospheric_loss_db_per_km();
  out->surface_backscatter_sigma0_db = fb->surface_backscatter_sigma0_db();
  out->use_flat_earth_geometry = fb->use_flat_earth_geometry();
  out->enable_atmospheric_attenuation = fb->enable_atmospheric_attenuation();
}

}  // namespace

std::string EncodeSarCycleInput(const SarCycleInput& value) {
  flatbuffers::FlatBufferBuilder fbb(512);
  std::vector<flatbuffers::Offset<replay::SarPointTarget>> target_offsets;
  target_offsets.reserve(value.point_targets.size());
  for (const SarPointTarget& target : value.point_targets) {
    target_offsets.push_back(replay::CreateSarPointTarget(
        fbb, target.latitude_deg, target.longitude_deg, target.altitude_m,
        target.radar_cross_section_dbsm, target.target_id, fbb.CreateString(target.target_name)));
  }
  const auto platform = BuildPlatformState(fbb, value.platform);
  const auto targets = fbb.CreateVector(target_offsets);
  const auto raw_iq = BuildRawIqFrame(fbb, value.raw_iq);
  fbb.Finish(
      replay::CreateSarCycleInput(fbb, value.cycle_index, value.dt_sec, platform, targets, raw_iq));
  return oneq::common::replay::CopyFinishedFlatbuffer(fbb);
}

bool DecodeSarCycleInput(const std::string& bytes, SarCycleInput* out) {
  if (!out) {
    return false;
  }
  flatbuffers::Verifier verifier(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size());
  if (!verifier.VerifyBuffer<replay::SarCycleInput>()) {
    return false;
  }
  const auto* fb = flatbuffers::GetRoot<replay::SarCycleInput>(bytes.data());
  SarCycleInput decoded;
  decoded.cycle_index = fb->cycle_index();
  decoded.dt_sec = fb->dt_sec();
  decoded.platform = FromFbPlatformState(fb->platform());
  if (fb->point_targets()) {
    decoded.point_targets.reserve(fb->point_targets()->size());
    for (const auto* target : *fb->point_targets()) {
      if (!target) {
        return false;
      }
      SarPointTarget decoded_target;
      decoded_target.target_id = target->target_id();
      decoded_target.target_name =
          target->target_name() ? target->target_name()->str() : std::string();
      decoded_target.latitude_deg = target->latitude_deg();
      decoded_target.longitude_deg = target->longitude_deg();
      decoded_target.altitude_m = target->altitude_m();
      decoded_target.radar_cross_section_dbsm = target->radar_cross_section_dbsm();
      decoded.point_targets.push_back(decoded_target);
    }
  }
  if (!FromFbRawIqFrame(fb->raw_iq(), &decoded.raw_iq)) {
    return false;
  }
  *out = std::move(decoded);
  return true;
}

std::string EncodeSarCycleResult(const SarCycleResult& value) {
  flatbuffers::FlatBufferBuilder fbb(512);
  const auto frame = BuildOutputFrame(fbb, value.output_frame);
  const auto focused_image = replay::CreateSarFocusedImage(
      fbb, static_cast<std::int32_t>(value.focused_image.source), value.focused_image.row_count,
      value.focused_image.column_count, fbb.CreateVector(value.focused_image.real_values),
      fbb.CreateVector(value.focused_image.imaginary_values), value.focused_image.is_placeholder);
  std::vector<flatbuffers::Offset<replay::SarDiagnosticIssue>> diagnostic_offsets;
  diagnostic_offsets.reserve(value.diagnostics.size());
  for (const SarDiagnosticIssue& diagnostic : value.diagnostics) {
    diagnostic_offsets.push_back(replay::CreateSarDiagnosticIssue(
        fbb, static_cast<std::int32_t>(diagnostic.severity), fbb.CreateString(diagnostic.code),
        fbb.CreateString(diagnostic.message)));
  }
  const auto raw_phase_history = replay::CreateSarRawPhaseHistory(
      fbb, static_cast<std::int32_t>(value.raw_phase_history.source),
      value.raw_phase_history.pulse_count, value.raw_phase_history.samples_per_pulse,
      fbb.CreateVector(value.raw_phase_history.i_values),
      fbb.CreateVector(value.raw_phase_history.q_values));
  fbb.Finish(replay::CreateSarCycleResult(fbb, value.input_cycle_index, frame, focused_image,
                                          fbb.CreateVector(diagnostic_offsets), raw_phase_history,
                                          value.has_error,
                                          value.executed_this_cycle, value.reused_previous_output,
                                          fbb.CreateString(value.abort_reason)));
  return oneq::common::replay::CopyFinishedFlatbuffer(fbb);
}

bool DecodeSarCycleResult(const std::string& bytes, SarCycleResult* out) {
  if (!out) {
    return false;
  }
  flatbuffers::Verifier verifier(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size());
  if (!verifier.VerifyBuffer<replay::SarCycleResult>()) {
    return false;
  }
  const auto* fb = flatbuffers::GetRoot<replay::SarCycleResult>(bytes.data());
  SarCycleResult decoded;
  decoded.input_cycle_index = fb->input_cycle_index();
  FromFbOutputFrame(fb->output_frame(), &decoded.output_frame);
  const auto* focused = fb->focused_image();
  if (focused == nullptr || focused->real_values() == nullptr ||
      focused->imaginary_values() == nullptr) {
    return false;
  }
  const std::size_t row_count = focused->row_count();
  const std::size_t column_count = focused->column_count();
  if (column_count != 0U &&
      row_count > std::numeric_limits<std::size_t>::max() / column_count) {
    return false;
  }
  const std::size_t focused_expected_size = row_count * column_count;
  const std::size_t real_size = focused->real_values()->size();
  const std::size_t imaginary_size = focused->imaginary_values()->size();
  const SarFocusedImageSource source = static_cast<SarFocusedImageSource>(focused->source());
  switch (source) {
    case SarFocusedImageSource::kNone:
      if (row_count != 0U || column_count != 0U || focused->is_placeholder() || real_size != 0U ||
          imaginary_size != 0U) {
        return false;
      }
      break;
    case SarFocusedImageSource::kL1Rda:
    case SarFocusedImageSource::kL3Bp:
      if (row_count == 0U || column_count == 0U ||
          (focused->is_placeholder() && (real_size != 0U || imaginary_size != 0U)) ||
          (!focused->is_placeholder() &&
           (real_size != focused_expected_size || imaginary_size != focused_expected_size))) {
        return false;
      }
      break;
    default:
      return false;
  }
  decoded.focused_image.source = source;
  decoded.focused_image.row_count = focused->row_count();
  decoded.focused_image.column_count = focused->column_count();
  decoded.focused_image.is_placeholder = focused->is_placeholder();
  decoded.focused_image.real_values.assign(focused->real_values()->begin(),
                                           focused->real_values()->end());
  decoded.focused_image.imaginary_values.assign(focused->imaginary_values()->begin(),
                                                focused->imaginary_values()->end());
  for (std::size_t index = 0U; index < real_size; ++index) {
    if (std::isfinite(decoded.focused_image.real_values[index]) == 0 ||
        std::isfinite(decoded.focused_image.imaginary_values[index]) == 0) {
      return false;
    }
  }
  if (fb->diagnostics()) {
    for (const auto* issue : *fb->diagnostics()) {
      SarDiagnosticIssue decoded_issue;
      decoded_issue.severity = static_cast<SarDiagnosticSeverity>(issue->severity());
      decoded_issue.code = issue->code() ? issue->code()->str() : std::string();
      decoded_issue.message = issue->message() ? issue->message()->str() : std::string();
      decoded.diagnostics.push_back(decoded_issue);
    }
  }
  if (fb->raw_phase_history()) {
    const auto* raw = fb->raw_phase_history();
    const std::size_t pulse_count = raw->pulse_count();
    const std::size_t samples_per_pulse = raw->samples_per_pulse();
    if (samples_per_pulse != 0U &&
        pulse_count > std::numeric_limits<std::size_t>::max() / samples_per_pulse) {
      return false;
    }
    const std::size_t expected_size = pulse_count * samples_per_pulse;
    if (raw->i_values() == nullptr || raw->q_values() == nullptr ||
        raw->i_values()->size() != expected_size || raw->q_values()->size() != expected_size) {
      return false;
    }
    decoded.raw_phase_history.source =
        static_cast<SarRawPhaseHistorySource>(raw->source());
    decoded.raw_phase_history.pulse_count = raw->pulse_count();
    decoded.raw_phase_history.samples_per_pulse = raw->samples_per_pulse();
    decoded.raw_phase_history.i_values.assign(raw->i_values()->begin(), raw->i_values()->end());
    decoded.raw_phase_history.q_values.assign(raw->q_values()->begin(), raw->q_values()->end());
    for (std::size_t index = 0U; index < expected_size; ++index) {
      if (std::isfinite(decoded.raw_phase_history.i_values[index]) == 0 ||
          std::isfinite(decoded.raw_phase_history.q_values[index]) == 0) {
        return false;
      }
    }
  }
  decoded.has_error = fb->has_error();
  decoded.executed_this_cycle = fb->executed_this_cycle();
  decoded.reused_previous_output = fb->reused_previous_output();
  decoded.abort_reason = fb->abort_reason() ? fb->abort_reason()->str() : std::string();
  *out = decoded;
  return true;
}

std::string EncodeSarSessionConfig(const config::SarSessionConfig& value) {
  flatbuffers::FlatBufferBuilder fbb(512);
  const auto hardware = BuildHardwareConfig(fbb, value.hardware);
  const auto mission = BuildMissionConfig(fbb, value.mission);
  const auto policy = BuildPolicyConfig(fbb, value.policy);
  const auto environment = BuildEnvironmentConfig(fbb, value.environment);
  fbb.Finish(replay::CreateSarSessionConfig(fbb, hardware, mission, policy, environment));
  return oneq::common::replay::CopyFinishedFlatbuffer(fbb);
}

bool DecodeSarSessionConfig(const std::string& bytes, config::SarSessionConfig* out) {
  if (!out) {
    return false;
  }
  flatbuffers::Verifier verifier(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size());
  if (!verifier.VerifyBuffer<replay::SarSessionConfig>()) {
    return false;
  }
  const auto* fb = flatbuffers::GetRoot<replay::SarSessionConfig>(bytes.data());
  FromFbHardwareConfig(fb->hardware(), &out->hardware);
  FromFbMissionConfig(fb->mission(), &out->mission);
  FromFbPolicyConfig(fb->policy(), &out->policy);
  FromFbEnvironmentConfig(fb->environment(), &out->environment);
  return true;
}

std::string EncodeSarRuntimeConfigPatch(const config::SarRuntimeConfigPatch& value) {
  flatbuffers::FlatBufferBuilder fbb(256);
  fbb.Finish(replay::CreateSarRuntimeConfigPatch(
      fbb, value.has_enable_raw_echo_generation, value.enable_raw_echo_generation,
      value.has_enable_range_compression, value.enable_range_compression,
      value.has_enable_l1_rda_imaging, value.enable_l1_rda_imaging,
      value.has_retain_raw_phase_history, value.retain_raw_phase_history,
      value.has_retain_focused_image, value.retain_focused_image, value.has_minimum_snr_db,
      value.minimum_snr_db));
  return oneq::common::replay::CopyFinishedFlatbuffer(fbb);
}

bool DecodeSarRuntimeConfigPatch(const std::string& bytes, config::SarRuntimeConfigPatch* out) {
  if (!out) {
    return false;
  }
  flatbuffers::Verifier verifier(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size());
  if (!verifier.VerifyBuffer<replay::SarRuntimeConfigPatch>()) {
    return false;
  }
  const auto* fb = flatbuffers::GetRoot<replay::SarRuntimeConfigPatch>(bytes.data());
  out->has_enable_raw_echo_generation = fb->has_enable_raw_echo_generation();
  out->enable_raw_echo_generation = fb->enable_raw_echo_generation();
  out->has_enable_range_compression = fb->has_enable_range_compression();
  out->enable_range_compression = fb->enable_range_compression();
  out->has_enable_l1_rda_imaging = fb->has_enable_l1_rda_imaging();
  out->enable_l1_rda_imaging = fb->enable_l1_rda_imaging();
  out->has_retain_raw_phase_history = fb->has_retain_raw_phase_history();
  out->retain_raw_phase_history = fb->retain_raw_phase_history();
  out->has_retain_focused_image = fb->has_retain_focused_image();
  out->retain_focused_image = fb->retain_focused_image();
  out->has_minimum_snr_db = fb->has_minimum_snr_db();
  out->minimum_snr_db = fb->minimum_snr_db();
  return true;
}

}  // namespace session
}  // namespace sar
