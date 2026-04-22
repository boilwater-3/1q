#include "1q/airborne_radar/session/RadarTraceSession.h"

#include <cstddef>
#include <sstream>
#include <string>

#include "1q/airborne_radar/config/RadarRuntimeConfigPatch.h"
#include "1q/airborne_radar/session/RadarSessionFactory.h"

namespace airborne_radar {
namespace session {
namespace {

std::string JsonBool(bool value) { return value ? "true" : "false"; }

std::string MakeEulerAnglesPayload(const model::EulerAnglesDeg& value) {
  std::ostringstream stream;
  stream << "{"
         << "\"yaw_deg\":" << value.yaw_deg << ","
         << "\"pitch_deg\":" << value.pitch_deg << ","
         << "\"roll_deg\":" << value.roll_deg << "}";
  return stream.str();
}

std::string MakeAzElPayload(const model::AzimuthElevationDeg& value) {
  std::ostringstream stream;
  stream << "{"
         << "\"az_deg\":" << value.az_deg << ","
         << "\"el_deg\":" << value.el_deg << "}";
  return stream.str();
}

std::string MakeAzElLimitsPayload(const model::AzimuthElevationLimitsDeg& value) {
  std::ostringstream stream;
  stream << "{"
         << "\"az_min_deg\":" << value.az_min_deg << ","
         << "\"az_max_deg\":" << value.az_max_deg << ","
         << "\"el_min_deg\":" << value.el_min_deg << ","
         << "\"el_max_deg\":" << value.el_max_deg << "}";
  return stream.str();
}

std::string MakeCommandedBeamwidthPayload(const model::CommandedBeamwidthDeg& value) {
  std::ostringstream stream;
  stream << "{"
         << "\"commanded_az_beamwidth_deg\":"
         << value.commanded_az_beamwidth_deg << ","
         << "\"commanded_el_beamwidth_deg\":"
         << value.commanded_el_beamwidth_deg << "}";
  return stream.str();
}

std::string MakeOrientationPayload(const model::RadarOrientationConfig& value) {
  std::ostringstream stream;
  stream << "{"
         << "\"mount_angles_deg\":" << MakeEulerAnglesPayload(value.mount_angles_deg) << ","
         << "\"scan_center_deg\":" << MakeAzElPayload(value.scan_center_deg) << ","
         << "\"mechanical_scan_limits_deg\":"
         << MakeAzElLimitsPayload(value.mechanical_scan_limits_deg) << ","
         << "\"electronic_scan_limits_deg\":"
         << MakeAzElLimitsPayload(value.electronic_scan_limits_deg) << ","
         << "\"scan_start_position\":"
         << static_cast<int>(value.scan_start_position) << ","
         << "\"scan_sequence\":" << static_cast<int>(value.scan_sequence) << ","
         << "\"work_sub_mode\":" << static_cast<int>(value.work_sub_mode) << ","
         << "\"commanded_beamwidth_enabled\":"
         << JsonBool(value.commanded_beamwidth_enabled) << ","
         << "\"commanded_beamwidth_deg\":"
         << MakeCommandedBeamwidthPayload(value.commanded_beamwidth_deg) << ","
         << "\"stabilization_mode\":"
         << static_cast<int>(value.stabilization_mode) << "}";
  return stream.str();
}

std::string MakeDetectionPayload(const config::DetectionConfig& value) {
  std::ostringstream stream;
  stream << "{"
         << "\"enable_physics_detection\":"
         << JsonBool(value.enable_physics_detection) << ","
         << "\"swerling_model\":" << static_cast<int>(value.swerling_model) << ","
         << "\"transmitter\":{"
         << "\"peak_power_w\":" << value.transmitter.peak_power_w << ","
         << "\"frequency_hz\":" << value.transmitter.frequency_hz << ","
         << "\"bandwidth_hz\":" << value.transmitter.bandwidth_hz << ","
         << "\"pulse_width_s\":" << value.transmitter.pulse_width_s << ","
         << "\"prf_hz\":" << value.transmitter.prf_hz << ","
         << "\"transmit_loss_db\":" << value.transmitter.transmit_loss_db << "},"
         << "\"antenna\":{"
         << "\"main_beam_gain_db\":" << value.antenna.main_beam_gain_db << ","
         << "\"nominal_az_beamwidth_deg\":"
         << value.antenna.nominal_az_beamwidth_deg << ","
         << "\"nominal_el_beamwidth_deg\":"
         << value.antenna.nominal_el_beamwidth_deg << ","
         << "\"enable_directional_pattern\":"
         << JsonBool(value.antenna.enable_directional_pattern) << ","
         << "\"pattern\":{"
         << "\"model_type\":" << static_cast<int>(value.antenna.pattern.model_type) << ","
         << "\"max_sidelobe_level_db\":"
         << value.antenna.pattern.max_sidelobe_level_db << ","
         << "\"backlobe_level_db\":" << value.antenna.pattern.backlobe_level_db << ","
         << "\"scan_loss_coeff_db_per_deg2\":"
         << value.antenna.pattern.scan_loss_coeff_db_per_deg2 << ","
         << "\"max_scan_loss_db\":" << value.antenna.pattern.max_scan_loss_db << ","
         << "\"boresight_offset_deg\":"
         << MakeAzElPayload(value.antenna.pattern.boresight_offset_deg) << "}},"
         << "\"receiver\":{"
         << "\"noise_figure_db\":" << value.receiver.noise_figure_db << ","
         << "\"receive_loss_db\":" << value.receiver.receive_loss_db << "},"
         << "\"detection_policy\":{"
         << "\"cfar_pfa\":" << value.detection_policy.cfar_pfa << ","
         << "\"min_snr_db\":" << value.detection_policy.min_snr_db << "},"
         << "\"rcs_physics\":{"
         << "\"enable_physical_rcs\":"
         << JsonBool(value.rcs_physics.enable_physical_rcs) << ","
         << "\"frequency_hz\":" << value.rcs_physics.frequency_hz << ","
         << "\"physics_mix_ratio\":" << value.rcs_physics.physics_mix_ratio << ","
         << "\"cylinder_weight\":" << value.rcs_physics.cylinder_weight << ","
         << "\"min_equivalent_radius_m\":"
         << value.rcs_physics.min_equivalent_radius_m << ","
         << "\"max_equivalent_radius_m\":"
         << value.rcs_physics.max_equivalent_radius_m << ","
         << "\"min_rcs_m2\":" << value.rcs_physics.min_rcs_m2 << ","
         << "\"max_rcs_m2\":" << value.rcs_physics.max_rcs_m2 << ","
         << "\"bistatic_psi_offset_deg\":"
         << value.rcs_physics.bistatic_psi_offset_deg << "},"
         << "\"min_detection_margin_db\":" << value.min_detection_margin_db << ","
         << "\"pulse_count\":" << value.pulse_count << "}";
  return stream.str();
}

std::string MakeBeamControlPayload(const config::BeamControlConfig& value) {
  std::ostringstream stream;
  stream << "{"
         << "\"pointing\":{"
         << "\"default_scan_center_deg\":"
         << MakeAzElPayload(value.pointing.default_scan_center_deg) << ","
         << "\"nominal_beamwidth_deg\":"
         << MakeCommandedBeamwidthPayload(value.pointing.nominal_beamwidth_deg)
         << "},"
         << "\"scheduler\":{"
         << "\"azimuth_step_count_hint\":"
         << value.scheduler.azimuth_step_count_hint << ","
         << "\"elevation_step_count_hint\":"
         << value.scheduler.elevation_step_count_hint << ","
         << "\"prefer_dense_tas_sampling\":"
         << JsonBool(value.scheduler.prefer_dense_tas_sampling) << "}}";
  return stream.str();
}

std::string MakePolicyPayload(const config::RadarPolicyConfig& value) {
  std::ostringstream stream;
  stream << "{"
         << "\"beam_control\":" << MakeBeamControlPayload(value.beam_control) << ","
         << "\"association\":{"
         << "\"unassigned_cost\":" << value.association.unassigned_cost << ","
         << "\"use_distance_gate_hint\":"
         << JsonBool(value.association.use_distance_gate_hint) << ","
         << "\"distance_gate_sigma_hint\":"
         << value.association.distance_gate_sigma_hint << "},"
         << "\"tracking\":{"
         << "\"enable_kalman_filter\":"
         << JsonBool(value.tracking.enable_kalman_filter) << ","
         << "\"kalman_measurement_noise_std\":"
         << value.tracking.kalman_measurement_noise_std << ","
         << "\"kalman_update_backend\":"
         << static_cast<int>(value.tracking.kalman_update_backend) << ","
         << "\"speed_decay_ratio_on_loss\":"
         << value.tracking.speed_decay_ratio_on_loss << ","
         << "\"rcs_decay_ratio_on_loss\":"
         << value.tracking.rcs_decay_ratio_on_loss << "},"
         << "\"lifecycle\":{"
         << "\"confirm_hits\":" << value.lifecycle.confirm_hits << ","
         << "\"max_miss_before_lost\":"
         << value.lifecycle.max_miss_before_lost << ","
         << "\"max_lost_cycles\":" << value.lifecycle.max_lost_cycles << ","
         << "\"enable_imm_lifecycle\":"
         << JsonBool(value.lifecycle.enable_imm_lifecycle) << "},"
         << "\"imm\":{"
         << "\"enable_imm_lifecycle\":"
         << JsonBool(value.imm.enable_imm_lifecycle) << ","
         << "\"model_count_hint\":" << value.imm.model_count_hint << "}}";
  return stream.str();
}

std::string MakeAtmosphericPhysicsPayload(
    const environment::AtmosphericPhysicsConfig& value) {
  std::ostringstream stream;
  stream << "{"
         << "\"enable_physical_model\":"
         << JsonBool(value.enable_physical_model) << ","
         << "\"pressure_hpa\":" << value.pressure_hpa << ","
         << "\"temperature_k\":" << value.temperature_k << ","
         << "\"relative_humidity\":" << value.relative_humidity << "}";
  return stream.str();
}

std::string MakeAtmosphericContextPayload(
    const environment::AtmosphericDerivedContext& value) {
  std::ostringstream stream;
  stream << "{"
         << "\"has_simulation_unix_seconds\":"
         << JsonBool(value.has_simulation_unix_seconds) << ","
         << "\"simulation_unix_seconds\":" << value.simulation_unix_seconds << ","
         << "\"solar_flux_f107a\":" << value.solar_flux_f107a << ","
         << "\"solar_flux_f107\":" << value.solar_flux_f107 << ","
         << "\"geomagnetic_ap\":" << value.geomagnetic_ap << "}";
  return stream.str();
}

std::string MakeVegetationPayload(
    const environment::VegetationScatterPhysicsConfig& value) {
  std::ostringstream stream;
  stream << "{"
         << "\"cover_profile\":" << static_cast<int>(value.cover_profile) << ","
         << "\"enable_physical_model\":"
         << JsonBool(value.enable_physical_model) << "}";
  return stream.str();
}

std::string MakeJammerEmitterPayload(
    const environment::JammerEmitterState& value) {
  std::ostringstream stream;
  stream << "{"
         << "\"technique\":" << static_cast<int>(value.technique) << ","
         << "\"power_db\":" << value.power_db << ","
         << "\"js_db\":" << value.js_db << ","
         << "\"has_direction_deg\":" << JsonBool(value.has_direction_deg) << ","
         << "\"azimuth_deg\":" << value.azimuth_deg << ","
         << "\"elevation_deg\":" << value.elevation_deg << ","
         << "\"angular_span_deg\":" << value.angular_span_deg << ","
         << "\"confidence\":" << value.confidence << "}";
  return stream.str();
}

std::string MakeFlatbuffersPayload(const char* type_name,
                                   const RadarSessionConfig& value) {
  std::ostringstream stream;
  stream << "{"
         << "\"serializer\":\"flatbuffers\","
         << "\"platform\":\"windows\","
         << "\"type\":\"" << type_name << "\","
         << "\"hardware\":{\"detection\":"
         << MakeDetectionPayload(value.hardware.detection) << "},"
         << "\"mission\":{\"orientation\":"
         << MakeOrientationPayload(value.mission.orientation) << "},"
         << "\"policy\":" << MakePolicyPayload(value.policy) << ","
         << "\"jamming_sensitivity_profile\":"
         << static_cast<int>(value.jamming_sensitivity_profile) << "}";
  return stream.str();
}

std::string MakeFlatbuffersPayload(
    const char* type_name,
    const environment::EnvironmentSceneState& value) {
  std::ostringstream stream;
  stream << "{"
         << "\"serializer\":\"flatbuffers\","
         << "\"platform\":\"windows\","
         << "\"type\":\"" << type_name << "\","
         << "\"atmospheric_physics\":"
         << MakeAtmosphericPhysicsPayload(value.atmospheric_physics) << ","
         << "\"atmospheric_context\":"
         << MakeAtmosphericContextPayload(value.atmospheric_context) << ","
         << "\"vegetation_scatter_physics\":"
         << MakeVegetationPayload(value.vegetation_scatter_physics) << ","
         << "\"jammer_emitters\":[";
  for (std::size_t i = 0; i < value.jammer_emitters.size(); ++i) {
    if (i > 0U) {
      stream << ",";
    }
    stream << MakeJammerEmitterPayload(value.jammer_emitters[i]);
  }
  stream << "]}";
  return stream.str();
}

std::string MakeFlatbuffersPayload(
    const char* type_name,
    const config::RadarRuntimeConfigPatch& value) {
  std::ostringstream stream;
  stream << "{"
         << "\"serializer\":\"flatbuffers\","
         << "\"platform\":\"windows\","
         << "\"type\":\"" << type_name << "\","
         << "\"has_mission\":" << JsonBool(value.has_mission) << ","
         << "\"mission\":{\"orientation\":"
         << MakeOrientationPayload(value.mission.orientation) << "},"
         << "\"has_policy\":" << JsonBool(value.has_policy) << ","
         << "\"policy\":" << MakePolicyPayload(value.policy) << ","
         << "\"has_environment_runtime_config\":"
         << JsonBool(value.has_environment_runtime_config) << ","
         << "\"has_work_sub_mode\":" << JsonBool(value.has_work_sub_mode) << ","
         << "\"work_sub_mode\":" << static_cast<int>(value.work_sub_mode) << ","
         << "\"has_scan_center_deg\":"
         << JsonBool(value.has_scan_center_deg) << ","
         << "\"scan_center_deg\":" << MakeAzElPayload(value.scan_center_deg) << ","
         << "\"has_dwell_center_deg\":"
         << JsonBool(value.has_dwell_center_deg) << ","
         << "\"dwell_center_deg\":" << MakeAzElPayload(value.dwell_center_deg) << ","
         << "\"has_commanded_beamwidth_deg\":"
         << JsonBool(value.has_commanded_beamwidth_deg) << ","
         << "\"commanded_beamwidth_deg\":"
         << MakeCommandedBeamwidthPayload(value.commanded_beamwidth_deg) << ","
         << "\"has_commanded_beamwidth_enabled\":"
         << JsonBool(value.has_commanded_beamwidth_enabled) << ","
         << "\"commanded_beamwidth_enabled\":"
         << JsonBool(value.commanded_beamwidth_enabled) << "}";
  return stream.str();
}

template <typename T>
std::string MakeFlatbuffersPayload(const char* type_name, const T& /*value*/) {
  std::ostringstream stream;
  stream << "{"
         << "\"serializer\":\"flatbuffers\","
         << "\"platform\":\"windows\","
         << "\"type\":\"" << type_name << "\""
         << "}";
  return stream.str();
}

std::string MakeInputPayload(const RadarCycleInput& input) {
  std::ostringstream stream;
  stream << "{"
         << "\"serializer\":\"flatbuffers\","
         << "\"platform\":\"windows\","
         << "\"type\":\"RadarCycleInput\","
         << "\"dt_sec\":" << input.dt_sec << ","
         << "\"platform_pose\":{"
         << "\"position_m\":[" << input.platform_pose.position_m.x << ","
         << input.platform_pose.position_m.y << "," << input.platform_pose.position_m.z << "],"
         << "\"velocity_mps\":[" << input.platform_pose.velocity_mps.x << ","
         << input.platform_pose.velocity_mps.y << "," << input.platform_pose.velocity_mps.z
         << "],"
         << "\"attitude_deg\":{"
         << "\"yaw_deg\":" << input.platform_pose.attitude_deg.yaw_deg << ","
         << "\"pitch_deg\":" << input.platform_pose.attitude_deg.pitch_deg << ","
         << "\"roll_deg\":" << input.platform_pose.attitude_deg.roll_deg << "}"
         << "},"
         << "\"target_features\":[";
  for (std::size_t i = 0; i < input.target_features.size(); ++i) {
    const model::TargetFeature& target = input.target_features[i];
    if (i > 0U) {
      stream << ",";
    }
    stream << "{"
           << "\"external_target_id\":" << target.external_target_id << ","
           << "\"velocity_mps\":[" << target.current_track_velocity_x << ","
           << target.current_track_velocity_y << "," << target.current_track_velocity_z << "],"
           << "\"current_track_speed\":" << target.current_track_speed << ","
           << "\"current_track_rcs\":" << target.current_track_rcs << ","
           << "\"range_m\":" << target.range_m << ","
           << "\"has_cartesian_position\":"
           << (target.has_cartesian_position ? "true" : "false") << ","
           << "\"position_m\":[" << target.position_x << "," << target.position_y << ","
           << target.position_z << "],"
           << "\"target_swerling_type\":" << target.target_swerling_type << "}";
  }
  stream << "]}";
  return stream.str();
}

std::string MakeOutputPayload(const output::TrackOutputFrame& output) {
  std::ostringstream stream;
  stream << "{"
         << "\"serializer\":\"flatbuffers\","
         << "\"platform\":\"windows\","
         << "\"type\":\"TrackOutputFrame\","
         << "\"cycle_index\":" << output.cycle_index << ","
         << "\"published_track_count\":" << output.published_track_count
         << "}";
  return stream.str();
}

std::string MakeResultPayload(const RadarCycleResult& output) {
  std::ostringstream stream;
  stream << "{"
         << "\"serializer\":\"flatbuffers\","
         << "\"platform\":\"windows\","
         << "\"type\":\"RadarCycleResult\","
         << "\"validation_issue_count\":" << output.validation_issues.size() << ","
         << "\"executed_this_cycle\":" << (output.executed_this_cycle ? "true" : "false")
         << "}";
  return stream.str();
}

std::string MakeSceneInputPayload(const RadarCycleInput& input,
                                  const environment::EnvironmentSceneState& scene_state) {
  std::ostringstream stream;
  stream << "{"
         << "\"serializer\":\"flatbuffers\","
         << "\"platform\":\"windows\","
         << "\"type\":\"RadarCycleInputWithScene\","
         << "\"cycle_input\":" << MakeInputPayload(input) << ","
         << "\"jammer_emitter_count\":" << scene_state.jammer_emitters.size()
         << "}";
  return stream.str();
}

}  // namespace

RadarTraceSession::RadarTraceSession(const RadarSessionConfig& config,
                                     RadarTraceSessionOptions options)
    : session_(RadarSessionFactory::Create(config)),
      sink_(std::move(options.sink)),
      replay_writer_(std::move(options.replay_writer)) {
  if (replay_writer_ && options.trace_config_on_construct) {
    RecordReplay("session_config", "RadarSessionConfig",
                 MakeFlatbuffersPayload("RadarSessionConfig", config));
  }
  if (sink_ && options.trace_config_on_construct) {
    Record("config", MakeFlatbuffersPayload("RadarSessionConfig", config));
  }
}

output::TrackOutputFrame RadarTraceSession::Step(const RadarCycleInput& input) {
  if (replay_writer_) {
    RecordReplay("cycle_input", "RadarCycleInput", MakeInputPayload(input));
  }
  if (sink_) {
    Record("input", MakeInputPayload(input));
  }
  const output::TrackOutputFrame output = session_.Step(input);
  if (replay_writer_) {
    RecordReplay("cycle_output", "TrackOutputFrame", MakeOutputPayload(output),
                 output.cycle_index);
  }
  if (sink_) {
    Record("output", MakeOutputPayload(output));
  }
  return output;
}

output::TrackOutputFrame RadarTraceSession::Step(
    const RadarCycleInput& input, const environment::EnvironmentSceneState& scene_state) {
  if (replay_writer_) {
    RecordReplay("cycle_input", "RadarCycleInput", MakeInputPayload(input));
    RecordReplay("scene_state", "EnvironmentSceneState",
                 MakeFlatbuffersPayload("EnvironmentSceneState", scene_state));
  }
  if (sink_) {
    Record("input", MakeSceneInputPayload(input, scene_state));
  }
  const output::TrackOutputFrame output = session_.Step(input, scene_state);
  if (replay_writer_) {
    RecordReplay("cycle_output", "TrackOutputFrame", MakeOutputPayload(output),
                 output.cycle_index);
  }
  if (sink_) {
    Record("output", MakeOutputPayload(output));
  }
  return output;
}

RadarCycleResult RadarTraceSession::StepWithResult(const RadarCycleInput& input) {
  if (replay_writer_) {
    RecordReplay("cycle_input", "RadarCycleInput", MakeInputPayload(input));
  }
  if (sink_) {
    Record("input", MakeInputPayload(input));
  }
  const RadarCycleResult output = session_.StepWithResult(input);
  if (replay_writer_) {
    RecordReplay("cycle_output", "RadarCycleResult", MakeResultPayload(output),
                 output.track_output_frame.cycle_index);
  }
  if (sink_) {
    Record("output", MakeResultPayload(output));
  }
  return output;
}

RadarCycleResult RadarTraceSession::StepWithResult(
    const RadarCycleInput& input, const environment::EnvironmentSceneState& scene_state) {
  if (replay_writer_) {
    RecordReplay("cycle_input", "RadarCycleInput", MakeInputPayload(input));
    RecordReplay("scene_state", "EnvironmentSceneState",
                 MakeFlatbuffersPayload("EnvironmentSceneState", scene_state));
  }
  if (sink_) {
    Record("input", MakeSceneInputPayload(input, scene_state));
  }
  const RadarCycleResult output = session_.StepWithResult(input, scene_state);
  if (replay_writer_) {
    RecordReplay("cycle_output", "RadarCycleResult", MakeResultPayload(output),
                 output.track_output_frame.cycle_index);
  }
  if (sink_) {
    Record("output", MakeResultPayload(output));
  }
  return output;
}

void RadarTraceSession::ApplyRuntimeConfig(const config::RadarRuntimeConfigPatch& patch) {
  if (replay_writer_) {
    RecordReplay("runtime_config_patch", "RadarRuntimeConfigPatch",
                 MakeFlatbuffersPayload("RadarRuntimeConfigPatch", patch));
  }
  session_.ApplyRuntimeConfig(patch);
  if (sink_) {
    Record("runtime_config", MakeFlatbuffersPayload("RadarRuntimeConfigPatch", patch));
  }
}

const std::vector<extension::control::RadarCommand>& RadarTraceSession::GetSubmittedCommands()
    const {
  return session_.GetSubmittedCommands();
}

bool RadarTraceSession::HasLatestControlProfile() const {
  return session_.HasLatestControlProfile();
}

const extension::control::RadarControlProfile& RadarTraceSession::GetLatestControlProfile() const {
  return session_.GetLatestControlProfile();
}

extension::AssociationQualityMetrics RadarTraceSession::GetLastAssociationQualityMetrics() const {
  return session_.GetLastAssociationQualityMetrics();
}

RadarSession& RadarTraceSession::session() { return session_; }

const RadarSession& RadarTraceSession::session() const { return session_; }

void RadarTraceSession::Record(const std::string& phase, const std::string& payload_json) const {
  sink_->Record("airborne_radar", phase, payload_json);
}

void RadarTraceSession::RecordReplay(const std::string& event_type,
                                     const std::string& payload_type,
                                     const std::string& payload_json) const {
  oneq::replay::ReplayTraceEvent event;
  event.module = "airborne_radar";
  event.event_type = event_type;
  event.payload_type = payload_type;
  event.payload_json = payload_json;
  replay_writer_->WriteEvent(event);
}

void RadarTraceSession::RecordReplay(const std::string& event_type,
                                     const std::string& payload_type,
                                     const std::string& payload_json,
                                     std::uint32_t cycle_index) const {
  oneq::replay::ReplayTraceEvent event;
  event.module = "airborne_radar";
  event.event_type = event_type;
  event.payload_type = payload_type;
  event.payload_json = payload_json;
  event.has_cycle_index = true;
  event.cycle_index = cycle_index;
  replay_writer_->WriteEvent(event);
}

}  // namespace session
}  // namespace airborne_radar
