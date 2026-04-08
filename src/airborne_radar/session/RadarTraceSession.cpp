#include "1q/airborne_radar/session/RadarTraceSession.h"

#include <cstddef>
#include <string>

#include <nlohmann/json.hpp>

#include "1q/airborne_radar/config/RadarRuntimeConfigBuilder.h"
#include "1q/airborne_radar/session/RadarSessionFactory.h"

namespace airborne_radar {
namespace session {
namespace {

using Json = nlohmann::ordered_json;

template <typename Container, typename Serializer>
Json SerializeArray(const Container& values, const Serializer& serializer) {
  Json result = Json::array();
  for (std::size_t i = 0; i < values.size(); ++i) {
    result.push_back(serializer(values[i]));
  }
  return result;
}

template <typename T>
Json NumberArray3(T first, T second, T third) {
  Json json = Json::array();
  json.push_back(first);
  json.push_back(second);
  json.push_back(third);
  return json;
}

Json BuildJson(const model::EulerAnglesDeg& value) {
  Json json;
  json["yaw_deg"] = value.yaw_deg;
  json["pitch_deg"] = value.pitch_deg;
  json["roll_deg"] = value.roll_deg;
  return json;
}

Json BuildJson(const model::AzimuthElevationDeg& value) {
  Json json;
  json["az_deg"] = value.az_deg;
  json["el_deg"] = value.el_deg;
  return json;
}

Json BuildJson(const model::AzimuthElevationLimitsDeg& value) {
  Json json;
  json["az_min_deg"] = value.az_min_deg;
  json["az_max_deg"] = value.az_max_deg;
  json["el_min_deg"] = value.el_min_deg;
  json["el_max_deg"] = value.el_max_deg;
  return json;
}

Json BuildJson(const model::CommandedBeamwidthDeg& value) {
  Json json;
  json["commanded_az_beamwidth_deg"] = value.commanded_az_beamwidth_deg;
  json["commanded_el_beamwidth_deg"] = value.commanded_el_beamwidth_deg;
  return json;
}

Json BuildJson(const model::RadarOrientationConfig& value) {
  Json json;
  json["mount_angles_deg"] = BuildJson(value.mount_angles_deg);
  json["scan_center_deg"] = BuildJson(value.scan_center_deg);
  json["mechanical_scan_limits_deg"] = BuildJson(value.mechanical_scan_limits_deg);
  json["electronic_scan_limits_deg"] = BuildJson(value.electronic_scan_limits_deg);
  json["scan_start_position"] = static_cast<int>(value.scan_start_position);
  json["scan_sequence"] = static_cast<int>(value.scan_sequence);
  json["work_sub_mode"] = static_cast<int>(value.work_sub_mode);
  json["dwell_center_deg"] = BuildJson(value.dwell_center_deg);
  json["commanded_beamwidth_enabled"] = value.commanded_beamwidth_enabled;
  json["commanded_beamwidth_deg"] = BuildJson(value.commanded_beamwidth_deg);
  json["stabilization_mode"] = static_cast<int>(value.stabilization_mode);
  return json;
}

Json BuildJson(const config::AntennaPatternConfig& value) {
  Json json;
  json["model_type"] = static_cast<int>(value.model_type);
  json["max_sidelobe_level_db"] = value.max_sidelobe_level_db;
  json["backlobe_level_db"] = value.backlobe_level_db;
  json["scan_loss_coeff_db_per_deg2"] = value.scan_loss_coeff_db_per_deg2;
  json["max_scan_loss_db"] = value.max_scan_loss_db;
  json["boresight_offset_deg"] = BuildJson(value.boresight_offset_deg);
  return json;
}

Json BuildJson(const config::SignalDetectionConfig& value) {
  Json json;
  json["enable_physics_detection"] = value.enable_physics_detection;
  json["min_detection_margin_db"] = value.min_detection_margin_db;
  json["pulse_count"] = value.pulse_count;

  Json transmitter;
  transmitter["peak_power_w"] = value.transmitter.peak_power_w;
  transmitter["frequency_hz"] = value.transmitter.frequency_hz;
  transmitter["bandwidth_hz"] = value.transmitter.bandwidth_hz;
  transmitter["pulse_width_s"] = value.transmitter.pulse_width_s;
  transmitter["prf_hz"] = value.transmitter.prf_hz;
  transmitter["transmit_loss_db"] = value.transmitter.transmit_loss_db;

  Json antenna;
  antenna["main_beam_gain_db"] = value.antenna.main_beam_gain_db;
  antenna["nominal_az_beamwidth_deg"] = value.antenna.nominal_az_beamwidth_deg;
  antenna["nominal_el_beamwidth_deg"] = value.antenna.nominal_el_beamwidth_deg;
  antenna["enable_directional_pattern"] = value.antenna.enable_directional_pattern;
  antenna["pattern"] = BuildJson(value.antenna.pattern);

  Json receiver;
  receiver["noise_figure_db"] = value.receiver.noise_figure_db;
  receiver["receive_loss_db"] = value.receiver.receive_loss_db;

  Json detection;
  detection["cfar_pfa"] = value.detection_policy.cfar_pfa;
  detection["min_snr_db"] = value.detection_policy.min_snr_db;

  Json radar_system;
  radar_system["transmitter"] = transmitter;
  radar_system["antenna"] = antenna;
  radar_system["receiver"] = receiver;
  radar_system["detection"] = detection;

  json["radar_system"] = radar_system;
  return json;
}

Json BuildJson(const config::SignalTrackingConfig& value) {
  Json json;
  json["enable_kalman_filter"] = value.enable_kalman_filter;
  json["kalman_measurement_noise_std"] = value.kalman_measurement_noise_std;
  return json;
}

Json BuildJson(const config::LifecycleConfig& value) {
  Json json;
  json["confirm_hits"] = value.confirm_hits;
  json["max_miss_before_lost"] = value.max_miss_before_lost;
  json["max_lost_cycles"] = value.max_lost_cycles;
  return json;
}

Json BuildJson(const config::SignalLifecycleConfig& value) {
  Json json;
  json["lifecycle_config"] = BuildJson(value.lifecycle_config);
  json["enable_imm_lifecycle"] = value.enable_imm_lifecycle;
  return json;
}

Json BuildJson(const config::SignalPipelineConfig& value) {
  Json json;
  json["detection"] = BuildJson(value.detection);

  Json beam_control;
  beam_control["radar_orientation"] = BuildJson(value.beam_control.radar_orientation);
  beam_control["platform_attitude_deg"] = BuildJson(value.beam_control.platform_attitude_deg);
  json["beam_control"] = beam_control;

  json["tracking"] = BuildJson(value.tracking);
  json["lifecycle"] = BuildJson(value.lifecycle);
  return json;
}

Json BuildJson(const environment::AtmosphericPhysicsConfig& value) {
  Json json;
  json["enable_physical_model"] = value.enable_physical_model;
  json["frequency_hz"] = value.frequency_hz;
  json["path_length_m"] = value.path_length_m;
  json["radar_altitude_m"] = value.radar_altitude_m;
  json["target_altitude_m"] = value.target_altitude_m;
  json["elevation_deg"] = value.elevation_deg;
  json["pressure_hpa"] = value.pressure_hpa;
  json["temperature_k"] = value.temperature_k;
  json["relative_humidity"] = value.relative_humidity;
  json["k_factor"] = value.k_factor;
  json["day_of_year"] = value.day_of_year;
  json["solar_flux_f107a"] = value.solar_flux_f107a;
  json["solar_flux_f107"] = value.solar_flux_f107;
  json["geomagnetic_ap"] = value.geomagnetic_ap;
  return json;
}

Json BuildJson(const environment::JammerSourceFact& value) {
  Json json;
  json["technique"] = static_cast<int>(value.technique);
  json["power_db"] = value.power_db;
  json["js_db"] = value.js_db;
  json["frequency_overlap_ratio"] = value.frequency_overlap_ratio;
  json["prf_lock_risk"] = value.prf_lock_risk;
  json["azimuth_deg"] = value.azimuth_deg;
  json["elevation_deg"] = value.elevation_deg;
  json["angular_span_deg"] = value.angular_span_deg;
  json["in_sidelobe"] = value.in_sidelobe;
  json["confidence"] = value.confidence;
  return json;
}

Json BuildJson(const environment::EnvironmentModelConfig& value) {
  Json json;
  json["base_propagation_loss_db"] = value.base_propagation_loss_db;
  json["atmospheric_attenuation_db"] = value.atmospheric_attenuation_db;
  json["terrain_reflection_db"] = value.terrain_reflection_db;
  json["clutter_power_db"] = value.clutter_power_db;
  json["atmospheric_physics"] = BuildJson(value.atmospheric_physics);
  json["jammer_sources"] =
      SerializeArray(value.jammer_sources, [](const environment::JammerSourceFact& source) {
        return BuildJson(source);
      });
  return json;
}

Json BuildJson(const environment::EnvironmentRuntimeConfigPatch& value) {
  Json json;
  json["has_model_config"] = value.has_model_config;
  json["model_config"] = BuildJson(value.model_config);
  json["has_jamming_detection_threshold_db"] = value.has_jamming_detection_threshold_db;
  json["jamming_detection_threshold_db"] = value.jamming_detection_threshold_db;
  return json;
}

Json BuildJson(const environment::EnvironmentSceneState& value) {
  Json json;
  json["base_propagation_loss_db"] = value.base_propagation_loss_db;
  json["atmospheric_attenuation_db"] = value.atmospheric_attenuation_db;
  json["terrain_reflection_db"] = value.terrain_reflection_db;
  json["clutter_power_db"] = value.clutter_power_db;
  json["atmospheric_physics"] = BuildJson(value.atmospheric_physics);
  json["jammer_emitters"] =
      SerializeArray(value.jammer_emitters, [](const environment::JammerSourceFact& source) {
        return BuildJson(source);
      });
  return json;
}

Json BuildJson(const RadarSessionConfig& value) {
  Json json;
  config::SignalPipelineConfig pipeline_config;
  pipeline_config.detection = value.detection;
  pipeline_config.beam_control = value.beam_control;
  pipeline_config.tracking = value.tracking;
  pipeline_config.lifecycle = value.lifecycle;
  json["signal_pipeline_config"] = BuildJson(pipeline_config);
  json["environment_default_config"] = {
      {"model_config", BuildJson(value.environment_default_config.model_config)},
      {"jamming_detection_threshold_db",
       value.environment_default_config.jamming_detection_threshold_db}};
  return json;
}

Json BuildJson(const model::TargetFeature& value) {
  Json json;
  json["external_target_id"] = value.external_target_id;
  json["velocity_mps"] = NumberArray3(value.current_track_velocity_x, value.current_track_velocity_y,
                                        value.current_track_velocity_z);
  json["current_track_speed"] = value.current_track_speed;
  json["current_track_rcs"] = value.current_track_rcs;
  json["range_m"] = value.range_m;
  json["has_cartesian_position"] = value.has_cartesian_position;
  json["position_m"] = NumberArray3(value.position_x, value.position_y, value.position_z);
  json["target_swerling_type"] = value.target_swerling_type;
  return json;
}

Json BuildJson(const RadarCycleInput& value) {
  Json json;
  json["dt_sec"] = value.dt_sec;
  json["platform_attitude_deg"] = BuildJson(value.platform_attitude_deg);
  json["target_features"] =
      SerializeArray(value.target_features, [](const model::TargetFeature& target) {
        return BuildJson(target);
      });
  return json;
}

Json BuildJson(const ValidationIssue& value) {
  Json json;
  json["severity"] = static_cast<int>(value.severity);
  json["code"] = static_cast<int>(value.code);
  json["target_index"] = value.target_index;
  json["message"] = value.message;
  return json;
}

Json BuildJson(const extension::control::RadarCommand& value) {
  Json json;
  json["type"] = static_cast<int>(value.type);
  json["source"] = static_cast<int>(value.source);
  return json;
}

Json BuildJson(const extension::control::RadarControlProfile& value) {
  Json json;
  json["version"] = value.version;
  json["enable_lpi_power_control"] = value.enable_lpi_power_control;
  json["lpi_power_scale"] = value.lpi_power_scale;
  json["enable_lpi_beamforming"] = value.enable_lpi_beamforming;
  json["lpi_dwell_scale"] = value.lpi_dwell_scale;
  json["enable_agility_frequency"] = value.enable_agility_frequency;
  json["agility_frequency_hop_phase"] = static_cast<unsigned int>(value.agility_frequency_hop_phase);
  json["enable_sidelobe_canceller"] = value.enable_sidelobe_canceller;
  json["enable_adaptive_beamforming"] = value.enable_adaptive_beamforming;
  json["enable_eccm_rejitter"] = value.enable_eccm_rejitter;
  json["eccm_burnthrough_gain"] = value.eccm_burnthrough_gain;
  return json;
}

Json BuildJson(const extension::AssociationQualityMetrics& value) {
  Json json;
  json["prior_track_count"] = value.prior_track_count;
  json["detection_count"] = value.detection_count;
  json["matched_count"] = value.matched_count;
  json["new_track_count"] = value.new_track_count;
  json["missed_track_count"] = value.missed_track_count;
  json["match_rate"] = value.match_rate;
  json["new_track_rate"] = value.new_track_rate;
  json["missed_track_rate"] = value.missed_track_rate;
  json["mean_match_cost"] = value.mean_match_cost;
  json["p95_match_cost"] = value.p95_match_cost;
  json["dominant_jamming_semantic"] = static_cast<int>(value.dominant_jamming_semantic);
  json["jamming_severity"] = value.jamming_severity;
  json["association_stress"] = value.association_stress;
  return json;
}

Json BuildJson(const model::DecisionTrackSnapshot& value) {
  Json state;
  state["association_key"] = value.state.association_key;
  state["external_target_id"] = value.state.external_target_id;
  state["status"] = static_cast<int>(value.state.status);
  state["position_m"] = NumberArray3(value.state.position_x, value.state.position_y,
                                      value.state.position_z);
  state["velocity_mps"] = NumberArray3(value.state.velocity_x, value.state.velocity_y,
                                        value.state.velocity_z);
  state["speed"] = value.state.speed;
  state["acceleration_mps2"] = NumberArray3(value.state.acceleration_x,
                                             value.state.acceleration_y,
                                             value.state.acceleration_z);
  state["acceleration"] = value.state.acceleration;
  state["rcs"] = value.state.rcs;
  state["jamming_detected"] = value.state.jamming_detected;
  state["hit_count"] = value.state.hit_count;
  state["miss_count"] = value.state.miss_count;

  Json evidence;
  evidence["has_measurement_evidence"] = value.evidence.has_measurement_evidence;
  evidence["updated_this_cycle"] = value.evidence.updated_this_cycle;
  evidence["predicted_only_this_cycle"] = value.evidence.predicted_only_this_cycle;
  evidence["matched_existing_track"] = value.evidence.matched_existing_track;
  evidence["association_cost"] = value.evidence.association_cost;
  evidence["detection_margin_db"] = value.evidence.detection_margin_db;
  evidence["used_position_association"] = value.evidence.used_position_association;
  evidence["used_external_association_seeds"] = value.evidence.used_external_association_seeds;

  Json json;
  json["state"] = state;
  json["evidence"] = evidence;
  return json;
}

Json BuildJson(const output::TrackOutputFrame& value) {
  Json json;
  json["cycle_index"] = value.cycle_index;
  json["batch_id"] = value.batch_id;
  json["published_track_count"] = value.published_track_count;
  json["confirmed_track_count"] = value.confirmed_track_count;
  json["contains_lost_tracks"] = value.contains_lost_tracks;
  json["tracks"] =
      SerializeArray(value.tracks, [](const model::DecisionTrackSnapshot& track) {
        return BuildJson(track);
      });
  return json;
}

Json BuildJson(const RadarCycleResult& value) {
  Json json;
  json["track_output_frame"] = BuildJson(value.track_output_frame);
  json["submitted_commands"] =
      SerializeArray(value.submitted_commands, [](const extension::control::RadarCommand& command) {
        return BuildJson(command);
      });
  json["validation_issues"] =
      SerializeArray(value.validation_issues, [](const ValidationIssue& issue) {
        return BuildJson(issue);
      });
  json["has_validation_error"] = value.has_validation_error;
  json["has_control_profile"] = value.has_control_profile;
  json["control_profile"] = BuildJson(value.control_profile);
  json["association_quality_metrics"] = BuildJson(value.association_quality_metrics);
  return json;
}

Json BuildJson(const config::RadarRuntimeConfigPatch& value) {
  Json json;
  json["has_signal_pipeline_config"] = value.has_signal_pipeline_config;
  json["signal_pipeline_config"] = BuildJson(value.signal_pipeline_config);
  json["has_environment_runtime_config"] = value.has_environment_runtime_config;
  json["environment_runtime_config"] = BuildJson(value.environment_runtime_config);
  json["has_work_sub_mode"] = value.has_work_sub_mode;
  json["work_sub_mode"] = static_cast<int>(value.work_sub_mode);
  json["has_scan_center_deg"] = value.has_scan_center_deg;
  json["scan_center_deg"] = BuildJson(value.scan_center_deg);
  json["has_dwell_center_deg"] = value.has_dwell_center_deg;
  json["dwell_center_deg"] = BuildJson(value.dwell_center_deg);
  json["has_commanded_beamwidth_deg"] = value.has_commanded_beamwidth_deg;
  json["commanded_beamwidth_deg"] = BuildJson(value.commanded_beamwidth_deg);
  json["has_commanded_beamwidth_enabled"] = value.has_commanded_beamwidth_enabled;
  json["commanded_beamwidth_enabled"] = value.commanded_beamwidth_enabled;
  return json;
}

template <typename T>
std::string ToJson(const T& value) {
  return BuildJson(value).dump();
}

}  // namespace

RadarTraceSession::RadarTraceSession(const RadarSessionConfig& config,
                                     RadarTraceSessionOptions options)
    : session_(RadarSessionFactory::Create(config)), sink_(std::move(options.sink)) {
  if (sink_ && options.trace_config_on_construct) {
    Record("config", ToJson(config));
  }
}

output::TrackOutputFrame RadarTraceSession::Step(const RadarCycleInput& input) {
  if (sink_) {
    Record("input", ToJson(input));
  }
  const output::TrackOutputFrame output = session_.Step(input);
  if (sink_) {
    Record("output", ToJson(output));
  }
  return output;
}

output::TrackOutputFrame RadarTraceSession::Step(
    const RadarCycleInput& input, const environment::EnvironmentSceneState& scene_state) {
  if (sink_) {
    Json input_payload;
    input_payload["cycle_input"] = BuildJson(input);
    input_payload["scene_state"] = BuildJson(scene_state);
    Record("input", input_payload.dump());
  }
  const output::TrackOutputFrame output = session_.Step(input, scene_state);
  if (sink_) {
    Record("output", ToJson(output));
  }
  return output;
}

RadarCycleResult RadarTraceSession::StepWithResult(const RadarCycleInput& input) {
  if (sink_) {
    Record("input", ToJson(input));
  }
  const RadarCycleResult output = session_.StepWithResult(input);
  if (sink_) {
    Record("output", ToJson(output));
  }
  return output;
}

RadarCycleResult RadarTraceSession::StepWithResult(
    const RadarCycleInput& input, const environment::EnvironmentSceneState& scene_state) {
  if (sink_) {
    Json input_payload;
    input_payload["cycle_input"] = BuildJson(input);
    input_payload["scene_state"] = BuildJson(scene_state);
    Record("input", input_payload.dump());
  }
  const RadarCycleResult output = session_.StepWithResult(input, scene_state);
  if (sink_) {
    Record("output", ToJson(output));
  }
  return output;
}

void RadarTraceSession::ApplyRuntimeConfig(const config::RadarRuntimeConfigPatch& patch) {
  session_.ApplyRuntimeConfig(patch);
  if (sink_) {
    Record("runtime_config", ToJson(patch));
  }
}

const std::vector<extension::control::RadarCommand>& RadarTraceSession::GetSubmittedCommands() const {
  return session_.GetSubmittedCommands();
}

bool RadarTraceSession::HasLatestControlProfile() const { return session_.HasLatestControlProfile(); }

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

}  // namespace session
}  // namespace airborne_radar
