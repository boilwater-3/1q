#include "1q/electronic_surveillance_radar/session/EsrTraceSession.h"

#include <cstddef>
#include <string>

#include <nlohmann/json.hpp>

namespace electronic_surveillance_radar {
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

Json BuildJson(const oneq::common::Vector3f& value) {
  Json json = Json::array();
  json.push_back(value.x);
  json.push_back(value.y);
  json.push_back(value.z);
  return json;
}

Json BuildJson(const oneq::common::EulerAnglesDeg& value) {
  Json json;
  json["yaw_deg"] = value.yaw_deg;
  json["pitch_deg"] = value.pitch_deg;
  json["roll_deg"] = value.roll_deg;
  return json;
}

Json BuildJson(const model::EsrPoseState& value) {
  Json json;
  json["position_m"] = BuildJson(value.position_m);
  json["velocity_mps"] = BuildJson(value.velocity_mps);
  json["attitude_deg"] = BuildJson(value.attitude_deg);
  return json;
}

Json BuildJson(const model::EmitterBeamState& value) {
  Json json;
  json["center_az_deg"] = value.center_az_deg;
  json["center_el_deg"] = value.center_el_deg;
  json["az_beamwidth_deg"] = value.az_beamwidth_deg;
  json["el_beamwidth_deg"] = value.el_beamwidth_deg;
  json["beam_state_valid"] = value.beam_state_valid;
  return json;
}

Json BuildJson(const model::EmitterTruthState& value) {
  Json json;
  json["emitter_id"] = value.emitter_id;
  json["pose"] = BuildJson(value.pose);
  json["carrier_hz"] = value.carrier_hz;
  json["bandwidth_hz"] = value.bandwidth_hz;
  json["tx_power_w"] = value.tx_power_w;
  json["pulse_width_s"] = value.pulse_width_s;
  json["pri_s"] = value.pri_s;
  json["beam_state"] = BuildJson(value.beam_state);
  json["is_emitting"] = value.is_emitting;
  return json;
}

Json BuildJson(const environment::EsrJammerSource& value) {
  Json json;
  json["technique"] = static_cast<int>(value.technique);
  json["active"] = value.active;
  json["center_hz"] = value.center_hz;
  json["bandwidth_hz"] = value.bandwidth_hz;
  json["power_w"] = value.power_w;
  json["deception_risk"] = value.deception_risk;
  json["confidence"] = value.confidence;
  return json;
}

Json BuildJson(const environment::EsrAtmosphericPhysicsConfig& value) {
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

Json BuildJson(const environment::EsrEnvironmentSceneState& value) {
  Json json;
  json["base_propagation_loss_db"] = value.base_propagation_loss_db;
  json["atmospheric_attenuation_db"] = value.atmospheric_attenuation_db;
  json["terrain_reflection_db"] = value.terrain_reflection_db;
  json["clutter_noise_w"] = value.clutter_noise_w;
  json["spectrum_occupancy_ratio"] = value.spectrum_occupancy_ratio;
  json["atmospheric_physics"] = BuildJson(value.atmospheric_physics);
  json["jammer_sources"] =
      SerializeArray(value.jammer_sources, [](const environment::EsrJammerSource& source) {
        return BuildJson(source);
      });
  return json;
}

Json BuildJson(const session::EsrCycleInput& value) {
  Json json;
  json["cycle_index"] = value.cycle_index;
  json["dt_sec"] = value.dt_sec;
  json["platform_pose"] = BuildJson(value.platform_pose);
  json["scene_emitters"] =
      SerializeArray(value.scene_emitters, [](const model::EmitterTruthState& emitter) {
        return BuildJson(emitter);
      });
  json["environment_scene_state"] = BuildJson(value.environment_scene_state);
  return json;
}

Json BuildJson(const session::EsrValidationIssue& value) {
  Json json;
  json["severity"] = static_cast<int>(value.severity);
  json["code"] = static_cast<int>(value.code);
  json["emitter_index"] = value.emitter_index;
  json["message"] = value.message;
  return json;
}

Json BuildJson(const model::EmitterObservation& value) {
  Json json;
  json["observation_id"] = value.observation_id;
  json["timestamp_s"] = value.timestamp_s;
  json["aoa_az_deg"] = value.aoa_az_deg;
  json["aoa_el_deg"] = value.aoa_el_deg;
  json["rf_hz"] = value.rf_hz;
  json["pulse_width_s"] = value.pulse_width_s;
  json["amplitude_db"] = value.amplitude_db;
  json["snr_db"] = value.snr_db;
  json["quality"] = static_cast<int>(value.quality);
  json["is_jammed"] = value.is_jammed;
  return json;
}

Json BuildJson(const model::EmitterHypothesis& value) {
  Json json;
  json["hypothesis_id"] = value.hypothesis_id;
  json["candidate_classes"] =
      SerializeArray(value.candidate_classes, [](const std::string& name) { return Json(name); });
  json["mode"] = static_cast<int>(value.mode);
  json["threat_level"] = static_cast<int>(value.threat_level);
  json["bearing_az_deg"] = value.bearing_az_deg;
  json["bearing_el_deg"] = value.bearing_el_deg;
  json["bearing_std_deg"] = value.bearing_std_deg;
  json["confidence"] = value.confidence;
  json["last_seen_cycle"] = value.last_seen_cycle;
  return json;
}

Json BuildJson(const output::TruthAssociationRecord& value) {
  Json json;
  json["observation_id"] = value.observation_id;
  json["truth_emitter_id"] = value.truth_emitter_id;
  json["matched"] = value.matched;
  json["confidence"] = value.confidence;
  return json;
}

Json BuildJson(const output::EsrOutputFrame& value) {
  Json json;

  Json observation_output;
  observation_output["cycle_index"] = value.observation_output.cycle_index;
  observation_output["batch_id"] = value.observation_output.batch_id;
  observation_output["observations"] = SerializeArray(
      value.observation_output.observations,
      [](const model::EmitterObservation& observation) { return BuildJson(observation); });
  json["observation_output"] = observation_output;

  Json emitter_output;
  emitter_output["cycle_index"] = value.emitter_output.cycle_index;
  emitter_output["batch_id"] = value.emitter_output.batch_id;
  emitter_output["hypotheses"] = SerializeArray(
      value.emitter_output.hypotheses,
      [](const model::EmitterHypothesis& hypothesis) { return BuildJson(hypothesis); });
  json["emitter_output"] = emitter_output;

  Json truth_output;
  truth_output["cycle_index"] = value.truth_evaluation_output.cycle_index;
  truth_output["batch_id"] = value.truth_evaluation_output.batch_id;
  truth_output["matched_count"] = value.truth_evaluation_output.matched_count;
  truth_output["total_observation_count"] = value.truth_evaluation_output.total_observation_count;
  truth_output["associations"] = SerializeArray(
      value.truth_evaluation_output.associations,
      [](const output::TruthAssociationRecord& association) { return BuildJson(association); });
  json["truth_evaluation_output"] = truth_output;

  return json;
}

Json BuildJson(const extension::InterceptPipelineConfig& value) {
  Json json;

  Json detection;
  detection["receiver_noise_floor_w"] = value.detection.receiver_noise_floor_w;
  detection["min_detect_snr_db"] = value.detection.min_detect_snr_db;
  detection["max_detect_range_m"] = value.detection.max_detect_range_m;
  detection["min_dynamic_range_margin_db"] = value.detection.min_dynamic_range_margin_db;
  detection["boundary_resolution_m"] = value.detection.boundary_resolution_m;
  detection["boundary_max_iterations"] = value.detection.boundary_max_iterations;
  json["detection"] = detection;

  Json statistical_detection;
  statistical_detection["pfa"] = value.statistical_detection.pfa;
  statistical_detection["min_snr_db"] = value.statistical_detection.min_snr_db;
  statistical_detection["pulse_count"] = value.statistical_detection.pulse_count;
  statistical_detection["integration_mode"] =
      static_cast<int>(value.statistical_detection.integration_mode);
  statistical_detection["threshold_scale"] = value.statistical_detection.threshold_scale;
  statistical_detection["enable_statistical_detection"] =
      value.statistical_detection.enable_statistical_detection;
  json["statistical_detection"] = statistical_detection;

  Json scan;
  scan["scan_start_az_deg"] = value.scan.scan_start_az_deg;
  scan["scan_end_az_deg"] = value.scan.scan_end_az_deg;
  scan["scan_start_el_deg"] = value.scan.scan_start_el_deg;
  scan["scan_end_el_deg"] = value.scan.scan_end_el_deg;
  scan["az_step_deg"] = value.scan.az_step_deg;
  scan["el_step_deg"] = value.scan.el_step_deg;
  scan["scan_start_pos"] = value.scan.scan_start_pos;
  scan["scan_sequence"] = value.scan.scan_sequence;
  json["scan"] = scan;

  Json algorithm;
  algorithm["random_seed"] = value.algorithm.random_seed;
  algorithm["angle_error_coefficient"] = value.algorithm.angle_error_coefficient;
  json["algorithm"] = algorithm;

  Json preprocess;
  preprocess["dedup_time_window_sec"] = value.preprocess.dedup_time_window_sec;
  preprocess["dedup_rf_window_hz"] = value.preprocess.dedup_rf_window_hz;
  preprocess["dedup_pw_window_sec"] = value.preprocess.dedup_pw_window_sec;
  preprocess["dedup_az_window_deg"] = value.preprocess.dedup_az_window_deg;
  preprocess["dedup_el_window_deg"] = value.preprocess.dedup_el_window_deg;
  preprocess["normalize_quality"] = value.preprocess.normalize_quality;
  json["preprocess"] = preprocess;

  Json cluster;
  cluster["radius"] = value.cluster.radius;
  cluster["min_points"] = value.cluster.min_points;
  cluster["rf_scale_hz"] = value.cluster.rf_scale_hz;
  cluster["pw_scale_sec"] = value.cluster.pw_scale_sec;
  cluster["az_scale_deg"] = value.cluster.az_scale_deg;
  cluster["el_scale_deg"] = value.cluster.el_scale_deg;
  cluster["snr_scale_db"] = value.cluster.snr_scale_db;
  json["cluster"] = cluster;

  Json association;
  association["gate_distance"] = value.association.gate_distance;
  association["confirm_hits"] = value.association.confirm_hits;
  association["max_missed_cycles"] = value.association.max_missed_cycles;
  association["confidence_alpha"] = value.association.confidence_alpha;
  association["output_tentative"] = value.association.output_tentative;
  json["association"] = association;

  Json suppression_model;
  suppression_model["suppression_noise_scale"] = value.suppression_model.suppression_noise_scale;
  suppression_model["suppression_mark_threshold_w"] =
      value.suppression_model.suppression_mark_threshold_w;
  json["suppression_model"] = suppression_model;

  Json deception_model;
  deception_model["false_alarm_probability_scale"] =
      value.deception_model.false_alarm_probability_scale;
  deception_model["confusion_probability_scale"] = value.deception_model.confusion_probability_scale;
  deception_model["max_false_observations_per_emitter"] =
      value.deception_model.max_false_observations_per_emitter;
  deception_model["aoa_confusion_std_deg"] = value.deception_model.aoa_confusion_std_deg;
  deception_model["rf_confusion_ratio"] = value.deception_model.rf_confusion_ratio;
  deception_model["pw_confusion_ratio"] = value.deception_model.pw_confusion_ratio;
  deception_model["cluster_confidence_penalty_scale"] =
      value.deception_model.cluster_confidence_penalty_scale;
  json["deception_model"] = deception_model;

  return json;
}

Json BuildJson(const session::EsrSessionConfig& value) {
  Json json;
  json["enable_layered_config"] = value.enable_layered_config;

  Json hardware;
  hardware["receiver_band_lower_hz"] = value.layered_config.hardware.receiver_band_lower_hz;
  hardware["receiver_band_upper_hz"] = value.layered_config.hardware.receiver_band_upper_hz;
  hardware["receiver_sensitivity_w"] = value.layered_config.hardware.receiver_sensitivity_w;
  hardware["integrated_receive_loss_db"] = value.layered_config.hardware.integrated_receive_loss_db;
  hardware["beam_az_width_deg"] = value.layered_config.hardware.beam_az_width_deg;
  hardware["beam_el_width_deg"] = value.layered_config.hardware.beam_el_width_deg;
  hardware["az_scan_range_deg"] = value.layered_config.hardware.az_scan_range_deg;
  hardware["el_scan_range_deg"] = value.layered_config.hardware.el_scan_range_deg;
  hardware["antenna_mount_az_deg"] = value.layered_config.hardware.antenna_mount_az_deg;
  hardware["antenna_mount_el_deg"] = value.layered_config.hardware.antenna_mount_el_deg;

  Json mission;
  mission["power_on"] = value.layered_config.mission.power_on;
  mission["work_mode"] = static_cast<int>(value.layered_config.mission.work_mode);
  mission["scan_center_az_deg"] = value.layered_config.mission.scan_center_az_deg;
  mission["scan_center_el_deg"] = value.layered_config.mission.scan_center_el_deg;
  mission["scan_rate_hz"] = value.layered_config.mission.scan_rate_hz;
  mission["scan_start_position"] =
      static_cast<int>(value.layered_config.mission.scan_start_position);
  mission["scan_sequence"] = static_cast<int>(value.layered_config.mission.scan_sequence);
  mission["use_explicit_scan_bounds"] = value.layered_config.mission.use_explicit_scan_bounds;
  mission["scan_start_az_deg"] = value.layered_config.mission.scan_start_az_deg;
  mission["scan_end_az_deg"] = value.layered_config.mission.scan_end_az_deg;
  mission["scan_start_el_deg"] = value.layered_config.mission.scan_start_el_deg;
  mission["scan_end_el_deg"] = value.layered_config.mission.scan_end_el_deg;

  Json layered;
  layered["hardware"] = hardware;
  layered["mission"] = mission;
  json["layered_config"] = layered;

  json["pipeline_config"] = BuildJson(value.pipeline_config);

  Json environment_default_config;
  environment_default_config["model_config"] = {
      {"default_clutter_noise_w",
       value.environment_default_config.model_config.default_clutter_noise_w},
      {"jamming_detection_threshold_w",
       value.environment_default_config.model_config.jamming_detection_threshold_w},
      {"atmospheric_physics",
       BuildJson(value.environment_default_config.model_config.atmospheric_physics)}};
  json["environment_default_config"] = environment_default_config;

  return json;
}

Json BuildJson(const session::EsrCycleResult& value) {
  Json json;
  json["output_frame"] = BuildJson(value.output_frame);
  json["validation_issues"] =
      SerializeArray(value.validation_issues, [](const session::EsrValidationIssue& issue) {
        return BuildJson(issue);
      });
  json["has_validation_error"] = value.has_validation_error;
  return json;
}

Json BuildJson(const session::EsrRuntimeConfigPatch& value) {
  Json json;
  json["has_sensor_enabled"] = value.has_sensor_enabled;
  json["sensor_enabled"] = value.sensor_enabled;
  json["has_scan_rate_hz"] = value.has_scan_rate_hz;
  json["scan_rate_hz"] = value.scan_rate_hz;
  json["has_integrated_receive_loss_db"] = value.has_integrated_receive_loss_db;
  json["integrated_receive_loss_db"] = value.integrated_receive_loss_db;
  json["has_fixed_receiver_window_hz"] = value.has_fixed_receiver_window_hz;
  json["receiver_lower_hz"] = value.receiver_lower_hz;
  json["receiver_upper_hz"] = value.receiver_upper_hz;
  json["has_use_fixed_receiver_window"] = value.has_use_fixed_receiver_window;
  json["use_fixed_receiver_window"] = value.use_fixed_receiver_window;
  json["has_enable_statistical_detection"] = value.has_enable_statistical_detection;
  json["enable_statistical_detection"] = value.enable_statistical_detection;
  json["has_enable_spectral_analysis"] = value.has_enable_spectral_analysis;
  json["enable_spectral_analysis"] = value.enable_spectral_analysis;
  json["has_detection_min_snr_db"] = value.has_detection_min_snr_db;
  json["detection_min_snr_db"] = value.detection_min_snr_db;
  json["has_environment_runtime_config"] = value.has_environment_runtime_config;
  json["environment_runtime_config"] = {
      {"has_model_config", value.environment_runtime_config.has_model_config},
      {"model_config",
       {
           {"default_clutter_noise_w",
            value.environment_runtime_config.model_config.default_clutter_noise_w},
           {"jamming_detection_threshold_w",
            value.environment_runtime_config.model_config.jamming_detection_threshold_w},
           {"atmospheric_physics",
            BuildJson(value.environment_runtime_config.model_config.atmospheric_physics)},
       }},
      {"has_jamming_detection_threshold_w",
       value.environment_runtime_config.has_jamming_detection_threshold_w},
      {"jamming_detection_threshold_w",
       value.environment_runtime_config.jamming_detection_threshold_w}};
  json["has_observation_jam_mark_threshold_w"] = value.has_observation_jam_mark_threshold_w;
  json["observation_jam_mark_threshold_w"] = value.observation_jam_mark_threshold_w;
  return json;
}

template <typename T>
std::string ToJson(const T& value) {
  return BuildJson(value).dump();
}

}  // namespace

EsrTraceSession::EsrTraceSession(session::EsrSessionConfig config,
                                 EsrTraceSessionOptions options)
    : session_(config), sink_(std::move(options.sink)) {
  if (sink_ && options.trace_config_on_construct) {
    Record("config", ToJson(config));
  }
}

output::EsrOutputFrame EsrTraceSession::Step(const session::EsrCycleInput& input) {
  if (sink_) {
    Record("input", ToJson(input));
  }
  const output::EsrOutputFrame output = session_.Step(input);
  if (sink_) {
    Record("output", ToJson(output));
  }
  return output;
}

session::EsrCycleResult EsrTraceSession::StepWithResult(
    const session::EsrCycleInput& input) {
  if (sink_) {
    Record("input", ToJson(input));
  }
  const session::EsrCycleResult output = session_.StepWithResult(input);
  if (sink_) {
    Record("output", ToJson(output));
  }
  return output;
}

void EsrTraceSession::ApplyRuntimeConfig(const session::EsrRuntimeConfigPatch& patch) {
  session_.ApplyRuntimeConfig(patch);
  if (sink_) {
    Record("runtime_config_patch", ToJson(patch));
  }
}

session::EsrSession& EsrTraceSession::session() { return session_; }

const session::EsrSession& EsrTraceSession::session() const { return session_; }

void EsrTraceSession::Record(const std::string& phase, const std::string& payload_json) const {
  sink_->Record("electronic_surveillance_radar", phase, payload_json);
}

}  // namespace session
}  // namespace electronic_surveillance_radar
