#include "1q/electronic_surveillance_radar/tools/EsrTraceSession.h"

#include <sstream>
#include <string>

#include "common/trace/JsonFormatUtils.h"

namespace electronic_surveillance_radar {
namespace tools {
namespace {

using oneq::common::trace::internal::BoolToJson;
using oneq::common::trace::internal::QuoteString;

std::string ToJson(const oneq::common::Vector3f& value) {
  std::ostringstream stream;
  stream << "[" << value.x << "," << value.y << "," << value.z << "]";
  return stream.str();
}

std::string ToJson(const oneq::common::EulerAnglesDeg& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"yaw_deg\":" << value.yaw_deg << ",";
  stream << "\"pitch_deg\":" << value.pitch_deg << ",";
  stream << "\"roll_deg\":" << value.roll_deg;
  stream << "}";
  return stream.str();
}

std::string ToJson(const common::EsrPoseState& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"position_m\":" << ToJson(value.position_m) << ",";
  stream << "\"velocity_mps\":" << ToJson(value.velocity_mps) << ",";
  stream << "\"attitude_deg\":" << ToJson(value.attitude_deg);
  stream << "}";
  return stream.str();
}

std::string ToJson(const common::EmitterBeamState& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"center_az_deg\":" << value.center_az_deg << ",";
  stream << "\"center_el_deg\":" << value.center_el_deg << ",";
  stream << "\"az_beamwidth_deg\":" << value.az_beamwidth_deg << ",";
  stream << "\"el_beamwidth_deg\":" << value.el_beamwidth_deg << ",";
  stream << "\"beam_state_valid\":" << BoolToJson(value.beam_state_valid);
  stream << "}";
  return stream.str();
}

std::string ToJson(const common::EmitterTruthState& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"emitter_id\":" << QuoteString(value.emitter_id) << ",";
  stream << "\"pose\":" << ToJson(value.pose) << ",";
  stream << "\"carrier_hz\":" << value.carrier_hz << ",";
  stream << "\"bandwidth_hz\":" << value.bandwidth_hz << ",";
  stream << "\"tx_power_w\":" << value.tx_power_w << ",";
  stream << "\"pulse_width_s\":" << value.pulse_width_s << ",";
  stream << "\"pri_s\":" << value.pri_s << ",";
  stream << "\"beam_state\":" << ToJson(value.beam_state) << ",";
  stream << "\"is_emitting\":" << BoolToJson(value.is_emitting);
  stream << "}";
  return stream.str();
}

std::string ToJson(const environment::EsrJammerSource& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"technique\":" << static_cast<int>(value.technique) << ",";
  stream << "\"active\":" << BoolToJson(value.active) << ",";
  stream << "\"center_hz\":" << value.center_hz << ",";
  stream << "\"bandwidth_hz\":" << value.bandwidth_hz << ",";
  stream << "\"power_w\":" << value.power_w << ",";
  stream << "\"deception_risk\":" << value.deception_risk << ",";
  stream << "\"confidence\":" << value.confidence;
  stream << "}";
  return stream.str();
}

std::string ToJson(const environment::EsrAtmosphericPhysicsConfig& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"enable_physical_model\":" << BoolToJson(value.enable_physical_model) << ",";
  stream << "\"frequency_hz\":" << value.frequency_hz << ",";
  stream << "\"path_length_m\":" << value.path_length_m << ",";
  stream << "\"radar_altitude_m\":" << value.radar_altitude_m << ",";
  stream << "\"target_altitude_m\":" << value.target_altitude_m << ",";
  stream << "\"elevation_deg\":" << value.elevation_deg << ",";
  stream << "\"pressure_hpa\":" << value.pressure_hpa << ",";
  stream << "\"temperature_k\":" << value.temperature_k << ",";
  stream << "\"relative_humidity\":" << value.relative_humidity << ",";
  stream << "\"k_factor\":" << value.k_factor << ",";
  stream << "\"day_of_year\":" << value.day_of_year << ",";
  stream << "\"solar_flux_f107a\":" << value.solar_flux_f107a << ",";
  stream << "\"solar_flux_f107\":" << value.solar_flux_f107 << ",";
  stream << "\"geomagnetic_ap\":" << value.geomagnetic_ap;
  stream << "}";
  return stream.str();
}

std::string ToJson(const environment::EsrEnvironmentSceneState& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"base_propagation_loss_db\":" << value.base_propagation_loss_db << ",";
  stream << "\"atmospheric_attenuation_db\":" << value.atmospheric_attenuation_db << ",";
  stream << "\"terrain_reflection_db\":" << value.terrain_reflection_db << ",";
  stream << "\"clutter_noise_w\":" << value.clutter_noise_w << ",";
  stream << "\"spectrum_occupancy_ratio\":" << value.spectrum_occupancy_ratio << ",";
  stream << "\"atmospheric_physics\":" << ToJson(value.atmospheric_physics) << ",";
  stream << "\"jammer_sources\":[";
  for (std::size_t i = 0; i < value.jammer_sources.size(); ++i) {
    if (i > 0U) {
      stream << ",";
    }
    stream << ToJson(value.jammer_sources[i]);
  }
  stream << "]";
  stream << "}";
  return stream.str();
}

std::string ToJson(const core::context::EsrCycleInput& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"cycle_index\":" << value.cycle_index << ",";
  stream << "\"dt_sec\":" << value.dt_sec << ",";
  stream << "\"platform_pose\":" << ToJson(value.platform_pose) << ",";
  stream << "\"scene_emitters\":[";
  for (std::size_t i = 0; i < value.scene_emitters.size(); ++i) {
    if (i > 0U) {
      stream << ",";
    }
    stream << ToJson(value.scene_emitters[i]);
  }
  stream << "],";
  stream << "\"environment_scene_state\":" << ToJson(value.environment_scene_state);
  stream << "}";
  return stream.str();
}

std::string ToJson(const core::context::EsrValidationIssue& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"severity\":" << static_cast<int>(value.severity) << ",";
  stream << "\"code\":" << static_cast<int>(value.code) << ",";
  stream << "\"emitter_index\":" << value.emitter_index << ",";
  stream << "\"message\":" << QuoteString(value.message);
  stream << "}";
  return stream.str();
}

std::string ToJson(const common::EmitterObservation& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"observation_id\":" << value.observation_id << ",";
  stream << "\"timestamp_s\":" << value.timestamp_s << ",";
  stream << "\"aoa_az_deg\":" << value.aoa_az_deg << ",";
  stream << "\"aoa_el_deg\":" << value.aoa_el_deg << ",";
  stream << "\"rf_hz\":" << value.rf_hz << ",";
  stream << "\"pulse_width_s\":" << value.pulse_width_s << ",";
  stream << "\"amplitude_db\":" << value.amplitude_db << ",";
  stream << "\"snr_db\":" << value.snr_db << ",";
  stream << "\"quality\":" << static_cast<int>(value.quality) << ",";
  stream << "\"is_jammed\":" << BoolToJson(value.is_jammed);
  stream << "}";
  return stream.str();
}

std::string ToJson(const common::EmitterHypothesis& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"hypothesis_id\":" << value.hypothesis_id << ",";
  stream << "\"candidate_classes\":[";
  for (std::size_t i = 0; i < value.candidate_classes.size(); ++i) {
    if (i > 0U) {
      stream << ",";
    }
    stream << QuoteString(value.candidate_classes[i]);
  }
  stream << "],";
  stream << "\"mode\":" << static_cast<int>(value.mode) << ",";
  stream << "\"threat_level\":" << static_cast<int>(value.threat_level) << ",";
  stream << "\"bearing_az_deg\":" << value.bearing_az_deg << ",";
  stream << "\"bearing_el_deg\":" << value.bearing_el_deg << ",";
  stream << "\"bearing_std_deg\":" << value.bearing_std_deg << ",";
  stream << "\"confidence\":" << value.confidence << ",";
  stream << "\"last_seen_cycle\":" << value.last_seen_cycle;
  stream << "}";
  return stream.str();
}

std::string ToJson(const common::TruthAssociationRecord& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"observation_id\":" << value.observation_id << ",";
  stream << "\"truth_emitter_id\":" << QuoteString(value.truth_emitter_id) << ",";
  stream << "\"matched\":" << BoolToJson(value.matched) << ",";
  stream << "\"confidence\":" << value.confidence;
  stream << "}";
  return stream.str();
}

std::string ToJson(const common::EsrOutputFrame& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"observation_output\":{";
  stream << "\"cycle_index\":" << value.observation_output.cycle_index << ",";
  stream << "\"batch_id\":" << value.observation_output.batch_id << ",";
  stream << "\"observations\":[";
  for (std::size_t i = 0; i < value.observation_output.observations.size(); ++i) {
    if (i > 0U) {
      stream << ",";
    }
    stream << ToJson(value.observation_output.observations[i]);
  }
  stream << "]},";
  stream << "\"emitter_output\":{";
  stream << "\"cycle_index\":" << value.emitter_output.cycle_index << ",";
  stream << "\"batch_id\":" << value.emitter_output.batch_id << ",";
  stream << "\"hypotheses\":[";
  for (std::size_t i = 0; i < value.emitter_output.hypotheses.size(); ++i) {
    if (i > 0U) {
      stream << ",";
    }
    stream << ToJson(value.emitter_output.hypotheses[i]);
  }
  stream << "]},";
  stream << "\"truth_evaluation_output\":{";
  stream << "\"cycle_index\":" << value.truth_evaluation_output.cycle_index << ",";
  stream << "\"batch_id\":" << value.truth_evaluation_output.batch_id << ",";
  stream << "\"matched_count\":" << value.truth_evaluation_output.matched_count << ",";
  stream << "\"total_observation_count\":"
         << value.truth_evaluation_output.total_observation_count << ",";
  stream << "\"associations\":[";
  for (std::size_t i = 0; i < value.truth_evaluation_output.associations.size(); ++i) {
    if (i > 0U) {
      stream << ",";
    }
    stream << ToJson(value.truth_evaluation_output.associations[i]);
  }
  stream << "]}";
  stream << "}";
  return stream.str();
}

std::string ToJson(const pipeline::InterceptPipelineConfig& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"detection\":{";
  stream << "\"receiver_noise_floor_w\":" << value.detection.receiver_noise_floor_w << ",";
  stream << "\"min_detect_snr_db\":" << value.detection.min_detect_snr_db << ",";
  stream << "\"max_detect_range_m\":" << value.detection.max_detect_range_m << ",";
  stream << "\"min_dynamic_range_margin_db\":" << value.detection.min_dynamic_range_margin_db
         << ",";
  stream << "\"boundary_resolution_m\":" << value.detection.boundary_resolution_m << ",";
  stream << "\"boundary_max_iterations\":" << value.detection.boundary_max_iterations;
  stream << "},";
  stream << "\"statistical_detection\":{";
  stream << "\"pfa\":" << value.statistical_detection.pfa << ",";
  stream << "\"min_snr_db\":" << value.statistical_detection.min_snr_db << ",";
  stream << "\"pulse_count\":" << value.statistical_detection.pulse_count << ",";
  stream << "\"integration_mode\":" << static_cast<int>(value.statistical_detection.integration_mode)
         << ",";
  stream << "\"threshold_scale\":" << value.statistical_detection.threshold_scale << ",";
  stream << "\"enable_statistical_detection\":"
         << BoolToJson(value.statistical_detection.enable_statistical_detection);
  stream << "},";
  stream << "\"scan\":{";
  stream << "\"scan_start_az_deg\":" << value.scan.scan_start_az_deg << ",";
  stream << "\"scan_end_az_deg\":" << value.scan.scan_end_az_deg << ",";
  stream << "\"scan_start_el_deg\":" << value.scan.scan_start_el_deg << ",";
  stream << "\"scan_end_el_deg\":" << value.scan.scan_end_el_deg << ",";
  stream << "\"az_step_deg\":" << value.scan.az_step_deg << ",";
  stream << "\"el_step_deg\":" << value.scan.el_step_deg << ",";
  stream << "\"scan_start_pos\":" << value.scan.scan_start_pos << ",";
  stream << "\"scan_sequence\":" << value.scan.scan_sequence;
  stream << "},";
  stream << "\"algorithm\":{";
  stream << "\"random_seed\":" << value.algorithm.random_seed << ",";
  stream << "\"angle_error_coefficient\":" << value.algorithm.angle_error_coefficient;
  stream << "},";
  stream << "\"preprocess\":{";
  stream << "\"dedup_time_window_sec\":" << value.preprocess.dedup_time_window_sec << ",";
  stream << "\"dedup_rf_window_hz\":" << value.preprocess.dedup_rf_window_hz << ",";
  stream << "\"dedup_pw_window_sec\":" << value.preprocess.dedup_pw_window_sec << ",";
  stream << "\"dedup_az_window_deg\":" << value.preprocess.dedup_az_window_deg << ",";
  stream << "\"dedup_el_window_deg\":" << value.preprocess.dedup_el_window_deg << ",";
  stream << "\"normalize_quality\":" << BoolToJson(value.preprocess.normalize_quality);
  stream << "},";
  stream << "\"cluster\":{";
  stream << "\"radius\":" << value.cluster.radius << ",";
  stream << "\"min_points\":" << value.cluster.min_points << ",";
  stream << "\"rf_scale_hz\":" << value.cluster.rf_scale_hz << ",";
  stream << "\"pw_scale_sec\":" << value.cluster.pw_scale_sec << ",";
  stream << "\"az_scale_deg\":" << value.cluster.az_scale_deg << ",";
  stream << "\"el_scale_deg\":" << value.cluster.el_scale_deg << ",";
  stream << "\"snr_scale_db\":" << value.cluster.snr_scale_db;
  stream << "},";
  stream << "\"association\":{";
  stream << "\"gate_distance\":" << value.association.gate_distance << ",";
  stream << "\"confirm_hits\":" << value.association.confirm_hits << ",";
  stream << "\"max_missed_cycles\":" << value.association.max_missed_cycles << ",";
  stream << "\"confidence_alpha\":" << value.association.confidence_alpha << ",";
  stream << "\"output_tentative\":" << BoolToJson(value.association.output_tentative);
  stream << "},";
  stream << "\"suppression_model\":{";
  stream << "\"suppression_noise_scale\":" << value.suppression_model.suppression_noise_scale
         << ",";
  stream << "\"suppression_mark_threshold_w\":"
         << value.suppression_model.suppression_mark_threshold_w;
  stream << "},";
  stream << "\"deception_model\":{";
  stream << "\"false_alarm_probability_scale\":"
         << value.deception_model.false_alarm_probability_scale << ",";
  stream << "\"confusion_probability_scale\":"
         << value.deception_model.confusion_probability_scale << ",";
  stream << "\"max_false_observations_per_emitter\":"
         << value.deception_model.max_false_observations_per_emitter << ",";
  stream << "\"aoa_confusion_std_deg\":" << value.deception_model.aoa_confusion_std_deg << ",";
  stream << "\"rf_confusion_ratio\":" << value.deception_model.rf_confusion_ratio << ",";
  stream << "\"pw_confusion_ratio\":" << value.deception_model.pw_confusion_ratio << ",";
  stream << "\"cluster_confidence_penalty_scale\":"
         << value.deception_model.cluster_confidence_penalty_scale;
  stream << "}";
  stream << "}";
  return stream.str();
}

std::string ToJson(const core::session::EsrSessionConfig& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"enable_layered_config\":" << BoolToJson(value.enable_layered_config) << ",";
  stream << "\"layered_config\":{";
  stream << "\"hardware\":{";
  stream << "\"receiver_band_lower_hz\":" << value.layered_config.hardware.receiver_band_lower_hz
         << ",";
  stream << "\"receiver_band_upper_hz\":" << value.layered_config.hardware.receiver_band_upper_hz
         << ",";
  stream << "\"receiver_sensitivity_w\":" << value.layered_config.hardware.receiver_sensitivity_w
         << ",";
  stream << "\"integrated_receive_loss_db\":"
         << value.layered_config.hardware.integrated_receive_loss_db << ",";
  stream << "\"beam_az_width_deg\":" << value.layered_config.hardware.beam_az_width_deg << ",";
  stream << "\"beam_el_width_deg\":" << value.layered_config.hardware.beam_el_width_deg << ",";
  stream << "\"az_scan_range_deg\":" << value.layered_config.hardware.az_scan_range_deg << ",";
  stream << "\"el_scan_range_deg\":" << value.layered_config.hardware.el_scan_range_deg << ",";
  stream << "\"antenna_mount_az_deg\":" << value.layered_config.hardware.antenna_mount_az_deg
         << ",";
  stream << "\"antenna_mount_el_deg\":" << value.layered_config.hardware.antenna_mount_el_deg;
  stream << "},";
  stream << "\"mission\":{";
  stream << "\"power_on\":" << BoolToJson(value.layered_config.mission.power_on) << ",";
  stream << "\"work_mode\":" << static_cast<int>(value.layered_config.mission.work_mode) << ",";
  stream << "\"scan_center_az_deg\":" << value.layered_config.mission.scan_center_az_deg << ",";
  stream << "\"scan_center_el_deg\":" << value.layered_config.mission.scan_center_el_deg << ",";
  stream << "\"scan_rate_hz\":" << value.layered_config.mission.scan_rate_hz << ",";
  stream << "\"scan_start_position\":"
         << static_cast<int>(value.layered_config.mission.scan_start_position) << ",";
  stream << "\"scan_sequence\":" << static_cast<int>(value.layered_config.mission.scan_sequence)
         << ",";
  stream << "\"use_explicit_scan_bounds\":"
         << BoolToJson(value.layered_config.mission.use_explicit_scan_bounds) << ",";
  stream << "\"scan_start_az_deg\":" << value.layered_config.mission.scan_start_az_deg << ",";
  stream << "\"scan_end_az_deg\":" << value.layered_config.mission.scan_end_az_deg << ",";
  stream << "\"scan_start_el_deg\":" << value.layered_config.mission.scan_start_el_deg << ",";
  stream << "\"scan_end_el_deg\":" << value.layered_config.mission.scan_end_el_deg;
  stream << "}},";
  stream << "\"pipeline_config\":" << ToJson(value.pipeline_config) << ",";
  stream << "\"environment_config\":{";
  stream << "\"default_clutter_noise_w\":" << value.environment_config.default_clutter_noise_w
         << ",";
  stream << "\"jamming_detection_threshold_w\":"
         << value.environment_config.jamming_detection_threshold_w << ",";
  stream << "\"atmospheric_physics\":" << ToJson(value.environment_config.atmospheric_physics);
  stream << "}";
  stream << "}";
  return stream.str();
}

std::string ToJson(const core::session::EsrCycleResult& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"output_frame\":" << ToJson(value.output_frame) << ",";
  stream << "\"validation_issues\":[";
  for (std::size_t i = 0; i < value.validation_issues.size(); ++i) {
    if (i > 0U) {
      stream << ",";
    }
    stream << ToJson(value.validation_issues[i]);
  }
  stream << "],";
  stream << "\"has_validation_error\":" << BoolToJson(value.has_validation_error);
  stream << "}";
  return stream.str();
}

std::string ToJson(const core::session::EsrRuntimeConfigPatch& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"has_sensor_enabled\":" << BoolToJson(value.has_sensor_enabled) << ",";
  stream << "\"sensor_enabled\":" << BoolToJson(value.sensor_enabled) << ",";
  stream << "\"has_scan_rate_hz\":" << BoolToJson(value.has_scan_rate_hz) << ",";
  stream << "\"scan_rate_hz\":" << value.scan_rate_hz << ",";
  stream << "\"has_integrated_receive_loss_db\":"
         << BoolToJson(value.has_integrated_receive_loss_db) << ",";
  stream << "\"integrated_receive_loss_db\":" << value.integrated_receive_loss_db << ",";
  stream << "\"has_fixed_receiver_window_hz\":"
         << BoolToJson(value.has_fixed_receiver_window_hz) << ",";
  stream << "\"receiver_lower_hz\":" << value.receiver_lower_hz << ",";
  stream << "\"receiver_upper_hz\":" << value.receiver_upper_hz << ",";
  stream << "\"has_use_fixed_receiver_window\":"
         << BoolToJson(value.has_use_fixed_receiver_window) << ",";
  stream << "\"use_fixed_receiver_window\":" << BoolToJson(value.use_fixed_receiver_window)
         << ",";
  stream << "\"has_enable_statistical_detection\":"
         << BoolToJson(value.has_enable_statistical_detection) << ",";
  stream << "\"enable_statistical_detection\":"
         << BoolToJson(value.enable_statistical_detection) << ",";
  stream << "\"has_enable_spectral_analysis\":"
         << BoolToJson(value.has_enable_spectral_analysis) << ",";
  stream << "\"enable_spectral_analysis\":" << BoolToJson(value.enable_spectral_analysis) << ",";
  stream << "\"has_detection_min_snr_db\":" << BoolToJson(value.has_detection_min_snr_db) << ",";
  stream << "\"detection_min_snr_db\":" << value.detection_min_snr_db << ",";
  stream << "\"has_jamming_detection_threshold_w\":"
         << BoolToJson(value.has_jamming_detection_threshold_w) << ",";
  stream << "\"jamming_detection_threshold_w\":" << value.jamming_detection_threshold_w;
  stream << "}";
  return stream.str();
}

}  // namespace

EsrTraceSession::EsrTraceSession(core::session::EsrSessionConfig config,
                                 EsrTraceSessionOptions options)
    : session_(config), sink_(std::move(options.sink)) {
  if (sink_ && options.trace_config_on_construct) {
    Record("config", ToJson(config));
  }
}

common::EsrOutputFrame EsrTraceSession::Step(const core::context::EsrCycleInput& input) {
  if (sink_) {
    Record("input", ToJson(input));
  }
  const common::EsrOutputFrame output = session_.Step(input);
  if (sink_) {
    Record("output", ToJson(output));
  }
  return output;
}

core::session::EsrCycleResult EsrTraceSession::StepWithResult(const core::context::EsrCycleInput& input) {
  if (sink_) {
    Record("input", ToJson(input));
  }
  const core::session::EsrCycleResult output = session_.StepWithResult(input);
  if (sink_) {
    Record("output", ToJson(output));
  }
  return output;
}

void EsrTraceSession::ApplyRuntimeConfig(const core::session::EsrRuntimeConfigPatch& patch) {
  session_.ApplyRuntimeConfig(patch);
  if (sink_) {
    Record("runtime_config_patch", ToJson(patch));
  }
}

core::session::EsrSession& EsrTraceSession::session() { return session_; }

const core::session::EsrSession& EsrTraceSession::session() const { return session_; }

void EsrTraceSession::Record(const std::string& phase, const std::string& payload_json) const {
  sink_->Record("electronic_surveillance_radar", phase, payload_json);
}

}  // namespace tools
}  // namespace electronic_surveillance_radar
