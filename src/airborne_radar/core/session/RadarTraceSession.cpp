#include "1q/airborne_radar/tools/RadarTraceSession.h"

#include <sstream>
#include <string>

#include "1q/airborne_radar/config/RadarRuntimeConfigBuilder.h"
#include "common/trace/JsonFormatUtils.h"

namespace airborne_radar {
namespace tools {
namespace {

using oneq::common::trace::internal::BoolToJson;
using oneq::common::trace::internal::QuoteString;

template <typename T>
std::string NumberArray3(T first, T second, T third) {
  std::ostringstream stream;
  stream << "[" << first << "," << second << "," << third << "]";
  return stream.str();
}

std::string ToJson(const common::config::EulerAnglesDeg& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"yaw_deg\":" << value.yaw_deg << ",";
  stream << "\"pitch_deg\":" << value.pitch_deg << ",";
  stream << "\"roll_deg\":" << value.roll_deg;
  stream << "}";
  return stream.str();
}

std::string ToJson(const common::config::AzimuthElevationDeg& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"az_deg\":" << value.az_deg << ",";
  stream << "\"el_deg\":" << value.el_deg;
  stream << "}";
  return stream.str();
}

std::string ToJson(const common::config::AzimuthElevationLimitsDeg& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"az_min_deg\":" << value.az_min_deg << ",";
  stream << "\"az_max_deg\":" << value.az_max_deg << ",";
  stream << "\"el_min_deg\":" << value.el_min_deg << ",";
  stream << "\"el_max_deg\":" << value.el_max_deg;
  stream << "}";
  return stream.str();
}

std::string ToJson(const common::config::CommandedBeamwidthDeg& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"commanded_az_beamwidth_deg\":" << value.commanded_az_beamwidth_deg << ",";
  stream << "\"commanded_el_beamwidth_deg\":" << value.commanded_el_beamwidth_deg;
  stream << "}";
  return stream.str();
}

std::string ToJson(const common::config::RadarOrientationConfig& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"mount_angles_deg\":" << ToJson(value.mount_angles_deg) << ",";
  stream << "\"scan_center_deg\":" << ToJson(value.scan_center_deg) << ",";
  stream << "\"mechanical_scan_limits_deg\":" << ToJson(value.mechanical_scan_limits_deg)
         << ",";
  stream << "\"electronic_scan_limits_deg\":" << ToJson(value.electronic_scan_limits_deg)
         << ",";
  stream << "\"scan_start_position\":" << static_cast<int>(value.scan_start_position) << ",";
  stream << "\"scan_sequence\":" << static_cast<int>(value.scan_sequence) << ",";
  stream << "\"work_sub_mode\":" << static_cast<int>(value.work_sub_mode) << ",";
  stream << "\"dwell_center_deg\":" << ToJson(value.dwell_center_deg) << ",";
  stream << "\"commanded_beamwidth_enabled\":" << BoolToJson(value.commanded_beamwidth_enabled)
         << ",";
  stream << "\"commanded_beamwidth_deg\":" << ToJson(value.commanded_beamwidth_deg) << ",";
  stream << "\"stabilization_mode\":" << static_cast<int>(value.stabilization_mode);
  stream << "}";
  return stream.str();
}

std::string ToJson(const common::config::AntennaPatternConfig& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"model_type\":" << static_cast<int>(value.model_type) << ",";
  stream << "\"max_sidelobe_level_db\":" << value.max_sidelobe_level_db << ",";
  stream << "\"backlobe_level_db\":" << value.backlobe_level_db << ",";
  stream << "\"scan_loss_coeff_db_per_deg2\":" << value.scan_loss_coeff_db_per_deg2 << ",";
  stream << "\"max_scan_loss_db\":" << value.max_scan_loss_db << ",";
  stream << "\"boresight_offset_deg\":" << ToJson(value.boresight_offset_deg);
  stream << "}";
  return stream.str();
}

std::string ToJson(const common::config::SignalDetectionConfig& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"enable_physics_detection\":" << BoolToJson(value.enable_physics_detection) << ",";
  stream << "\"min_detection_margin_db\":" << value.min_detection_margin_db << ",";
  stream << "\"pulse_count\":" << value.pulse_count << ",";
  stream << "\"radar_system\":{";
  stream << "\"transmitter\":{";
  stream << "\"peak_power_w\":" << value.radar_system.transmitter.peak_power_w << ",";
  stream << "\"frequency_hz\":" << value.radar_system.transmitter.frequency_hz << ",";
  stream << "\"bandwidth_hz\":" << value.radar_system.transmitter.bandwidth_hz << ",";
  stream << "\"pulse_width_s\":" << value.radar_system.transmitter.pulse_width_s << ",";
  stream << "\"prf_hz\":" << value.radar_system.transmitter.prf_hz << ",";
  stream << "\"transmit_loss_db\":" << value.radar_system.transmitter.transmit_loss_db;
  stream << "},";
  stream << "\"antenna\":{";
  stream << "\"main_beam_gain_db\":" << value.radar_system.antenna.main_beam_gain_db << ",";
  stream << "\"nominal_az_beamwidth_deg\":" << value.radar_system.antenna.nominal_az_beamwidth_deg
         << ",";
  stream << "\"nominal_el_beamwidth_deg\":" << value.radar_system.antenna.nominal_el_beamwidth_deg
         << ",";
  stream << "\"enable_directional_pattern\":"
         << BoolToJson(value.radar_system.antenna.enable_directional_pattern) << ",";
  stream << "\"pattern\":" << ToJson(value.radar_system.antenna.pattern);
  stream << "},";
  stream << "\"receiver\":{";
  stream << "\"noise_figure_db\":" << value.radar_system.receiver.noise_figure_db << ",";
  stream << "\"receive_loss_db\":" << value.radar_system.receiver.receive_loss_db;
  stream << "},";
  stream << "\"detection\":{";
  stream << "\"cfar_pfa\":" << value.radar_system.detection.cfar_pfa << ",";
  stream << "\"min_snr_db\":" << value.radar_system.detection.min_snr_db;
  stream << "}";
  stream << "}";
  stream << "}";
  return stream.str();
}

std::string ToJson(const common::config::SignalTrackingConfig& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"enable_kalman_filter\":" << BoolToJson(value.enable_kalman_filter) << ",";
  stream << "\"kalman_measurement_noise_std\":" << value.kalman_measurement_noise_std;
  stream << "}";
  return stream.str();
}

std::string ToJson(const common::config::LifecycleConfig& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"confirm_hits\":" << value.confirm_hits << ",";
  stream << "\"max_miss_before_lost\":" << value.max_miss_before_lost << ",";
  stream << "\"max_lost_cycles\":" << value.max_lost_cycles;
  stream << "}";
  return stream.str();
}

std::string ToJson(const common::config::SignalLifecycleConfig& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"enable_auto_lifecycle_manager\":"
         << BoolToJson(value.enable_auto_lifecycle_manager) << ",";
  stream << "\"lifecycle_config\":" << ToJson(value.lifecycle_config) << ",";
  stream << "\"enable_imm_lifecycle\":" << BoolToJson(value.enable_imm_lifecycle);
  stream << "}";
  return stream.str();
}

std::string ToJson(const common::config::SignalPipelineConfig& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"detection\":" << ToJson(value.detection) << ",";
  stream << "\"beam_control\":{";
  stream << "\"radar_orientation\":" << ToJson(value.beam_control.radar_orientation) << ",";
  stream << "\"platform_attitude_deg\":" << ToJson(value.beam_control.platform_attitude_deg);
  stream << "},";
  stream << "\"tracking\":" << ToJson(value.tracking) << ",";
  stream << "\"lifecycle\":" << ToJson(value.lifecycle);
  stream << "}";
  return stream.str();
}

std::string ToJson(const environment::AtmosphericPhysicsConfig& value) {
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

std::string ToJson(const environment::JammerSourceFact& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"technique\":" << static_cast<int>(value.technique) << ",";
  stream << "\"power_db\":" << value.power_db << ",";
  stream << "\"js_db\":" << value.js_db << ",";
  stream << "\"frequency_overlap_ratio\":" << value.frequency_overlap_ratio << ",";
  stream << "\"prf_lock_risk\":" << value.prf_lock_risk << ",";
  stream << "\"azimuth_deg\":" << value.azimuth_deg << ",";
  stream << "\"elevation_deg\":" << value.elevation_deg << ",";
  stream << "\"angular_span_deg\":" << value.angular_span_deg << ",";
  stream << "\"in_sidelobe\":" << BoolToJson(value.in_sidelobe) << ",";
  stream << "\"confidence\":" << value.confidence;
  stream << "}";
  return stream.str();
}

std::string ToJson(const environment::EnvironmentModelConfig& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"base_propagation_loss_db\":" << value.base_propagation_loss_db << ",";
  stream << "\"atmospheric_attenuation_db\":" << value.atmospheric_attenuation_db << ",";
  stream << "\"terrain_reflection_db\":" << value.terrain_reflection_db << ",";
  stream << "\"clutter_power_db\":" << value.clutter_power_db << ",";
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

std::string ToJson(const environment::EnvironmentSceneState& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"base_propagation_loss_db\":" << value.base_propagation_loss_db << ",";
  stream << "\"atmospheric_attenuation_db\":" << value.atmospheric_attenuation_db << ",";
  stream << "\"terrain_reflection_db\":" << value.terrain_reflection_db << ",";
  stream << "\"clutter_power_db\":" << value.clutter_power_db << ",";
  stream << "\"atmospheric_physics\":" << ToJson(value.atmospheric_physics) << ",";
  stream << "\"jammer_emitters\":[";
  for (std::size_t i = 0; i < value.jammer_emitters.size(); ++i) {
    if (i > 0U) {
      stream << ",";
    }
    stream << ToJson(value.jammer_emitters[i]);
  }
  stream << "]";
  stream << "}";
  return stream.str();
}

std::string ToJson(const core::session::RadarSessionConfig& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"signal_pipeline_config\":" << ToJson(value.signal_pipeline_config) << ",";
  stream << "\"environment_model_config\":" << ToJson(value.environment_model_config) << ",";
  stream << "\"jamming_detection_threshold_db\":" << value.jamming_detection_threshold_db;
  stream << "}";
  return stream.str();
}

std::string ToJson(const common::model::TargetFeature& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"external_target_id\":" << value.external_target_id << ",";
  stream << "\"velocity_mps\":"
         << NumberArray3(value.current_track_velocity_x, value.current_track_velocity_y,
                         value.current_track_velocity_z)
         << ",";
  stream << "\"current_track_speed\":" << value.current_track_speed << ",";
  stream << "\"current_track_rcs\":" << value.current_track_rcs << ",";
  stream << "\"range_m\":" << value.range_m << ",";
  stream << "\"has_cartesian_position\":" << BoolToJson(value.has_cartesian_position) << ",";
  stream << "\"position_m\":" << NumberArray3(value.position_x, value.position_y, value.position_z)
         << ",";
  stream << "\"target_swerling_type\":" << value.target_swerling_type;
  stream << "}";
  return stream.str();
}

std::string ToJson(const core::context::RadarCycleInput& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"dt_sec\":" << value.dt_sec << ",";
  stream << "\"platform_attitude_deg\":" << ToJson(value.platform_attitude_deg) << ",";
  stream << "\"target_features\":[";
  for (std::size_t i = 0; i < value.target_features.size(); ++i) {
    if (i > 0U) {
      stream << ",";
    }
    stream << ToJson(value.target_features[i]);
  }
  stream << "]";
  stream << "}";
  return stream.str();
}

std::string ToJson(const core::context::ValidationIssue& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"severity\":" << static_cast<int>(value.severity) << ",";
  stream << "\"code\":" << static_cast<int>(value.code) << ",";
  stream << "\"target_index\":" << value.target_index << ",";
  stream << "\"message\":" << QuoteString(value.message);
  stream << "}";
  return stream.str();
}

std::string ToJson(const common::control::RadarCommand& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"type\":" << static_cast<int>(value.type) << ",";
  stream << "\"source\":" << static_cast<int>(value.source);
  stream << "}";
  return stream.str();
}

std::string ToJson(const common::control::RadarControlProfile& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"version\":" << value.version << ",";
  stream << "\"enable_lpi_power_control\":" << BoolToJson(value.enable_lpi_power_control) << ",";
  stream << "\"lpi_power_scale\":" << value.lpi_power_scale << ",";
  stream << "\"enable_lpi_beamforming\":" << BoolToJson(value.enable_lpi_beamforming) << ",";
  stream << "\"lpi_dwell_scale\":" << value.lpi_dwell_scale << ",";
  stream << "\"enable_agility_frequency\":" << BoolToJson(value.enable_agility_frequency) << ",";
  stream << "\"agility_frequency_hop_phase\":"
         << static_cast<unsigned int>(value.agility_frequency_hop_phase) << ",";
  stream << "\"enable_sidelobe_canceller\":" << BoolToJson(value.enable_sidelobe_canceller) << ",";
  stream << "\"enable_adaptive_beamforming\":" << BoolToJson(value.enable_adaptive_beamforming) << ",";
  stream << "\"enable_eccm_rejitter\":" << BoolToJson(value.enable_eccm_rejitter) << ",";
  stream << "\"eccm_burnthrough_gain\":" << value.eccm_burnthrough_gain;
  stream << "}";
  return stream.str();
}

std::string ToJson(const signal::pipeline::AssociationQualityMetrics& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"prior_track_count\":" << value.prior_track_count << ",";
  stream << "\"detection_count\":" << value.detection_count << ",";
  stream << "\"matched_count\":" << value.matched_count << ",";
  stream << "\"new_track_count\":" << value.new_track_count << ",";
  stream << "\"missed_track_count\":" << value.missed_track_count << ",";
  stream << "\"match_rate\":" << value.match_rate << ",";
  stream << "\"new_track_rate\":" << value.new_track_rate << ",";
  stream << "\"missed_track_rate\":" << value.missed_track_rate << ",";
  stream << "\"mean_match_cost\":" << value.mean_match_cost << ",";
  stream << "\"p95_match_cost\":" << value.p95_match_cost << ",";
  stream << "\"dominant_jamming_semantic\":" << static_cast<int>(value.dominant_jamming_semantic)
         << ",";
  stream << "\"jamming_severity\":" << value.jamming_severity << ",";
  stream << "\"association_stress\":" << value.association_stress;
  stream << "}";
  return stream.str();
}

std::string ToJson(const common::model::DecisionTrackSnapshot& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"state\":{";
  stream << "\"association_key\":" << value.state.association_key << ",";
  stream << "\"external_target_id\":" << value.state.external_target_id << ",";
  stream << "\"status\":" << static_cast<int>(value.state.status) << ",";
  stream << "\"position_m\":"
         << NumberArray3(value.state.position_x, value.state.position_y, value.state.position_z)
         << ",";
  stream << "\"velocity_mps\":"
         << NumberArray3(value.state.velocity_x, value.state.velocity_y, value.state.velocity_z)
         << ",";
  stream << "\"speed\":" << value.state.speed << ",";
  stream << "\"acceleration_mps2\":"
         << NumberArray3(value.state.acceleration_x, value.state.acceleration_y,
                         value.state.acceleration_z)
         << ",";
  stream << "\"acceleration\":" << value.state.acceleration << ",";
  stream << "\"rcs\":" << value.state.rcs << ",";
  stream << "\"jamming_detected\":" << BoolToJson(value.state.jamming_detected) << ",";
  stream << "\"hit_count\":" << value.state.hit_count << ",";
  stream << "\"miss_count\":" << value.state.miss_count;
  stream << "},";
  stream << "\"evidence\":{";
  stream << "\"has_measurement_evidence\":" << BoolToJson(value.evidence.has_measurement_evidence)
         << ",";
  stream << "\"updated_this_cycle\":" << BoolToJson(value.evidence.updated_this_cycle) << ",";
  stream << "\"predicted_only_this_cycle\":" << BoolToJson(value.evidence.predicted_only_this_cycle)
         << ",";
  stream << "\"matched_existing_track\":" << BoolToJson(value.evidence.matched_existing_track)
         << ",";
  stream << "\"association_cost\":" << value.evidence.association_cost << ",";
  stream << "\"detection_margin_db\":" << value.evidence.detection_margin_db << ",";
  stream << "\"used_position_association\":" << BoolToJson(value.evidence.used_position_association)
         << ",";
  stream << "\"used_external_association_seeds\":"
         << BoolToJson(value.evidence.used_external_association_seeds);
  stream << "}";
  stream << "}";
  return stream.str();
}

std::string ToJson(const common::output::TrackOutputFrame& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"cycle_index\":" << value.cycle_index << ",";
  stream << "\"batch_id\":" << value.batch_id << ",";
  stream << "\"published_track_count\":" << value.published_track_count << ",";
  stream << "\"confirmed_track_count\":" << value.confirmed_track_count << ",";
  stream << "\"contains_lost_tracks\":" << BoolToJson(value.contains_lost_tracks) << ",";
  stream << "\"tracks\":[";
  for (std::size_t i = 0; i < value.tracks.size(); ++i) {
    if (i > 0U) {
      stream << ",";
    }
    stream << ToJson(value.tracks[i]);
  }
  stream << "]";
  stream << "}";
  return stream.str();
}

std::string ToJson(const core::session::RadarCycleResult& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"track_output_frame\":" << ToJson(value.track_output_frame) << ",";
  stream << "\"submitted_commands\":[";
  for (std::size_t i = 0; i < value.submitted_commands.size(); ++i) {
    if (i > 0U) {
      stream << ",";
    }
    stream << ToJson(value.submitted_commands[i]);
  }
  stream << "],";
  stream << "\"validation_issues\":[";
  for (std::size_t i = 0; i < value.validation_issues.size(); ++i) {
    if (i > 0U) {
      stream << ",";
    }
    stream << ToJson(value.validation_issues[i]);
  }
  stream << "],";
  stream << "\"has_validation_error\":" << BoolToJson(value.has_validation_error) << ",";
  stream << "\"has_control_profile\":" << BoolToJson(value.has_control_profile) << ",";
  stream << "\"control_profile\":" << ToJson(value.control_profile) << ",";
  stream << "\"association_quality_metrics\":" << ToJson(value.association_quality_metrics);
  stream << "}";
  return stream.str();
}

std::string ToJson(const common::config::RadarRuntimeConfigPatch& value) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"has_signal_pipeline_config\":" << BoolToJson(value.has_signal_pipeline_config)
         << ",";
  stream << "\"signal_pipeline_config\":" << ToJson(value.signal_pipeline_config) << ",";
  stream << "\"has_signal_detection_config\":" << BoolToJson(value.has_signal_detection_config)
         << ",";
  stream << "\"signal_detection_config\":" << ToJson(value.signal_detection_config) << ",";
  stream << "\"has_rcs_enable_physical_override\":"
         << BoolToJson(value.has_rcs_enable_physical_override) << ",";
  stream << "\"rcs_enable_physical_override\":"
         << BoolToJson(value.rcs_enable_physical_override) << ",";
  stream << "\"has_rcs_physics_frequency_hz\":"
         << BoolToJson(value.has_rcs_physics_frequency_hz) << ",";
  stream << "\"rcs_physics_frequency_hz\":" << value.rcs_physics_frequency_hz << ",";
  stream << "\"has_rcs_physics_mix_ratio\":"
         << BoolToJson(value.has_rcs_physics_mix_ratio) << ",";
  stream << "\"rcs_physics_mix_ratio\":" << value.rcs_physics_mix_ratio << ",";
  stream << "\"has_rcs_physics_cylinder_weight\":"
         << BoolToJson(value.has_rcs_physics_cylinder_weight) << ",";
  stream << "\"rcs_physics_cylinder_weight\":" << value.rcs_physics_cylinder_weight << ",";
  stream << "\"has_rcs_equivalent_radius_range\":"
         << BoolToJson(value.has_rcs_equivalent_radius_range) << ",";
  stream << "\"rcs_min_equivalent_radius_m\":" << value.rcs_min_equivalent_radius_m << ",";
  stream << "\"rcs_max_equivalent_radius_m\":" << value.rcs_max_equivalent_radius_m << ",";
  stream << "\"has_rcs_output_range_m2\":" << BoolToJson(value.has_rcs_output_range_m2) << ",";
  stream << "\"rcs_min_rcs_m2\":" << value.rcs_min_rcs_m2 << ",";
  stream << "\"rcs_max_rcs_m2\":" << value.rcs_max_rcs_m2 << ",";
  stream << "\"has_rcs_bistatic_psi_offset_deg\":"
         << BoolToJson(value.has_rcs_bistatic_psi_offset_deg) << ",";
  stream << "\"rcs_bistatic_psi_offset_deg\":" << value.rcs_bistatic_psi_offset_deg << ",";
  stream << "\"has_signal_beam_control_config\":"
         << BoolToJson(value.has_signal_beam_control_config) << ",";
  stream << "\"signal_beam_control_config\":{";
  stream << "\"radar_orientation\":" << ToJson(value.signal_beam_control_config.radar_orientation)
         << ",";
  stream << "\"platform_attitude_deg\":"
         << ToJson(value.signal_beam_control_config.platform_attitude_deg);
  stream << "},";
  stream << "\"has_environment_model_config\":" << BoolToJson(value.has_environment_model_config)
         << ",";
  stream << "\"environment_model_config\":" << ToJson(value.environment_model_config) << ",";
  stream << "\"has_jamming_detection_threshold_db\":"
         << BoolToJson(value.has_jamming_detection_threshold_db) << ",";
  stream << "\"jamming_detection_threshold_db\":" << value.jamming_detection_threshold_db << ",";
  stream << "\"has_platform_attitude_deg\":" << BoolToJson(value.has_platform_attitude_deg)
         << ",";
  stream << "\"platform_attitude_deg\":" << ToJson(value.platform_attitude_deg) << ",";
  stream << "\"has_work_sub_mode\":" << BoolToJson(value.has_work_sub_mode) << ",";
  stream << "\"work_sub_mode\":" << static_cast<int>(value.work_sub_mode) << ",";
  stream << "\"has_scan_center_deg\":" << BoolToJson(value.has_scan_center_deg) << ",";
  stream << "\"scan_center_deg\":" << ToJson(value.scan_center_deg) << ",";
  stream << "\"has_dwell_center_deg\":" << BoolToJson(value.has_dwell_center_deg) << ",";
  stream << "\"dwell_center_deg\":" << ToJson(value.dwell_center_deg) << ",";
  stream << "\"has_commanded_beamwidth_deg\":" << BoolToJson(value.has_commanded_beamwidth_deg)
         << ",";
  stream << "\"commanded_beamwidth_deg\":" << ToJson(value.commanded_beamwidth_deg) << ",";
  stream << "\"has_commanded_beamwidth_enabled\":"
         << BoolToJson(value.has_commanded_beamwidth_enabled) << ",";
  stream << "\"commanded_beamwidth_enabled\":" << BoolToJson(value.commanded_beamwidth_enabled);
  stream << "}";
  return stream.str();
}

}  // namespace

RadarTraceSession::RadarTraceSession(const core::session::RadarSessionConfig& config,
                                     RadarTraceSessionOptions options)
    : session_(config), sink_(std::move(options.sink)) {
  if (sink_ && options.trace_config_on_construct) {
    Record("config", ToJson(config));
  }
}

common::output::TrackOutputFrame RadarTraceSession::Step(const core::context::RadarCycleInput& input) {
  if (sink_) {
    Record("input", ToJson(input));
  }
  const common::output::TrackOutputFrame output = session_.Step(input);
  if (sink_) {
    Record("output", ToJson(output));
  }
  return output;
}

common::output::TrackOutputFrame RadarTraceSession::Step(
    const core::context::RadarCycleInput& input, const environment::EnvironmentSceneState& scene_state) {
  if (sink_) {
    std::ostringstream stream;
    stream << "{";
    stream << "\"cycle_input\":" << ToJson(input) << ",";
    stream << "\"scene_state\":" << ToJson(scene_state);
    stream << "}";
    Record("input", stream.str());
  }
  const common::output::TrackOutputFrame output = session_.Step(input, scene_state);
  if (sink_) {
    Record("output", ToJson(output));
  }
  return output;
}

core::session::RadarCycleResult RadarTraceSession::StepWithResult(
    const core::context::RadarCycleInput& input) {
  if (sink_) {
    Record("input", ToJson(input));
  }
  const core::session::RadarCycleResult output = session_.StepWithResult(input);
  if (sink_) {
    Record("output", ToJson(output));
  }
  return output;
}

core::session::RadarCycleResult RadarTraceSession::StepWithResult(
    const core::context::RadarCycleInput& input, const environment::EnvironmentSceneState& scene_state) {
  if (sink_) {
    std::ostringstream stream;
    stream << "{";
    stream << "\"cycle_input\":" << ToJson(input) << ",";
    stream << "\"scene_state\":" << ToJson(scene_state);
    stream << "}";
    Record("input", stream.str());
  }
  const core::session::RadarCycleResult output = session_.StepWithResult(input, scene_state);
  if (sink_) {
    Record("output", ToJson(output));
  }
  return output;
}

void RadarTraceSession::UpdateSignalPipelineConfig(const common::config::SignalPipelineConfig& config) {
  session_.UpdateSignalPipelineConfig(config);
  if (sink_) {
    std::ostringstream stream;
    stream << "{";
    stream << "\"signal_pipeline_config\":" << ToJson(config);
    stream << "}";
    Record("runtime_config", stream.str());
  }
}

void RadarTraceSession::UpdateEnvironmentModelConfig(const environment::EnvironmentModelConfig& config) {
  session_.UpdateEnvironmentModelConfig(config);
  if (sink_) {
    std::ostringstream stream;
    stream << "{";
    stream << "\"environment_model_config\":" << ToJson(config);
    stream << "}";
    Record("runtime_config", stream.str());
  }
}

void RadarTraceSession::SetJammingDetectionThresholdDb(float threshold_db) {
  session_.SetJammingDetectionThresholdDb(threshold_db);
  if (sink_) {
    std::ostringstream stream;
    stream << "{";
    stream << "\"jamming_detection_threshold_db\":" << threshold_db;
    stream << "}";
    Record("runtime_config", stream.str());
  }
}

void RadarTraceSession::ApplyRuntimeConfig(const common::config::RadarRuntimeConfigPatch& patch) {
  session_.ApplyRuntimeConfig(patch);
  if (sink_) {
    Record("runtime_config", ToJson(patch));
  }
}

const std::vector<common::control::RadarCommand>& RadarTraceSession::GetSubmittedCommands() const {
  return session_.GetSubmittedCommands();
}

bool RadarTraceSession::HasLatestControlProfile() const { return session_.HasLatestControlProfile(); }

const common::control::RadarControlProfile& RadarTraceSession::GetLatestControlProfile() const {
  return session_.GetLatestControlProfile();
}

signal::pipeline::AssociationQualityMetrics RadarTraceSession::GetLastAssociationQualityMetrics() const {
  return session_.GetLastAssociationQualityMetrics();
}

core::session::RadarSession& RadarTraceSession::session() { return session_; }

const core::session::RadarSession& RadarTraceSession::session() const { return session_; }

void RadarTraceSession::Record(const std::string& phase, const std::string& payload_json) const {
  sink_->Record("airborne_radar", phase, payload_json);
}

}  // namespace tools
}  // namespace airborne_radar
