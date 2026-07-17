#ifndef EXAMPLES_AR_CONFIG_LOADER_DETAIL_H_
#define EXAMPLES_AR_CONFIG_LOADER_DETAIL_H_

#include "1q/airborne_radar/airborne_radar.hpp"
#include "1q/environment/AtmosphericTypes.h"
#include "config_loader_common.h"
#include "json_reader.h"

namespace examples {

namespace ar_det = airborne_radar::config::detection;

// -- hardware / detection sub-tree -------------------------------------------

inline void LoadTransmitter(const examples::JsonValue& j, ar_det::TransmitterConfig* v) {
  if (j.IsNull()) return;
  v->peak_power_w = static_cast<float>(j["peak_power_w"].AsDouble());
  v->frequency_hz = static_cast<float>(j["frequency_hz"].AsDouble());
  v->bandwidth_hz = static_cast<float>(j["bandwidth_hz"].AsDouble());
  v->pulse_width_s = static_cast<float>(j["pulse_width_s"].AsDouble());
  v->prf_hz = static_cast<float>(j["prf_hz"].AsDouble());
  v->transmit_loss_db = static_cast<float>(j["transmit_loss_db"].AsDouble());
}

inline void LoadAntennaPattern(const examples::JsonValue& j,
                               ar_det::AntennaPatternConfig* v) {
  if (j.IsNull()) return;
  v->model_type = ar_det::AntennaPatternModelType::kGaussianMainLobe;
  v->max_sidelobe_level_db = static_cast<float>(j["max_sidelobe_level_db"].AsDouble());
  v->backlobe_level_db = static_cast<float>(j["backlobe_level_db"].AsDouble());
  v->scan_loss_coeff_db_per_deg2 =
      static_cast<float>(j["scan_loss_coeff_db_per_deg2"].AsDouble());
  v->max_scan_loss_db = static_cast<float>(j["max_scan_loss_db"].AsDouble());
  LoadAzEl(j["boresight_offset_deg"], &v->boresight_offset_deg);
}

inline void LoadAntenna(const examples::JsonValue& j, ar_det::AntennaConfig* v) {
  if (j.IsNull()) return;
  v->main_beam_gain_db = static_cast<float>(j["main_beam_gain_db"].AsDouble());
  v->nominal_az_beamwidth_deg =
      static_cast<float>(j["nominal_az_beamwidth_deg"].AsDouble());
  v->nominal_el_beamwidth_deg =
      static_cast<float>(j["nominal_el_beamwidth_deg"].AsDouble());
  LoadAntennaPattern(j["pattern"], &v->pattern);
  v->enable_directional_pattern = j["enable_directional_pattern"].AsBool();
}

inline void LoadReceiver(const examples::JsonValue& j, ar_det::ReceiverConfig* v) {
  if (j.IsNull()) return;
  v->noise_figure_db = static_cast<float>(j["noise_figure_db"].AsDouble());
  v->receive_loss_db = static_cast<float>(j["receive_loss_db"].AsDouble());
}

inline void LoadRcsPhysics(const examples::JsonValue& j, ar_det::RcsPhysicsConfig* v) {
  if (j.IsNull()) return;
  v->enable_physical_rcs = j["enable_physical_rcs"].AsBool();
  v->physics_mix_ratio = static_cast<float>(j["physics_mix_ratio"].AsDouble());
  v->cylinder_weight = static_cast<float>(j["cylinder_weight"].AsDouble());
  v->min_equivalent_radius_m = static_cast<float>(j["min_equivalent_radius_m"].AsDouble());
  v->max_equivalent_radius_m = static_cast<float>(j["max_equivalent_radius_m"].AsDouble());
  v->min_rcs_m2 = static_cast<float>(j["min_rcs_m2"].AsDouble());
  v->max_rcs_m2 = static_cast<float>(j["max_rcs_m2"].AsDouble());
  v->bistatic_psi_offset_deg = static_cast<float>(j["bistatic_psi_offset_deg"].AsDouble());
}

inline void LoadDetectionConfig(const examples::JsonValue& j,
                                ar_det::DetectionConfig* v) {
  if (j.IsNull()) return;
  v->enable_physics_detection = j["enable_physics_detection"].AsBool();
  LoadTransmitter(j["transmitter"], &v->transmitter);
  LoadAntenna(j["antenna"], &v->antenna);
  LoadReceiver(j["receiver"], &v->receiver);
  LoadRcsPhysics(j["rcs_physics"], &v->rcs_physics);
}

inline void LoadHardware(const examples::JsonValue& j,
                         airborne_radar::config::ArHardwareConfig* v) {
  if (j.IsNull()) return;
  LoadDetectionConfig(j["detection"], v);
}

// -- mission / orientation ---------------------------------------------------

inline void LoadOrientation(const examples::JsonValue& j,
                            airborne_radar::config::ArOrientationConfig* v) {
  if (j.IsNull()) return;
  LoadEulerAngles(j["mount_angles_deg"], &v->mount_angles_deg);
  LoadAzEl(j["scan_center_deg"], &v->scan_center_deg);
  LoadAzElLimits(j["mechanical_scan_limits_deg"], &v->mechanical_scan_limits_deg);
  LoadAzElLimits(j["electronic_scan_limits_deg"], &v->electronic_scan_limits_deg);
  v->scan_start_position = static_cast<oneq::foundation::ScanStartPosition>(
      j["scan_start_position"].AsInt());
  v->scan_sequence = static_cast<oneq::foundation::ScanSequence>(
      j["scan_sequence"].AsInt());
  v->work_mode = WorkModeFromString(j["work_mode"].AsString());
  v->commanded_beamwidth_enabled = j["commanded_beamwidth_enabled"].AsBool();
  LoadCmdBeamwidth(j["commanded_beamwidth_deg"], &v->commanded_beamwidth_deg);
  v->stabilization_mode = StabilizationFromString(j["stabilization_mode"].AsString());
}

inline void LoadMission(const examples::JsonValue& j,
                        airborne_radar::config::ArMissionConfig* v) {
  if (j.IsNull()) return;
  LoadOrientation(j["orientation"], &v->orientation);
}

// -- policy sub-tree ---------------------------------------------------------

inline void LoadDetectionPolicy(
    const examples::JsonValue& j,
    airborne_radar::config::ArDetectionPolicyConfig* v) {
  if (j.IsNull()) return;
  v->minimum_snr_db = static_cast<float>(j["minimum_snr_db"].AsDouble());
  v->pfa = static_cast<float>(j["pfa"].AsDouble());
  v->pulse_count = static_cast<int>(j["pulse_count"].AsInt());
  v->minimum_detection_margin_db =
      static_cast<float>(j["minimum_detection_margin_db"].AsDouble());
}

inline void LoadBeamPointing(const examples::JsonValue& j,
                             airborne_radar::config::BeamPointingConfig* v) {
  if (j.IsNull()) return;
  LoadCmdBeamwidth(j["nominal_beamwidth_deg"], &v->nominal_beamwidth_deg);
}

inline void LoadBeamScheduler(const examples::JsonValue& j,
                              airborne_radar::config::BeamSchedulerConfig* v) {
  if (j.IsNull()) return;
  v->azimuth_step_count_hint =
      static_cast<std::uint32_t>(j["azimuth_step_count_hint"].AsInt());
  v->elevation_step_count_hint =
      static_cast<std::uint32_t>(j["elevation_step_count_hint"].AsInt());
  v->prefer_dense_tas_sampling = j["prefer_dense_tas_sampling"].AsBool();
}

inline void LoadBeamControl(const examples::JsonValue& j,
                            airborne_radar::config::BeamControlConfig* v) {
  if (j.IsNull()) return;
  LoadBeamPointing(j["pointing"], &v->pointing);
  LoadBeamScheduler(j["scheduler"], &v->scheduler);
}

inline void LoadAssociation(const examples::JsonValue& j,
                            airborne_radar::config::AssociationConfig* v) {
  if (j.IsNull()) return;
  v->distance_gate_sigma = static_cast<float>(j["distance_gate_sigma"].AsDouble());
}

inline void LoadTracking(const examples::JsonValue& j,
                         airborne_radar::config::TrackingConfig* v) {
  if (j.IsNull()) return;
  v->enable_kalman_filter = j["enable_kalman_filter"].AsBool();
  v->kalman_measurement_noise_std =
      static_cast<float>(j["kalman_measurement_noise_std"].AsDouble());
  v->speed_decay_ratio_on_loss =
      static_cast<float>(j["speed_decay_ratio_on_loss"].AsDouble());
  v->rcs_decay_ratio_on_loss =
      static_cast<float>(j["rcs_decay_ratio_on_loss"].AsDouble());
}

inline void LoadLifecycle(const examples::JsonValue& j,
                          airborne_radar::config::LifecycleConfig* v) {
  if (j.IsNull()) return;
  v->confirm_hits = static_cast<std::uint32_t>(j["confirm_hits"].AsInt());
  v->max_miss_before_lost =
      static_cast<std::uint32_t>(j["max_miss_before_lost"].AsInt());
  v->max_lost_cycles = static_cast<std::uint32_t>(j["max_lost_cycles"].AsInt());
  v->enable_imm_lifecycle = j["enable_imm_lifecycle"].AsBool();
  if (j.Has("model_count_hint")) {
    v->model_count_hint = static_cast<std::uint32_t>(j["model_count_hint"].AsInt());
  }
}

inline void LoadPolicy(const examples::JsonValue& j,
                       airborne_radar::config::ArPolicyConfig* v) {
  if (j.IsNull()) return;
  LoadDetectionPolicy(j["detection"], &v->detection);
  LoadBeamControl(j["beam_control"], &v->beam_control);
  LoadAssociation(j["association"], &v->association);
  LoadTracking(j["tracking"], &v->tracking);
  LoadLifecycle(j["lifecycle"], &v->lifecycle);
}

// -- environment sub-tree ----------------------------------------------------

inline void LoadAtmosObservation(const examples::JsonValue& j,
                                 oneq::environment::AtmosphericObservation* v) {
  if (j.IsNull()) return;
  v->enable_physical_model = j["enable_physical_model"].AsBool();
  v->pressure_hpa = static_cast<float>(j["pressure_hpa"].AsDouble());
  v->temperature_k = static_cast<float>(j["temperature_k"].AsDouble());
  v->relative_humidity = static_cast<float>(j["relative_humidity"].AsDouble());
}

inline void LoadAtmosContext(const examples::JsonValue& j,
                             airborne_radar::config::AtmosphericDerivedContext* v) {
  if (j.IsNull()) return;
  v->has_simulation_unix_seconds = j["has_simulation_unix_seconds"].AsBool();
  v->simulation_unix_seconds =
      static_cast<std::int64_t>(j["simulation_unix_seconds"].AsInt());
  v->solar_flux_f107a = static_cast<float>(j["solar_flux_f107a"].AsDouble());
  v->solar_flux_f107 = static_cast<float>(j["solar_flux_f107"].AsDouble());
  v->geomagnetic_ap = static_cast<float>(j["geomagnetic_ap"].AsDouble());
}

inline void LoadVegScatter(
    const examples::JsonValue& j,
    airborne_radar::config::VegetationScatterPhysicsConfig* v) {
  if (j.IsNull()) return;
  v->cover_profile = VegCoverFromString(j["cover_profile"].AsString());
  v->enable_physical_model = j["enable_physical_model"].AsBool();
}

inline void LoadScenario(const examples::JsonValue& j,
                         airborne_radar::config::EnvironmentScenarioConfig* v) {
  if (j.IsNull()) return;
  LoadAtmosObservation(j["atmospheric_physics"], &v->atmospheric_physics);
  LoadAtmosContext(j["atmospheric_context"], &v->atmospheric_context);
  LoadVegScatter(j["vegetation_scatter_physics"], &v->vegetation_scatter_physics);
}

inline void LoadEnvironment(const examples::JsonValue& j,
                            airborne_radar::config::ArEnvironmentConfig* v) {
  if (j.IsNull()) return;
  LoadScenario(j["scenario_config"], &v->scenario_config);
}

}  // namespace examples

#endif  // EXAMPLES_AR_CONFIG_LOADER_DETAIL_H_
