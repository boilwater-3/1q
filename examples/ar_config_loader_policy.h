#ifndef EXAMPLES_AR_CONFIG_LOADER_POLICY_H_
#define EXAMPLES_AR_CONFIG_LOADER_POLICY_H_

#include "1q/airborne_radar/airborne_radar.hpp"
#include "1q/foundation/json_reader.h"
#include "ar_config_loader_common.h"

namespace examples {

// -- mission / orientation ---------------------------------------------------

inline void LoadOrientation(const oneq::JsonValue& j,
                            airborne_radar::model::RadarOrientationConfig* v) {
  if (j.IsNull()) return;
  LoadEulerAngles(j["mount_angles_deg"], &v->mount_angles_deg);
  LoadAzEl(j["scan_center_deg"], &v->scan_center_deg);
  LoadAzElLimits(j["mechanical_scan_limits_deg"], &v->mechanical_scan_limits_deg);
  LoadAzElLimits(j["electronic_scan_limits_deg"], &v->electronic_scan_limits_deg);
  v->scan_start_position = static_cast<oneq::foundation::ScanStartPosition>(
      j["scan_start_position"].AsInt());
  v->scan_sequence = static_cast<oneq::foundation::ScanSequence>(
      j["scan_sequence"].AsInt());
  v->work_sub_mode = WorkSubModeFromString(j["work_sub_mode"].AsString());
  v->commanded_beamwidth_enabled = j["commanded_beamwidth_enabled"].AsBool();
  LoadCmdBeamwidth(j["commanded_beamwidth_deg"], &v->commanded_beamwidth_deg);
  v->stabilization_mode = StabilizationFromString(j["stabilization_mode"].AsString());
}

inline void LoadMission(const oneq::JsonValue& j,
                        airborne_radar::config::RadarMissionConfig* v) {
  if (j.IsNull()) return;
  LoadOrientation(j["orientation"], &v->orientation);
}

// -- policy sub-tree ---------------------------------------------------------

inline void LoadBeamPointing(const oneq::JsonValue& j,
                             airborne_radar::config::BeamPointingConfig* v) {
  if (j.IsNull()) return;
  LoadAzEl(j["default_scan_center_deg"], &v->default_scan_center_deg);
  LoadCmdBeamwidth(j["nominal_beamwidth_deg"], &v->nominal_beamwidth_deg);
}

inline void LoadBeamScheduler(const oneq::JsonValue& j,
                              airborne_radar::config::BeamSchedulerConfig* v) {
  if (j.IsNull()) return;
  v->azimuth_step_count_hint =
      static_cast<std::uint32_t>(j["azimuth_step_count_hint"].AsInt());
  v->elevation_step_count_hint =
      static_cast<std::uint32_t>(j["elevation_step_count_hint"].AsInt());
  v->prefer_dense_tas_sampling = j["prefer_dense_tas_sampling"].AsBool();
}

inline void LoadBeamControl(const oneq::JsonValue& j,
                            airborne_radar::config::BeamControlConfig* v) {
  if (j.IsNull()) return;
  LoadBeamPointing(j["pointing"], &v->pointing);
  LoadBeamScheduler(j["scheduler"], &v->scheduler);
}

inline void LoadAssociation(const oneq::JsonValue& j,
                            airborne_radar::config::AssociationConfig* v) {
  if (j.IsNull()) return;
  v->unassigned_cost = static_cast<float>(j["unassigned_cost"].AsDouble());
  v->use_distance_gate_hint = j["use_distance_gate_hint"].AsBool();
  v->distance_gate_sigma_hint =
      static_cast<float>(j["distance_gate_sigma_hint"].AsDouble());
}

inline void LoadTracking(const oneq::JsonValue& j,
                         airborne_radar::config::TrackingConfig* v) {
  if (j.IsNull()) return;
  v->enable_kalman_filter = j["enable_kalman_filter"].AsBool();
  v->kalman_measurement_noise_std =
      static_cast<float>(j["kalman_measurement_noise_std"].AsDouble());
  v->kalman_update_backend =
      KalmanBackendFromString(j["kalman_update_backend"].AsString());
  v->speed_decay_ratio_on_loss =
      static_cast<float>(j["speed_decay_ratio_on_loss"].AsDouble());
  v->rcs_decay_ratio_on_loss =
      static_cast<float>(j["rcs_decay_ratio_on_loss"].AsDouble());
}

inline void LoadLifecycle(const oneq::JsonValue& j,
                          airborne_radar::config::LifecycleConfig* v) {
  if (j.IsNull()) return;
  v->confirm_hits = static_cast<std::uint32_t>(j["confirm_hits"].AsInt());
  v->max_miss_before_lost =
      static_cast<std::uint32_t>(j["max_miss_before_lost"].AsInt());
  v->max_lost_cycles = static_cast<std::uint32_t>(j["max_lost_cycles"].AsInt());
  v->enable_imm_lifecycle = j["enable_imm_lifecycle"].AsBool();
}

inline void LoadImm(const oneq::JsonValue& j, airborne_radar::config::ImmConfig* v) {
  if (j.IsNull()) return;
  v->enable_imm_lifecycle = j["enable_imm_lifecycle"].AsBool();
  v->model_count_hint = static_cast<std::uint32_t>(j["model_count_hint"].AsInt());
}

inline void LoadPolicy(const oneq::JsonValue& j,
                       airborne_radar::config::RadarPolicyConfig* v) {
  if (j.IsNull()) return;
  LoadBeamControl(j["beam_control"], &v->beam_control);
  LoadAssociation(j["association"], &v->association);
  LoadTracking(j["tracking"], &v->tracking);
  LoadLifecycle(j["lifecycle"], &v->lifecycle);
  LoadImm(j["imm"], &v->imm);
}

// -- environment sub-tree ----------------------------------------------------

inline void LoadAtmosObservation(const oneq::JsonValue& j,
                                 oneq::foundation::AtmosphericObservation* v) {
  if (j.IsNull()) return;
  v->enable_physical_model = j["enable_physical_model"].AsBool();
  v->pressure_hpa = static_cast<float>(j["pressure_hpa"].AsDouble());
  v->temperature_k = static_cast<float>(j["temperature_k"].AsDouble());
  v->relative_humidity = static_cast<float>(j["relative_humidity"].AsDouble());
}

inline void LoadAtmosContext(const oneq::JsonValue& j,
                             airborne_radar::environment::AtmosphericDerivedContext* v) {
  if (j.IsNull()) return;
  v->has_simulation_unix_seconds = j["has_simulation_unix_seconds"].AsBool();
  v->simulation_unix_seconds =
      static_cast<std::int64_t>(j["simulation_unix_seconds"].AsInt());
  v->solar_flux_f107a = static_cast<float>(j["solar_flux_f107a"].AsDouble());
  v->solar_flux_f107 = static_cast<float>(j["solar_flux_f107"].AsDouble());
  v->geomagnetic_ap = static_cast<float>(j["geomagnetic_ap"].AsDouble());
}

inline void LoadVegScatter(
    const oneq::JsonValue& j,
    airborne_radar::environment::VegetationScatterPhysicsConfig* v) {
  if (j.IsNull()) return;
  v->cover_profile = VegCoverFromString(j["cover_profile"].AsString());
  v->enable_physical_model = j["enable_physical_model"].AsBool();
}

inline void LoadScenario(const oneq::JsonValue& j,
                         airborne_radar::environment::EnvironmentScenarioConfig* v) {
  if (j.IsNull()) return;
  LoadAtmosObservation(j["atmospheric_physics"], &v->atmospheric_physics);
  LoadAtmosContext(j["atmospheric_context"], &v->atmospheric_context);
  LoadVegScatter(j["vegetation_scatter_physics"], &v->vegetation_scatter_physics);
}

inline void LoadEnvironment(const oneq::JsonValue& j,
                            airborne_radar::config::RadarEnvironmentConfig* v) {
  if (j.IsNull()) return;
  LoadScenario(j["scenario_config"], &v->scenario_config);
}

}  // namespace examples

#endif  // EXAMPLES_AR_CONFIG_LOADER_POLICY_H_
