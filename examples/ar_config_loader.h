#ifndef EXAMPLES_AR_CONFIG_LOADER_H_
#define EXAMPLES_AR_CONFIG_LOADER_H_

#include <string>

#include "1q/airborne_radar/airborne_radar.hpp"
#include "1q/foundation/json_reader.h"

namespace examples {

// -- namespace aliases -------------------------------------------------------
namespace ar_det = airborne_radar::config::detection;
namespace ar_pro = airborne_radar::config::profiles;

// -- enum helpers ------------------------------------------------------------

inline ar_pro::SwerlingModel SwerlingModelFromString(const std::string& s) {
  if (s == "kSwerling0") return ar_pro::SwerlingModel::kSwerling0;
  if (s == "kSwerling1") return ar_pro::SwerlingModel::kSwerling1;
  if (s == "kSwerling2") return ar_pro::SwerlingModel::kSwerling2;
  if (s == "kSwerling3") return ar_pro::SwerlingModel::kSwerling3;
  if (s == "kSwerling4") return ar_pro::SwerlingModel::kSwerling4;
  return ar_pro::SwerlingModel::kSwerling0;
}

inline ar_det::AntennaPatternModelType AntennaPatternModelFromString(
    const std::string& s) {
  if (s == "kGaussianMainLobe")
    return ar_det::AntennaPatternModelType::kGaussianMainLobe;
  return ar_det::AntennaPatternModelType::kGaussianMainLobe;
}

inline airborne_radar::model::RadarWorkSubMode WorkSubModeFromString(
    const std::string& s) {
  if (s == "kStby") return airborne_radar::model::RadarWorkSubMode::kStby;
  if (s == "kTas") return airborne_radar::model::RadarWorkSubMode::kTas;
  if (s == "kTws") return airborne_radar::model::RadarWorkSubMode::kTws;
  if (s == "kStt") return airborne_radar::model::RadarWorkSubMode::kStt;
  return airborne_radar::model::RadarWorkSubMode::kTas;
}

inline airborne_radar::model::StabilizationMode StabilizationFromString(
    const std::string& s) {
  if (s == "kBodyStabilized")
    return airborne_radar::model::StabilizationMode::kBodyStabilized;
  if (s == "kInertialStabilized")
    return airborne_radar::model::StabilizationMode::kInertialStabilized;
  if (s == "kGroundStabilized")
    return airborne_radar::model::StabilizationMode::kGroundStabilized;
  return airborne_radar::model::StabilizationMode::kBodyStabilized;
}

inline oneq::foundation::ScanStartPosition ScanStartFromString(
    const std::string& s) {
  if (s == "kLeftTop") return oneq::foundation::ScanStartPosition::kLeftTop;
  if (s == "kRightTop") return oneq::foundation::ScanStartPosition::kRightTop;
  if (s == "kRightBottom") return oneq::foundation::ScanStartPosition::kRightBottom;
  if (s == "kLeftBottom") return oneq::foundation::ScanStartPosition::kLeftBottom;
  return oneq::foundation::ScanStartPosition::kLeftTop;
}

inline oneq::foundation::ScanSequence ScanSequenceFromString(
    const std::string& s) {
  if (s == "kAzimuthFirst")
    return oneq::foundation::ScanSequence::kAzimuthFirst;
  if (s == "kElevationFirst")
    return oneq::foundation::ScanSequence::kElevationFirst;
  return oneq::foundation::ScanSequence::kAzimuthFirst;
}

inline airborne_radar::config::KalmanUpdateBackend KalmanBackendFromString(
    const std::string& s) {
  if (s == "kStandardKfJoseph")
    return airborne_radar::config::KalmanUpdateBackend::kStandardKfJoseph;
  if (s == "kUdKf") return airborne_radar::config::KalmanUpdateBackend::kUdKf;
  if (s == "kSrif") return airborne_radar::config::KalmanUpdateBackend::kSrif;
  if (s == "kEkf") return airborne_radar::config::KalmanUpdateBackend::kEkf;
  return airborne_radar::config::KalmanUpdateBackend::kStandardKfJoseph;
}

inline airborne_radar::environment::JammingSensitivityProfile JammingSensFromString(
    const std::string& s) {
  using namespace airborne_radar::environment;
  if (s == "kRelaxed") return JammingSensitivityProfile::kRelaxed;
  if (s == "kBalanced") return JammingSensitivityProfile::kBalanced;
  if (s == "kStrict") return JammingSensitivityProfile::kStrict;
  return JammingSensitivityProfile::kBalanced;
}

inline airborne_radar::environment::VegetationCoverProfile VegCoverFromString(
    const std::string& s) {
  using namespace airborne_radar::environment;
  if (s == "kDisabled") return VegetationCoverProfile::kDisabled;
  if (s == "kOpenGrassland") return VegetationCoverProfile::kOpenGrassland;
  if (s == "kSparseWoodland") return VegetationCoverProfile::kSparseWoodland;
  if (s == "kDeciduousForest") return VegetationCoverProfile::kDeciduousForest;
  if (s == "kConiferousForest") return VegetationCoverProfile::kConiferousForest;
  if (s == "kTropicalDense") return VegetationCoverProfile::kTropicalDense;
  return VegetationCoverProfile::kDisabled;
}

// -- struct loaders ----------------------------------------------------------

inline void LoadAzEl(const oneq::JsonValue& j,
                     airborne_radar::model::AzimuthElevationDeg* v) {
  if (j.IsNull()) return;
  v->az_deg = static_cast<float>(j["az_deg"].AsDouble());
  v->el_deg = static_cast<float>(j["el_deg"].AsDouble());
}

inline void LoadAzElLimits(const oneq::JsonValue& j,
                           airborne_radar::model::AzimuthElevationLimitsDeg* v) {
  if (j.IsNull()) return;
  v->az_min_deg = static_cast<float>(j["az_min_deg"].AsDouble());
  v->az_max_deg = static_cast<float>(j["az_max_deg"].AsDouble());
  v->el_min_deg = static_cast<float>(j["el_min_deg"].AsDouble());
  v->el_max_deg = static_cast<float>(j["el_max_deg"].AsDouble());
}

inline void LoadCmdBeamwidth(const oneq::JsonValue& j,
                             airborne_radar::model::CommandedBeamwidthDeg* v) {
  if (j.IsNull()) return;
  v->commanded_az_beamwidth_deg =
      static_cast<float>(j["commanded_az_beamwidth_deg"].AsDouble());
  v->commanded_el_beamwidth_deg =
      static_cast<float>(j["commanded_el_beamwidth_deg"].AsDouble());
}

inline void LoadTransmitter(const oneq::JsonValue& j, ar_det::TransmitterConfig* v) {
  if (j.IsNull()) return;
  v->peak_power_w = static_cast<float>(j["peak_power_w"].AsDouble());
  v->frequency_hz = static_cast<float>(j["frequency_hz"].AsDouble());
  v->bandwidth_hz = static_cast<float>(j["bandwidth_hz"].AsDouble());
  v->pulse_width_s = static_cast<float>(j["pulse_width_s"].AsDouble());
  v->prf_hz = static_cast<float>(j["prf_hz"].AsDouble());
  v->transmit_loss_db = static_cast<float>(j["transmit_loss_db"].AsDouble());
}

inline void LoadAntennaPattern(const oneq::JsonValue& j,
                               ar_det::AntennaPatternConfig* v) {
  if (j.IsNull()) return;
  v->model_type = AntennaPatternModelFromString(j["model_type"].AsString());
  v->max_sidelobe_level_db = static_cast<float>(j["max_sidelobe_level_db"].AsDouble());
  v->backlobe_level_db = static_cast<float>(j["backlobe_level_db"].AsDouble());
  v->scan_loss_coeff_db_per_deg2 =
      static_cast<float>(j["scan_loss_coeff_db_per_deg2"].AsDouble());
  v->max_scan_loss_db = static_cast<float>(j["max_scan_loss_db"].AsDouble());
  LoadAzEl(j["boresight_offset_deg"], &v->boresight_offset_deg);
}

inline void LoadAntenna(const oneq::JsonValue& j, ar_det::AntennaConfig* v) {
  if (j.IsNull()) return;
  v->main_beam_gain_db = static_cast<float>(j["main_beam_gain_db"].AsDouble());
  v->nominal_az_beamwidth_deg =
      static_cast<float>(j["nominal_az_beamwidth_deg"].AsDouble());
  v->nominal_el_beamwidth_deg =
      static_cast<float>(j["nominal_el_beamwidth_deg"].AsDouble());
  LoadAntennaPattern(j["pattern"], &v->pattern);
  v->enable_directional_pattern = j["enable_directional_pattern"].AsBool();
}

inline void LoadReceiver(const oneq::JsonValue& j, ar_det::ReceiverConfig* v) {
  if (j.IsNull()) return;
  v->noise_figure_db = static_cast<float>(j["noise_figure_db"].AsDouble());
  v->receive_loss_db = static_cast<float>(j["receive_loss_db"].AsDouble());
}

inline void LoadDetectionPolicy(const oneq::JsonValue& j,
                                ar_det::DetectionPolicyConfig* v) {
  if (j.IsNull()) return;
  v->cfar_pfa = static_cast<float>(j["cfar_pfa"].AsDouble());
  v->min_snr_db = static_cast<float>(j["min_snr_db"].AsDouble());
}

inline void LoadRcsPhysics(const oneq::JsonValue& j, ar_det::RcsPhysicsConfig* v) {
  if (j.IsNull()) return;
  v->enable_physical_rcs = j["enable_physical_rcs"].AsBool();
  v->frequency_hz = static_cast<float>(j["frequency_hz"].AsDouble());
  v->physics_mix_ratio = static_cast<float>(j["physics_mix_ratio"].AsDouble());
  v->cylinder_weight = static_cast<float>(j["cylinder_weight"].AsDouble());
  v->min_equivalent_radius_m = static_cast<float>(j["min_equivalent_radius_m"].AsDouble());
  v->max_equivalent_radius_m = static_cast<float>(j["max_equivalent_radius_m"].AsDouble());
  v->min_rcs_m2 = static_cast<float>(j["min_rcs_m2"].AsDouble());
  v->max_rcs_m2 = static_cast<float>(j["max_rcs_m2"].AsDouble());
  v->bistatic_psi_offset_deg = static_cast<float>(j["bistatic_psi_offset_deg"].AsDouble());
}

inline void LoadDetectionConfig(const oneq::JsonValue& j,
                                ar_det::DetectionConfig* v) {
  if (j.IsNull()) return;
  v->enable_physics_detection = j["enable_physics_detection"].AsBool();
  LoadTransmitter(j["transmitter"], &v->transmitter);
  LoadAntenna(j["antenna"], &v->antenna);
  LoadReceiver(j["receiver"], &v->receiver);
  LoadDetectionPolicy(j["detection_policy"], &v->detection_policy);
  LoadRcsPhysics(j["rcs_physics"], &v->rcs_physics);
  v->min_detection_margin_db =
      static_cast<float>(j["min_detection_margin_db"].AsDouble());
  v->pulse_count = static_cast<int>(j["pulse_count"].AsInt());
  v->swerling_model = SwerlingModelFromString(j["swerling_model"].AsString());
}

inline void LoadHardware(const oneq::JsonValue& j,
                         airborne_radar::config::RadarHardwareConfig* v) {
  if (j.IsNull()) return;
  LoadDetectionConfig(j["detection"], &v->detection);
}

inline void LoadEulerAngles(const oneq::JsonValue& j,
                            airborne_radar::model::EulerAnglesDeg* v) {
  if (j.IsNull()) return;
  v->yaw_deg = static_cast<float>(j["yaw_deg"].AsDouble());
  v->pitch_deg = static_cast<float>(j["pitch_deg"].AsDouble());
  v->roll_deg = static_cast<float>(j["roll_deg"].AsDouble());
}

inline void LoadOrientation(const oneq::JsonValue& j,
                            airborne_radar::model::RadarOrientationConfig* v) {
  if (j.IsNull()) return;
  LoadEulerAngles(j["mount_angles_deg"], &v->mount_angles_deg);
  LoadAzEl(j["scan_center_deg"], &v->scan_center_deg);
  LoadAzElLimits(j["mechanical_scan_limits_deg"], &v->mechanical_scan_limits_deg);
  LoadAzElLimits(j["electronic_scan_limits_deg"], &v->electronic_scan_limits_deg);
  v->scan_start_position = ScanStartFromString(j["scan_start_position"].AsString());
  v->scan_sequence = ScanSequenceFromString(j["scan_sequence"].AsString());
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

/// Load an airborne_radar::session::RadarSessionConfig from a parsed JSON object.
inline void LoadArSessionConfig(
    const oneq::JsonValue& root,
    airborne_radar::session::RadarSessionConfig* config) {
  LoadHardware(root["hardware"], &config->hardware);
  LoadMission(root["mission"], &config->mission);
  LoadPolicy(root["policy"], &config->policy);
  LoadEnvironment(root["environment"], &config->environment);
  config->jamming_sensitivity_profile =
      JammingSensFromString(root["jamming_sensitivity_profile"].AsString());
}

/// Load an airborne_radar::session::RadarSessionConfig from a JSON file.
inline bool LoadArSessionConfigFromFile(
    const char* path, airborne_radar::session::RadarSessionConfig* config,
    std::string* error_msg) {
  oneq::JsonValue root;
  if (!oneq::JsonReader::ParseFile(path, &root, error_msg)) return false;
  if (root.type() != oneq::JsonValue::kObject) {
    *error_msg = "root value must be a JSON object";
    return false;
  }
  LoadArSessionConfig(root, config);
  return true;
}

}  // namespace examples

#endif  // EXAMPLES_AR_CONFIG_LOADER_H_
