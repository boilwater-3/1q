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
  if (j.Has("peak_power_w")) {
    v->peak_power_w = static_cast<float>(j["peak_power_w"].AsDouble());
  }
  if (j.Has("frequency_hz")) {
    v->frequency_hz = static_cast<float>(j["frequency_hz"].AsDouble());
  }
  if (j.Has("bandwidth_hz")) {
    v->bandwidth_hz = static_cast<float>(j["bandwidth_hz"].AsDouble());
  }
  if (j.Has("pulse_width_s")) {
    v->pulse_width_s = static_cast<float>(j["pulse_width_s"].AsDouble());
  }
  if (j.Has("prf_hz")) {
    v->prf_hz = static_cast<float>(j["prf_hz"].AsDouble());
  }
  if (j.Has("transmit_loss_db")) {
    v->transmit_loss_db = static_cast<float>(j["transmit_loss_db"].AsDouble());
  }
}

inline void LoadAntennaPattern(const examples::JsonValue& j, ar_det::AntennaPatternConfig* v) {
  if (j.IsNull()) return;
  v->model_type = ar_det::AntennaPatternModelType::kGaussianMainLobe;
  if (j.Has("max_sidelobe_level_db")) {
    v->max_sidelobe_level_db = static_cast<float>(j["max_sidelobe_level_db"].AsDouble());
  }
  if (j.Has("backlobe_level_db")) {
    v->backlobe_level_db = static_cast<float>(j["backlobe_level_db"].AsDouble());
  }
  if (j.Has("scan_loss_coeff_db_per_deg2")) {
    v->scan_loss_coeff_db_per_deg2 = static_cast<float>(j["scan_loss_coeff_db_per_deg2"].AsDouble());
  }
  if (j.Has("max_scan_loss_db")) {
    v->max_scan_loss_db = static_cast<float>(j["max_scan_loss_db"].AsDouble());
  }
  LoadAzEl(j["boresight_offset_deg"], &v->boresight_offset_deg);
}

inline void LoadAntenna(const examples::JsonValue& j, ar_det::AntennaConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("main_beam_gain_db")) {
    v->main_beam_gain_db = static_cast<float>(j["main_beam_gain_db"].AsDouble());
  }
  if (j.Has("nominal_az_beamwidth_deg")) {
    v->nominal_az_beamwidth_deg = static_cast<float>(j["nominal_az_beamwidth_deg"].AsDouble());
  }
  if (j.Has("nominal_el_beamwidth_deg")) {
    v->nominal_el_beamwidth_deg = static_cast<float>(j["nominal_el_beamwidth_deg"].AsDouble());
  }
  LoadAntennaPattern(j["pattern"], &v->pattern);
  if (j.Has("enable_directional_pattern")) {
    v->enable_directional_pattern = j["enable_directional_pattern"].AsBool();
  }
}

inline void LoadReceiver(const examples::JsonValue& j, ar_det::ReceiverConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("noise_figure_db")) {
    v->noise_figure_db = static_cast<float>(j["noise_figure_db"].AsDouble());
  }
  if (j.Has("receive_loss_db")) {
    v->receive_loss_db = static_cast<float>(j["receive_loss_db"].AsDouble());
  }
}

inline void LoadRcsPhysics(const examples::JsonValue& j, ar_det::RcsPhysicsConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("enable_physical_rcs")) {
    v->enable_physical_rcs = j["enable_physical_rcs"].AsBool();
  }
  if (j.Has("physics_mix_ratio")) {
    v->physics_mix_ratio = static_cast<float>(j["physics_mix_ratio"].AsDouble());
  }
  if (j.Has("cylinder_weight")) {
    v->cylinder_weight = static_cast<float>(j["cylinder_weight"].AsDouble());
  }
  if (j.Has("min_equivalent_radius_m")) {
    v->min_equivalent_radius_m = static_cast<float>(j["min_equivalent_radius_m"].AsDouble());
  }
  if (j.Has("max_equivalent_radius_m")) {
    v->max_equivalent_radius_m = static_cast<float>(j["max_equivalent_radius_m"].AsDouble());
  }
  if (j.Has("min_rcs_m2")) {
    v->min_rcs_m2 = static_cast<float>(j["min_rcs_m2"].AsDouble());
  }
  if (j.Has("max_rcs_m2")) {
    v->max_rcs_m2 = static_cast<float>(j["max_rcs_m2"].AsDouble());
  }
  if (j.Has("bistatic_psi_offset_deg")) {
    v->bistatic_psi_offset_deg = static_cast<float>(j["bistatic_psi_offset_deg"].AsDouble());
  }
}

inline void LoadDetectionConfig(const examples::JsonValue& j, ar_det::DetectionConfig* v) {
  if (j.IsNull()) return;
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

// -- orientation (static) / mission (runtime) --------------------------------

inline oneq::foundation::ScanStartPosition ArScanStartFromString(const std::string& s) {
  if (s == "kLeftTop") return oneq::foundation::ScanStartPosition::kLeftTop;
  if (s == "kRightTop") return oneq::foundation::ScanStartPosition::kRightTop;
  if (s == "kLeftBottom") return oneq::foundation::ScanStartPosition::kLeftBottom;
  if (s == "kRightBottom") return oneq::foundation::ScanStartPosition::kRightBottom;
  return oneq::foundation::ScanStartPosition::kLeftTop;
}

inline oneq::foundation::ScanSequence ArScanSequenceFromString(const std::string& s) {
  if (s == "kAzimuthFirst") return oneq::foundation::ScanSequence::kAzimuthFirst;
  if (s == "kElevationFirst") return oneq::foundation::ScanSequence::kElevationFirst;
  return oneq::foundation::ScanSequence::kAzimuthFirst;
}

inline void LoadOrientation(const examples::JsonValue& j,
                            airborne_radar::config::ArOrientationConfig* v) {
  if (j.IsNull()) return;
  LoadEulerAngles(j["mount_angles_deg"], &v->mount_angles_deg);
  LoadAzElLimits(j["mechanical_scan_limits_deg"], &v->mechanical_scan_limits_deg);
  LoadAzElLimits(j["electronic_scan_limits_deg"], &v->electronic_scan_limits_deg);
  if (!j["scan_start_position"].IsNull()) {
    if (j["scan_start_position"].IsString()) {
      v->scan_start_position = ArScanStartFromString(j["scan_start_position"].AsString());
    } else {
      v->scan_start_position =
          static_cast<oneq::foundation::ScanStartPosition>(j["scan_start_position"].AsInt());
    }
  }
  if (!j["scan_sequence"].IsNull()) {
    if (j["scan_sequence"].IsString()) {
      v->scan_sequence = ArScanSequenceFromString(j["scan_sequence"].AsString());
    } else {
      v->scan_sequence = static_cast<oneq::foundation::ScanSequence>(j["scan_sequence"].AsInt());
    }
  }
  if (j.Has("stabilization_mode")) {
    v->stabilization_mode = StabilizationFromString(j["stabilization_mode"].AsString());
  }
}

inline void LoadMission(const examples::JsonValue& j, airborne_radar::config::ArMissionConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("work_mode")) {
    v->work_mode = WorkModeFromString(j["work_mode"].AsString());
  }
  LoadAzEl(j["scan_center_deg"], &v->scan_center_deg);
  if (j.Has("commanded_beamwidth_enabled")) {
    v->commanded_beamwidth_enabled = j["commanded_beamwidth_enabled"].AsBool();
  }
  LoadCmdBeamwidth(j["commanded_beamwidth_deg"], &v->commanded_beamwidth_deg);
}

// -- policy sub-tree ---------------------------------------------------------

inline void LoadDetectionPolicy(const examples::JsonValue& j,
                                airborne_radar::config::ArDetectionPolicyConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("minimum_snr_db")) {
    v->minimum_snr_db = static_cast<float>(j["minimum_snr_db"].AsDouble());
  }
  if (j.Has("pfa")) {
    v->pfa = static_cast<float>(j["pfa"].AsDouble());
  }
  if (j.Has("pulse_count")) {
    v->pulse_count = static_cast<int>(j["pulse_count"].AsInt());
  }
  if (j.Has("minimum_detection_margin_db")) {
    v->minimum_detection_margin_db = static_cast<float>(j["minimum_detection_margin_db"].AsDouble());
  }
}

inline void LoadBeamPointing(const examples::JsonValue& j,
                             airborne_radar::config::BeamPointingConfig* v) {
  if (j.IsNull()) return;
  LoadCmdBeamwidth(j["nominal_beamwidth_deg"], &v->nominal_beamwidth_deg);
}

inline void LoadBeamScheduler(const examples::JsonValue& j,
                              airborne_radar::config::BeamSchedulerConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("azimuth_step_count_hint")) {
    v->azimuth_step_count_hint = static_cast<std::uint32_t>(j["azimuth_step_count_hint"].AsInt());
  }
  if (j.Has("elevation_step_count_hint")) {
    v->elevation_step_count_hint = static_cast<std::uint32_t>(j["elevation_step_count_hint"].AsInt());
  }
  if (j.Has("prefer_dense_tas_sampling")) {
    v->prefer_dense_tas_sampling = j["prefer_dense_tas_sampling"].AsBool();
  }
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
  if (j.Has("distance_gate_sigma")) {
    v->distance_gate_sigma = static_cast<float>(j["distance_gate_sigma"].AsDouble());
  }
}

inline void LoadTracking(const examples::JsonValue& j, airborne_radar::config::TrackingConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("enable_kalman_filter")) {
    v->enable_kalman_filter = j["enable_kalman_filter"].AsBool();
  }
  if (j.Has("kalman_measurement_noise_std")) {
    v->kalman_measurement_noise_std =
        static_cast<float>(j["kalman_measurement_noise_std"].AsDouble());
  }
  if (j.Has("speed_decay_ratio_on_loss")) {
    v->speed_decay_ratio_on_loss = static_cast<float>(j["speed_decay_ratio_on_loss"].AsDouble());
  }
  if (j.Has("rcs_decay_ratio_on_loss")) {
    v->rcs_decay_ratio_on_loss = static_cast<float>(j["rcs_decay_ratio_on_loss"].AsDouble());
  }
}

inline void LoadLifecycle(const examples::JsonValue& j,
                          airborne_radar::config::LifecycleConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("confirm_hits")) {
    v->confirm_hits = static_cast<std::uint32_t>(j["confirm_hits"].AsInt());
  }
  if (j.Has("max_miss_before_lost")) {
    v->max_miss_before_lost = static_cast<std::uint32_t>(j["max_miss_before_lost"].AsInt());
  }
  if (j.Has("max_lost_cycles")) {
    v->max_lost_cycles = static_cast<std::uint32_t>(j["max_lost_cycles"].AsInt());
  }
  if (j.Has("enable_imm_lifecycle")) {
    v->enable_imm_lifecycle = j["enable_imm_lifecycle"].AsBool();
  }
  if (j.Has("model_count_hint")) {
    v->model_count_hint = static_cast<std::uint32_t>(j["model_count_hint"].AsInt());
  }
}

inline void LoadPolicy(const examples::JsonValue& j, airborne_radar::config::ArPolicyConfig* v) {
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
  if (j.Has("enable_physical_model")) {
    v->enable_physical_model = j["enable_physical_model"].AsBool();
  }
  if (j.Has("pressure_hpa")) {
    v->pressure_hpa = static_cast<float>(j["pressure_hpa"].AsDouble());
  }
  if (j.Has("temperature_k")) {
    v->temperature_k = static_cast<float>(j["temperature_k"].AsDouble());
  }
  if (j.Has("relative_humidity")) {
    v->relative_humidity = static_cast<float>(j["relative_humidity"].AsDouble());
  }
}

inline void LoadVegScatter(const examples::JsonValue& j,
                           airborne_radar::config::VegetationScatterPhysicsConfig* v) {
  if (j.IsNull()) return;
  if (j.Has("cover_profile")) {
    v->cover_profile = VegCoverFromString(j["cover_profile"].AsString());
  }
  if (j.Has("enable_physical_model")) {
    v->enable_physical_model = j["enable_physical_model"].AsBool();
  }
}

inline void LoadScenario(const examples::JsonValue& j,
                         airborne_radar::config::EnvironmentScenarioConfig* v) {
  if (j.IsNull()) return;
  LoadAtmosObservation(j["atmospheric_physics"], &v->atmospheric_physics);
  LoadVegScatter(j["vegetation_scatter_physics"], &v->vegetation_scatter_physics);
}

inline void LoadEnvironment(const examples::JsonValue& j,
                            airborne_radar::config::ArEnvironmentConfig* v) {
  if (j.IsNull()) return;
  LoadScenario(j["scenario_config"], &v->scenario_config);
}

}  // namespace examples

#endif  // EXAMPLES_AR_CONFIG_LOADER_DETAIL_H_
