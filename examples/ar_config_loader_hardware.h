#ifndef EXAMPLES_AR_CONFIG_LOADER_HARDWARE_H_
#define EXAMPLES_AR_CONFIG_LOADER_HARDWARE_H_

#include "1q/airborne_radar/airborne_radar.hpp"
#include "1q/foundation/json_reader.h"
#include "ar_config_loader_common.h"

namespace examples {

namespace ar_det = airborne_radar::config::detection;

// -- hardware / detection sub-tree -------------------------------------------

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
  v->model_type = ar_det::AntennaPatternModelType::kGaussianMainLobe;
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

}  // namespace examples

#endif  // EXAMPLES_AR_CONFIG_LOADER_HARDWARE_H_
