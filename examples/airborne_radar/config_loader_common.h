#ifndef EXAMPLES_AR_CONFIG_LOADER_COMMON_H_
#define EXAMPLES_AR_CONFIG_LOADER_COMMON_H_

#include <string>

#include "1q/airborne_radar/airborne_radar.hpp"
#include "json_reader.h"

namespace examples {

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

inline airborne_radar::config::ArWorkMode WorkModeFromString(
    const std::string& s) {
  if (s == "kStby") return airborne_radar::config::ArWorkMode::kStby;
  if (s == "kTas") return airborne_radar::config::ArWorkMode::kTas;
  if (s == "kTws") return airborne_radar::config::ArWorkMode::kTws;
  if (s == "kStt") return airborne_radar::config::ArWorkMode::kStt;
  return airborne_radar::config::ArWorkMode::kTas;
}

inline airborne_radar::config::StabilizationMode StabilizationFromString(
    const std::string& s) {
  if (s == "kBodyStabilized")
    return airborne_radar::config::StabilizationMode::kBodyStabilized;
  if (s == "kInertialStabilized")
    return airborne_radar::config::StabilizationMode::kInertialStabilized;
  if (s == "kGroundStabilized")
    return airborne_radar::config::StabilizationMode::kGroundStabilized;
  return airborne_radar::config::StabilizationMode::kBodyStabilized;
}

inline airborne_radar::config::KalmanUpdateBackend KalmanBackendFromString(
    const std::string& s) {
  if (s == "kStandardKfJoseph")
    return airborne_radar::config::KalmanUpdateBackend::kStandardKfJoseph;
  return airborne_radar::config::KalmanUpdateBackend::kStandardKfJoseph;
}

inline airborne_radar::config::JammingSensitivityProfile JammingSensFromString(
    const std::string& s) {
  using namespace airborne_radar::config;
  if (s == "kRelaxed") return JammingSensitivityProfile::kRelaxed;
  if (s == "kBalanced") return JammingSensitivityProfile::kBalanced;
  if (s == "kStrict") return JammingSensitivityProfile::kStrict;
  return JammingSensitivityProfile::kBalanced;
}

inline airborne_radar::config::VegetationCoverProfile VegCoverFromString(
    const std::string& s) {
  using namespace airborne_radar::config;
  if (s == "kDisabled") return VegetationCoverProfile::kDisabled;
  if (s == "kOpenGrassland") return VegetationCoverProfile::kOpenGrassland;
  if (s == "kSparseWoodland") return VegetationCoverProfile::kSparseWoodland;
  if (s == "kDeciduousForest") return VegetationCoverProfile::kDeciduousForest;
  if (s == "kConiferousForest") return VegetationCoverProfile::kConiferousForest;
  if (s == "kTropicalDense") return VegetationCoverProfile::kTropicalDense;
  return VegetationCoverProfile::kDisabled;
}

// -- common geometry helpers -------------------------------------------------

inline void LoadAzEl(const examples::JsonValue& j,
                     airborne_radar::config::AzimuthElevationDeg* v) {
  if (j.IsNull()) return;
  v->az_deg = static_cast<float>(j["az_deg"].AsDouble());
  v->el_deg = static_cast<float>(j["el_deg"].AsDouble());
}

inline void LoadAzElLimits(const examples::JsonValue& j,
                           airborne_radar::config::AzimuthElevationLimitsDeg* v) {
  if (j.IsNull()) return;
  v->az_min_deg = static_cast<float>(j["az_min_deg"].AsDouble());
  v->az_max_deg = static_cast<float>(j["az_max_deg"].AsDouble());
  v->el_min_deg = static_cast<float>(j["el_min_deg"].AsDouble());
  v->el_max_deg = static_cast<float>(j["el_max_deg"].AsDouble());
}

inline void LoadCmdBeamwidth(const examples::JsonValue& j,
                             airborne_radar::config::CommandedBeamwidthDeg* v) {
  if (j.IsNull()) return;
  v->commanded_az_beamwidth_deg =
      static_cast<float>(j["commanded_az_beamwidth_deg"].AsDouble());
  v->commanded_el_beamwidth_deg =
      static_cast<float>(j["commanded_el_beamwidth_deg"].AsDouble());
}

inline void LoadEulerAngles(const examples::JsonValue& j,
                            airborne_radar::config::EulerAnglesDeg* v) {
  if (j.IsNull()) return;
  v->yaw_deg = static_cast<float>(j["yaw_deg"].AsDouble());
  v->pitch_deg = static_cast<float>(j["pitch_deg"].AsDouble());
  v->roll_deg = static_cast<float>(j["roll_deg"].AsDouble());
}

}  // namespace examples

#endif  // EXAMPLES_AR_CONFIG_LOADER_COMMON_H_
