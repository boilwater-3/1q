#ifndef EXAMPLES_RIR_CONFIG_LOADER_COMMON_H_
#define EXAMPLES_RIR_CONFIG_LOADER_COMMON_H_

#include <string>

#include "1q/electromagnetics/RfScene.h"
#include "1q/foundation/scan_schedule_types.h"
#include "1q/remote_identification_radar/remote_identification_radar.hpp"
#include "json_reader.h"

namespace examples {

namespace rir_cfg = remote_identification_radar::config;

inline rir_cfg::RirWorkMode RirWorkModeFromString(const std::string& s) {
  if (s == "kStby") return rir_cfg::RirWorkMode::kStby;
  if (s == "kIdentify") return rir_cfg::RirWorkMode::kIdentify;
  return rir_cfg::RirWorkMode::kStby;
}

inline rir_cfg::RirDetectionGateMode RirDetectionGateModeFromString(const std::string& s) {
  if (s == "kDetectorGate") return rir_cfg::RirDetectionGateMode::kDetectorGate;
  if (s == "kSnrFallback") return rir_cfg::RirDetectionGateMode::kSnrFallback;
  return rir_cfg::RirDetectionGateMode::kDetectorGate;
}

inline rir_cfg::hardware::RirAntennaPatternModelType RirAntennaPatternModelTypeFromString(
    const std::string& s) {
  using namespace rir_cfg::hardware;
  if (s == "kGaussianMainLobe") return RirAntennaPatternModelType::kGaussianMainLobe;
  if (s == "kCosinePower") return RirAntennaPatternModelType::kCosinePower;
  if (s == "kSincPattern") return RirAntennaPatternModelType::kSincPattern;
  return RirAntennaPatternModelType::kGaussianMainLobe;
}

inline oneq::electromagnetics::RfScenePolarization RfScenePolarizationFromString(
    const std::string& s) {
  using namespace oneq::electromagnetics;
  if (s == "kHorizontal") return RfScenePolarization::kHorizontal;
  if (s == "kVertical") return RfScenePolarization::kVertical;
  if (s == "kRightHandCircular") return RfScenePolarization::kRightHandCircular;
  if (s == "kLeftHandCircular") return RfScenePolarization::kLeftHandCircular;
  if (s == "kUnpolarized") return RfScenePolarization::kUnpolarized;
  if (s == "kFullPolarization") return RfScenePolarization::kFullPolarization;
  return RfScenePolarization::kHorizontal;
}

inline rir_cfg::RirVegetationCoverProfile RirVegetationCoverProfileFromString(
    const std::string& s) {
  if (s == "kDisabled") return rir_cfg::RirVegetationCoverProfile::kDisabled;
  if (s == "kOpenGrassland") return rir_cfg::RirVegetationCoverProfile::kOpenGrassland;
  if (s == "kSparseWoodland") return rir_cfg::RirVegetationCoverProfile::kSparseWoodland;
  if (s == "kDeciduousForest") return rir_cfg::RirVegetationCoverProfile::kDeciduousForest;
  if (s == "kConiferousForest") return rir_cfg::RirVegetationCoverProfile::kConiferousForest;
  if (s == "kTropicalDense") return rir_cfg::RirVegetationCoverProfile::kTropicalDense;
  return rir_cfg::RirVegetationCoverProfile::kDisabled;
}

inline oneq::foundation::ScanStartPosition ScanStartPositionFromString(const std::string& s) {
  using namespace oneq::foundation;
  if (s == "kLeftTop") return ScanStartPosition::kLeftTop;
  if (s == "kRightTop") return ScanStartPosition::kRightTop;
  if (s == "kRightBottom") return ScanStartPosition::kRightBottom;
  if (s == "kLeftBottom") return ScanStartPosition::kLeftBottom;
  return ScanStartPosition::kLeftTop;
}

inline oneq::foundation::ScanSequence ScanSequenceFromString(const std::string& s) {
  using namespace oneq::foundation;
  if (s == "kAzimuthFirst") return ScanSequence::kAzimuthFirst;
  if (s == "kElevationFirst") return ScanSequence::kElevationFirst;
  return ScanSequence::kAzimuthFirst;
}

inline void LoadRirAzEl(const examples::JsonValue& j, rir_cfg::RirAzimuthElevationDeg* v) {
  if (j.IsNull()) return;
  if (j.Has("az_deg")) {
    v->az_deg = static_cast<float>(j["az_deg"].AsDouble());
  }
  if (j.Has("el_deg")) {
    v->el_deg = static_cast<float>(j["el_deg"].AsDouble());
  }
}

inline void LoadRirAzElLimits(const examples::JsonValue& j,
                              rir_cfg::RirAzimuthElevationLimitsDeg* v) {
  if (j.IsNull()) return;
  if (j.Has("az_min_deg")) {
    v->az_min_deg = static_cast<float>(j["az_min_deg"].AsDouble());
  }
  if (j.Has("az_max_deg")) {
    v->az_max_deg = static_cast<float>(j["az_max_deg"].AsDouble());
  }
  if (j.Has("el_min_deg")) {
    v->el_min_deg = static_cast<float>(j["el_min_deg"].AsDouble());
  }
  if (j.Has("el_max_deg")) {
    v->el_max_deg = static_cast<float>(j["el_max_deg"].AsDouble());
  }
}

}  // namespace examples

#endif  // EXAMPLES_RIR_CONFIG_LOADER_COMMON_H_
