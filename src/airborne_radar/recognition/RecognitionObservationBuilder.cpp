/**
 * @file RecognitionObservationBuilder.cpp
 * @brief 识别观测构造器实现。
 */

#include "airborne_radar/recognition/RecognitionObservationBuilder.h"

#include "airborne_radar/recognition/MotionFeatureExtractor.h"
#include "airborne_radar/recognition/PolarizationFeatureExtractor.h"
#include "airborne_radar/recognition/RangeProfileFeatureExtractor.h"
#include "airborne_radar/recognition/RcsFeatureExtractor.h"

namespace airborne_radar {
namespace recognition {

namespace {

/** @brief 驻留时间质量标称值（s）：识别配置默认 recognition_dwell_sec。 */
constexpr float kNominalDwellSec = 0.05f;

std::uint8_t BuildFeatureMask(const RecognitionFeatureSet& set) {
  std::uint8_t mask = 0U;
  if (set.rcs.valid && set.rcs.quality > 0.0f) {
    mask |= static_cast<std::uint8_t>(session::ArRecognitionFeatureDimension::kRcs);
  }
  if (set.motion.valid && set.motion.quality > 0.0f) {
    mask |= static_cast<std::uint8_t>(session::ArRecognitionFeatureDimension::kMotion);
  }
  if (set.polarization.valid && set.polarization.quality > 0.0f) {
    mask |= static_cast<std::uint8_t>(session::ArRecognitionFeatureDimension::kPolarization);
  }
  if (set.range_profile.valid && set.range_profile.quality > 0.0f) {
    mask |= static_cast<std::uint8_t>(session::ArRecognitionFeatureDimension::kRangeProfile);
  }
  return mask;
}

/** @brief 驻留质量因子：驻留越短，观测质量越低（效能级模型）。 */
float DwellQualityFactor(float dwell_sec) {
  if (!std::isfinite(dwell_sec) || dwell_sec <= 0.0f) {
    return 0.0f;
  }
  return std::max(0.0f, std::min(1.0f, dwell_sec / kNominalDwellSec));
}

}  // namespace

RecognitionFeatureSet RecognitionObservationBuilder::Build(
    const session::ArSceneTarget& target, const session::TrackStateSnapshot& snapshot,
    const RecognitionObservationContext& context) {
  RecognitionFeatureSet set;
  set.rcs = RcsFeatureExtractor::Extract(target.aspect_rcs_samples, context.look_az_deg,
                                         context.look_el_deg, context.snr_db,
                                         context.minimum_aspect_coverage_deg);
  set.motion = MotionFeatureExtractor::Extract(snapshot, context.platform_altitude_m,
                                               snapshot.estimation_uncertainty_trace);
  set.polarization =
      PolarizationFeatureExtractor::Extract(target.polarization_rcs_samples, context.look_az_deg,
                                            context.look_el_deg, context.snr_db, context.range_m);
  set.range_profile = RangeProfileFeatureExtractor::Extract(
      target.range_rcs_scatterers, context.bandwidth_hz, context.snr_db,
      context.max_range_resolution_m);
  // 驻留时间质量因子：作用于除运动外的观测维度（运动来自滤波航迹，与驻留无关）。
  const float dwell_factor = DwellQualityFactor(context.dwell_sec);
  set.rcs.quality *= dwell_factor;
  set.polarization.quality *= dwell_factor;
  set.range_profile.quality *= dwell_factor;
  set.valid_feature_mask = BuildFeatureMask(set);
  return set;
}

}  // namespace recognition
}  // namespace airborne_radar
