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
  set.valid_feature_mask = BuildFeatureMask(set);
  return set;
}

}  // namespace recognition
}  // namespace airborne_radar
