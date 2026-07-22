#include "airborne_radar/signal/pipeline/DecisionFrameBuilders.h"

#include <algorithm>

namespace airborne_radar {
namespace signal {
namespace pipeline {

session::AssociationQualityInfo BuildAssociationQualityInfo(
    const AssociationQualityMetrics& metrics) {
  session::AssociationQualityInfo info;
  info.match_rate = metrics.match_rate;
  info.new_track_rate = metrics.new_track_rate;
  info.missed_track_rate = metrics.missed_track_rate;
  info.mean_match_cost = metrics.mean_match_cost;
  info.p95_match_cost = metrics.p95_match_cost;
  info.association_stress = metrics.association_stress;
  return info;
}

session::PerceptionQualityInfo BuildPerceptionQualityInfo(
    std::size_t input_target_count, const AssociationQualityMetrics& metrics) {
  session::PerceptionQualityInfo info;
  info.input_target_count = input_target_count;
  info.detection_count = metrics.detection_count;
  if (input_target_count == 0U) {
    info.detection_rate = 1.0f;
    info.detection_stress = 0.0f;
    return info;
  }

  info.detection_rate = std::min(
      1.0f, static_cast<float>(metrics.detection_count) / static_cast<float>(input_target_count));
  info.detection_stress = std::max(0.0f, 1.0f - info.detection_rate);
  return info;
}


}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
