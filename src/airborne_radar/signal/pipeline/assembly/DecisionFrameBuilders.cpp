#include "airborne_radar/signal/pipeline/assembly/DecisionFrameBuilders.h"

#include <algorithm>

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

namespace {

/** @brief 将环境层干扰源事实转换为 ECCM 干扰源信息。
 *  @param environment_source 环境快照中的干扰源事实。
 *  @return 填充后的 ECCM 干扰源信息。 */
model::EccmJammerSourceInfo BuildEccmJammerSourceInfo(
    const environment::JammerSourceFact& environment_source) {
  model::EccmJammerSourceInfo source_info;
  source_info.technique = environment_source.technique;
  source_info.jammer_power_db = environment_source.power_db;
  source_info.jammer_to_signal_db = environment_source.js_db;
  source_info.frequency_overlap_ratio = environment_source.frequency_overlap_ratio;
  source_info.prf_lock_risk = environment_source.prf_lock_risk;
  source_info.has_direction_deg = environment_source.has_direction_deg;
  if (environment_source.has_direction_deg) {
    source_info.direction_deg.azimuth_deg = environment_source.direction_deg.azimuth_deg;
    source_info.direction_deg.elevation_deg = environment_source.direction_deg.elevation_deg;
  }
  source_info.angular_span_deg = environment_source.angular_span_deg;
  source_info.jammer_in_sidelobe = environment_source.in_sidelobe;
  source_info.confidence = environment_source.confidence;
  return source_info;
}

}  // namespace

model::EccmSourceInfo BuildEccmSourceInfo(
    const environment::EnvironmentSnapshot& environment_snapshot) {
  model::EccmSourceInfo source_info;
  source_info.has_jamming_signal = environment_snapshot.jamming_detected;
  source_info.jammer_sources.reserve(environment_snapshot.jammer_sources.size());
  for (std::size_t i = 0; i < environment_snapshot.jammer_sources.size(); ++i) {
    source_info.jammer_sources.push_back(
        BuildEccmJammerSourceInfo(environment_snapshot.jammer_sources[i]));
  }
  return source_info;
}

model::AssociationQualityInfo BuildAssociationQualityInfo(
    const AssociationQualityMetrics& metrics) {
  model::AssociationQualityInfo info;
  info.match_rate = metrics.match_rate;
  info.new_track_rate = metrics.new_track_rate;
  info.missed_track_rate = metrics.missed_track_rate;
  info.mean_match_cost = metrics.mean_match_cost;
  info.p95_match_cost = metrics.p95_match_cost;
  info.dominant_jamming_semantic = metrics.dominant_jamming_semantic;
  info.jamming_severity = metrics.jamming_severity;
  info.association_stress = metrics.association_stress;
  return info;
}

model::PerceptionQualityInfo BuildPerceptionQualityInfo(
    std::size_t input_target_count, const AssociationQualityMetrics& metrics) {
  model::PerceptionQualityInfo info;
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

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
