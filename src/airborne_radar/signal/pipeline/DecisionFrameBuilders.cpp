#include "airborne_radar/signal/pipeline/DecisionFrameBuilders.h"

#include <algorithm>

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

namespace {

/** @brief 将环境层干扰源事实转换为 ECCM 干扰源信息。
 *  @param environment_source 环境快照中的干扰源事实。
 *  @return 填充后的 ECCM 干扰源信息。 */
common::EccmJammerSourceInfo BuildEccmJammerSourceInfo(
    const environment::JammerSourceFact& environment_source) {
  common::EccmJammerSourceInfo source_info;
  source_info.technique = environment_source.technique;
  source_info.jammer_power_db = environment_source.power_db;
  source_info.jammer_to_signal_db = environment_source.js_db;
  source_info.frequency_overlap_ratio = environment_source.frequency_overlap_ratio;
  source_info.prf_lock_risk = environment_source.prf_lock_risk;
  source_info.azimuth_deg = environment_source.azimuth_deg;
  source_info.elevation_deg = environment_source.elevation_deg;
  source_info.angular_span_deg = environment_source.angular_span_deg;
  source_info.jammer_in_sidelobe = environment_source.in_sidelobe;
  source_info.confidence = environment_source.confidence;
  return source_info;
}

/** @brief 从目标特征构建决策航迹快照。
 *  @param feature 目标特征数据。
 *  @param index 目标在列表中的索引，用于生成无外部 ID 时的关联键。
 *  @return 构建后的决策航迹快照，状态默认为 kConfirmed。 */
common::DecisionTrackSnapshot BuildDecisionTrackSnapshotFromFeature(
    const common::TargetFeature& feature, std::size_t index) {
  const std::uint64_t association_key = feature.external_target_id != 0U
                                            ? feature.external_target_id
                                            : static_cast<std::uint64_t>(index + 1U);
  common::DecisionTrackSnapshot snapshot(
      feature.current_track_velocity_x, feature.current_track_velocity_y,
      feature.current_track_velocity_z, feature.current_track_rcs, 0.0f, 0.0f, 0.0f, false,
      feature.external_target_id, association_key);
  snapshot.state.status = common::DecisionTrackStatus::kConfirmed;
  snapshot.state.position_x = feature.position_x;
  snapshot.state.position_y = feature.position_y;
  snapshot.state.position_z = feature.position_z;
  snapshot.evidence.has_measurement_evidence = true;
  snapshot.evidence.updated_this_cycle = true;
  snapshot.evidence.predicted_only_this_cycle = false;
  return snapshot;
}

}  // namespace

common::EccmSourceInfo BuildEccmSourceInfo(
    const environment::EnvironmentSnapshot& environment_snapshot) {
  common::EccmSourceInfo source_info;
  source_info.has_jamming_signal = environment_snapshot.jamming_detected;
  source_info.jammer_power_db = environment_snapshot.jammer_power_db;
  source_info.frequency_overlap_ratio = environment_snapshot.jammer_frequency_overlap_ratio;
  source_info.prf_lock_risk = environment_snapshot.jammer_prf_lock_risk;
  source_info.jammer_in_sidelobe = environment_snapshot.jammer_in_sidelobe;
  source_info.jammer_sources.reserve(environment_snapshot.jammer_sources.size());
  for (std::size_t i = 0; i < environment_snapshot.jammer_sources.size(); ++i) {
    source_info.jammer_sources.push_back(
        BuildEccmJammerSourceInfo(environment_snapshot.jammer_sources[i]));
  }
  return source_info;
}

common::AssociationQualityInfo BuildAssociationQualityInfo(
    const AssociationQualityMetrics& metrics) {
  common::AssociationQualityInfo info;
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

common::PerceptionQualityInfo BuildPerceptionQualityInfo(std::size_t input_target_count,
                                                         const AssociationQualityMetrics& metrics) {
  common::PerceptionQualityInfo info;
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

common::DecisionTrackSnapshotList BuildDecisionSnapshotsFromFeatures(
    const common::TargetFeatureList& features) {
  common::DecisionTrackSnapshotList track_snapshots;
  track_snapshots.reserve(features.size());
  for (std::size_t i = 0; i < features.size(); ++i) {
    track_snapshots.push_back(BuildDecisionTrackSnapshotFromFeature(features[i], i));
  }
  return track_snapshots;
}

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
