#include "airborne_radar/signal/pipeline/DecisionFrameBuilders.h"

#include <algorithm>

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

namespace {

float ResolveAssociationFragilityWeight(common::JammingSemantic semantic) {
  switch (semantic) {
    case common::JammingSemantic::kDeception:
      return 1.00f;
    case common::JammingSemantic::kRepeater:
      return 0.88f;
    case common::JammingSemantic::kMixed:
      return 0.94f;
    case common::JammingSemantic::kNoiseSuppression:
      return 0.60f;
    case common::JammingSemantic::kNone:
    default:
      return 0.0f;
  }
}

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

AssociationQualityMetrics ToPipelineAssociationQualityMetrics(
    const association::AssociationQualityMetrics& source,
    common::JammingSemantic dominant_jamming_semantic, float jamming_severity,
    float association_unassigned_cost) {
  AssociationQualityMetrics metrics;
  metrics.prior_track_count = source.prior_track_count;
  metrics.detection_count = source.detection_count;
  metrics.matched_count = source.matched_count;
  metrics.new_track_count = source.new_track_count;
  metrics.missed_track_count = source.missed_track_count;
  metrics.match_rate = source.match_rate;
  metrics.new_track_rate = source.new_track_rate;
  metrics.missed_track_rate = source.missed_track_rate;
  metrics.mean_match_cost = source.mean_match_cost;
  metrics.p95_match_cost = source.p95_match_cost;
  metrics.dominant_jamming_semantic = dominant_jamming_semantic;
  metrics.jamming_severity = std::max(0.0f, std::min(1.0f, jamming_severity));
  const float normalized_cost_pressure =
      association_unassigned_cost > 1e-6f
          ? std::max(0.0f, std::min(1.0f, source.mean_match_cost / association_unassigned_cost))
          : 0.0f;
  const float operational_pressure =
      0.20f + 0.30f * std::max(0.0f, std::min(1.0f, 1.0f - source.match_rate)) +
      0.20f * source.new_track_rate + 0.15f * source.missed_track_rate +
      0.15f * normalized_cost_pressure;
  metrics.association_stress = std::max(
      0.0f,
      std::min(1.0f, metrics.jamming_severity *
                         ResolveAssociationFragilityWeight(metrics.dominant_jamming_semantic) *
                         operational_pressure));
  return metrics;
}

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
