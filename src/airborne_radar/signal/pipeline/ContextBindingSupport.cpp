#include "airborne_radar/signal/pipeline/ContextBindingSupport.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

CycleWorkspace BuildCycleWorkspaceBindings(
    common::TargetFeatureList* output_state, common::DecisionInputFrame* decision_frame,
    AssociationQualityMetrics* association_quality_metrics,
    std::vector<tracking::TrackMeasurement>* track_measurements,
    std::vector<float>* signal_term_db, std::vector<float>* speed_penalty_db,
    std::vector<float>* detection_margin_db, std::vector<std::uint8_t>* detection_succeeded,
    std::vector<std::uint64_t>* association_keys, std::vector<int>* measurement_slots,
    std::vector<detection::ResolvedTargetGeometry>* target_geometry,
    std::vector<tracking::MeasurementCovariance>* measurement_covariances,
    association::AssociationResult* association_result) {
  CycleWorkspace workspace;
  workspace.output_state = output_state;
  workspace.decision_frame = decision_frame;
  workspace.association_quality_metrics = association_quality_metrics;
  workspace.track_measurements = track_measurements;
  workspace.signal_term_db = signal_term_db;
  workspace.speed_penalty_db = speed_penalty_db;
  workspace.detection_margin_db = detection_margin_db;
  workspace.detection_succeeded = detection_succeeded;
  workspace.association_keys = association_keys;
  workspace.measurement_slots = measurement_slots;
  workspace.target_geometry = target_geometry;
  workspace.measurement_covariances = measurement_covariances;
  workspace.association_result = association_result;
  return workspace;
}

DetectionExecutionBuffers BuildDetectionExecutionBuffers(
    std::vector<detection::ResolvedTargetGeometry>* target_geometry,
    std::vector<float>* signal_term_db, std::vector<float>* speed_penalty_db,
    std::vector<float>* detection_margin_db, std::vector<std::uint8_t>* detection_succeeded,
    std::vector<tracking::MeasurementCovariance>* measurement_covariances) {
  DetectionExecutionBuffers buffers;
  buffers.target_geometry = target_geometry;
  buffers.signal_term_db = signal_term_db;
  buffers.speed_penalty_db = speed_penalty_db;
  buffers.detection_margin_db = detection_margin_db;
  buffers.detection_succeeded = detection_succeeded;
  buffers.measurement_covariances = measurement_covariances;
  return buffers;
}

TrackMeasurementBuildContext BuildTrackMeasurementBuildContextBindings(
    const common::TargetFeatureList* input,
    const association::AssociationResult* association_result,
    const std::vector<std::uint8_t>* detection_succeeded,
    const std::vector<std::uint64_t>* association_keys,
    const std::vector<float>* detection_margin_db,
    const std::vector<detection::ResolvedTargetGeometry>* target_geometry,
    const std::vector<tracking::MeasurementCovariance>* measurement_covariances,
    bool jamming_detected, common::JammingSemantic dominant_jamming_semantic, float jamming_severity,
    std::vector<int>* measurement_slots,
    std::vector<tracking::TrackMeasurement>* track_measurements) {
  TrackMeasurementBuildContext context;
  context.input = input;
  context.association_result = association_result;
  context.detection_succeeded = detection_succeeded;
  context.association_keys = association_keys;
  context.detection_margin_db = detection_margin_db;
  context.target_geometry = target_geometry;
  context.measurement_covariances = measurement_covariances;
  context.jamming_detected = jamming_detected;
  context.dominant_jamming_semantic = dominant_jamming_semantic;
  context.jamming_severity = jamming_severity;
  context.measurement_slots = measurement_slots;
  context.track_measurements = track_measurements;
  return context;
}

TrackFilterApplyContext BuildTrackFilterApplyContextBindings(
    const common::TargetFeatureList* input, common::TargetFeatureList* output,
    const std::vector<std::uint8_t>* detection_succeeded,
    const std::vector<float>* detection_margin_db, bool jamming_detected,
    common::JammingSemantic dominant_jamming_semantic, float jamming_severity,
    tracking::TrackFilter* track_filter, const std::vector<int>* measurement_slots,
    std::vector<tracking::TrackMeasurement>* track_measurements) {
  TrackFilterApplyContext context;
  context.input = input;
  context.output = output;
  context.detection_succeeded = detection_succeeded;
  context.detection_margin_db = detection_margin_db;
  context.jamming_detected = jamming_detected;
  context.dominant_jamming_semantic = dominant_jamming_semantic;
  context.jamming_severity = jamming_severity;
  context.track_filter = track_filter;
  context.measurement_slots = measurement_slots;
  context.track_measurements = track_measurements;
  return context;
}

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
