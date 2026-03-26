/**
 * @file ContextBindingSupport.h
 * @brief 定义 SignalPipeline 各阶段上下文绑定的内部辅助函数。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CONTEXT_BINDING_SUPPORT_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CONTEXT_BINDING_SUPPORT_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/common/JammingSemantics.h"
#include "1q/airborne_radar/common/TargetFeature.h"
#include "1q/airborne_radar/signal/pipeline/SignalPipelineTypes.h"
#include "airborne_radar/signal/association/DataAssociation.h"
#include "airborne_radar/signal/detection/TargetGeometryResolver.h"
#include "airborne_radar/signal/pipeline/CycleContextSupport.h"
#include "airborne_radar/signal/pipeline/DetectionExecution.h"
#include "airborne_radar/signal/pipeline/TrackMeasurementProcessing.h"
#include "airborne_radar/signal/tracking/GaussianTrackState.h"
#include "airborne_radar/signal/tracking/TrackLifecycleTypes.h"

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
    association::AssociationResult* association_result);

DetectionExecutionBuffers BuildDetectionExecutionBuffers(
    std::vector<detection::ResolvedTargetGeometry>* target_geometry,
    std::vector<float>* signal_term_db, std::vector<float>* speed_penalty_db,
    std::vector<float>* detection_margin_db, std::vector<std::uint8_t>* detection_succeeded,
    std::vector<tracking::MeasurementCovariance>* measurement_covariances);

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
    std::vector<tracking::TrackMeasurement>* track_measurements);

TrackFilterApplyContext BuildTrackFilterApplyContextBindings(
    const common::TargetFeatureList* input, common::TargetFeatureList* output,
    const std::vector<std::uint8_t>* detection_succeeded,
    const std::vector<float>* detection_margin_db, bool jamming_detected,
    common::JammingSemantic dominant_jamming_semantic, float jamming_severity,
    tracking::TrackFilter* track_filter, const std::vector<int>* measurement_slots,
    std::vector<tracking::TrackMeasurement>* track_measurements);

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CONTEXT_BINDING_SUPPORT_H_
