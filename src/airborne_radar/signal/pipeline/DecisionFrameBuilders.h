/**
 * @file DecisionFrameBuilders.h
 * @brief 定义 SignalPipeline 决策帧与快照构建的内部辅助函数。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_DECISION_FRAME_BUILDERS_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_DECISION_FRAME_BUILDERS_H_

#include "1q/airborne_radar/environment/EnvironmentTypes.h"
#include "1q/airborne_radar/signal/pipeline/SignalPipelineTypes.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

common::EccmSourceInfo BuildEccmSourceInfo(
    const environment::EnvironmentSnapshot& environment_snapshot);

common::AssociationQualityInfo BuildAssociationQualityInfo(
    const AssociationQualityMetrics& metrics);

common::PerceptionQualityInfo BuildPerceptionQualityInfo(std::size_t input_target_count,
                                                         const AssociationQualityMetrics& metrics);

common::DecisionTrackSnapshotList BuildDecisionSnapshotsFromFeatures(
    const common::TargetFeatureList& features);

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_DECISION_FRAME_BUILDERS_H_
