/**
 * @file OutputAssemblySupport.h
 * @brief 定义 SignalPipeline 周期输出收尾装配的内部辅助函数。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_OUTPUT_ASSEMBLY_SUPPORT_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_OUTPUT_ASSEMBLY_SUPPORT_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/environment/EnvironmentTypes.h"
#include "1q/airborne_radar/signal/pipeline/SignalPipelineTypes.h"
#include "airborne_radar/core/output/IDataOutputManager.h"
#include "airborne_radar/signal/association/DataAssociation.h"
#include "airborne_radar/signal/tracking/ITrackLifecycleManager.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

void CollectCycleOutputs(
    const common::RadarControlProfile& control_profile, std::uint32_t cycle_index,
    std::uint64_t batch_id, const SignalPipelineConfig& runtime_config,
    const environment::EnvironmentSnapshot& environment_snapshot,
    const common::TargetFeatureList& input_state, const common::TargetFeatureList& output_state,
    const association::AssociationResult& association_result,
    const std::vector<tracking::TrackMeasurement>& track_measurements,
    core::output::IDataOutputManager* output_manager,
    tracking::ITrackLifecycleManager* auto_lifecycle_manager,
    AssociationQualityMetrics* association_quality_metrics,
    common::DecisionInputFrame* decision_frame);

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_OUTPUT_ASSEMBLY_SUPPORT_H_
