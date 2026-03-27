/**
 * @file AssociationExecutionSupport.h
 * @brief 定义 SignalPipeline 关联阶段准备与执行的内部辅助函数。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_ASSOCIATION_EXECUTION_SUPPORT_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_ASSOCIATION_EXECUTION_SUPPORT_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/common/TargetFeature.h"
#include "airborne_radar/signal/association/DataAssociation.h"
#include "airborne_radar/signal/tracking/ITrackLifecycleManager.h"
#include "airborne_radar/signal/tracking/TrackLifecycleTypes.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

void PrepareAssociationSeedsForCycle(
    bool has_manual_association_seeds,
    const std::vector<tracking::AssociationTrackSeed>& manual_association_seeds,
    const tracking::ITrackLifecycleManager* auto_lifecycle_manager,
    association::DataAssociationEngine* association_engine);

void RunAssociationPass(const common::TargetFeatureList& input_state,
                        const std::vector<std::uint8_t>& detection_succeeded,
                        const std::vector<tracking::MeasurementCovariance>& measurement_covariances,
                        float dt_sec,
                        association::DataAssociationEngine* association_engine,
                        association::AssociationResult* association_result,
                        std::vector<std::uint64_t>* association_keys);

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_ASSOCIATION_EXECUTION_SUPPORT_H_
