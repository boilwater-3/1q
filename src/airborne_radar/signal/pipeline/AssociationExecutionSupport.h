/**
 * @file AssociationExecutionSupport.h
 * @brief 定义 SignalPipeline 关联阶段准备与执行的内部辅助函数。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_ASSOCIATION_EXECUTION_SUPPORT_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_ASSOCIATION_EXECUTION_SUPPORT_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/model/TargetFeature.h"
#include "airborne_radar/signal/association/DataAssociation.h"
#include "airborne_radar/signal/tracking/ITrackLifecycleManager.h"
#include "airborne_radar/signal/tracking/TrackLifecycleTypes.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

/**
 * @brief 为当前周期准备关联阶段的先验种子。
 * @param[in] has_manual_association_seeds 是否存在外部手动注入种子。
 * @param[in] manual_association_seeds 手动注入的关联种子。
 * @param[in] auto_lifecycle_manager 自动生命周期管理器（可为 nullptr）。
 * @param[in,out] association_engine 关联引擎。
 */
void PrepareAssociationSeedsForCycle(
    bool has_manual_association_seeds,
    const std::vector<tracking::AssociationTrackSeed>& manual_association_seeds,
    const tracking::ITrackLifecycleManager* auto_lifecycle_manager,
    association::DataAssociationEngine* association_engine);

/**
 * @brief 执行一次完整的关联匹配流程。
 * @param[in] input_state 当前周期输入目标列表。
 * @param[in] detection_succeeded 各目标的探测成功标志。
 * @param[in] measurement_covariances 各目标的量测协方差。
 * @param[in] dt_sec 周期步长（单位：s）。
 * @param[in,out] association_engine 关联引擎。
 * @param[out] association_result 关联匹配结果。
 * @param[out] association_keys 各目标的关联键。
 */
void RunAssociationPass(const model::TargetFeatureList& input_state,
                        const std::vector<std::uint8_t>& detection_succeeded,
                        const std::vector<tracking::MeasurementCovariance>& measurement_covariances,
                        float dt_sec, association::DataAssociationEngine* association_engine,
                        association::AssociationResult* association_result,
                        std::vector<std::uint64_t>* association_keys);

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_ASSOCIATION_EXECUTION_SUPPORT_H_
