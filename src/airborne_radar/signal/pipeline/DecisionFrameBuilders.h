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

/**
 * @brief 将环境快照转换为决策层 ECCM 输入摘要。
 * @param[in] environment_snapshot 当前周期环境快照。
 * @return 供决策层消费的 ECCM 输入摘要。
 */
common::model::EccmSourceInfo BuildEccmSourceInfo(
    const environment::EnvironmentSnapshot& environment_snapshot);

/**
 * @brief 将 Pipeline 关联质量指标转换为决策层质量摘要。
 * @param[in] metrics Pipeline 对外关联质量指标。
 * @return 决策层消费的关联质量摘要。
 */
common::model::AssociationQualityInfo BuildAssociationQualityInfo(
    const AssociationQualityMetrics& metrics);

/**
 * @brief 构造当前周期探测质量摘要。
 * @param[in] input_target_count 当前周期输入目标数。
 * @param[in] metrics 当前周期关联质量指标。
 * @return 决策层消费的探测质量摘要。
 */
common::model::PerceptionQualityInfo BuildPerceptionQualityInfo(std::size_t input_target_count,
                                                         const AssociationQualityMetrics& metrics);

/**
 * @brief 将目标特征列表转换为决策轨迹快照列表。
 * @param[in] features 输入目标特征列表。
 * @return 对应的决策轨迹快照列表。
 */
common::model::DecisionTrackSnapshotList BuildDecisionSnapshotsFromFeatures(
    const common::model::TargetFeatureList& features);

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_DECISION_FRAME_BUILDERS_H_
