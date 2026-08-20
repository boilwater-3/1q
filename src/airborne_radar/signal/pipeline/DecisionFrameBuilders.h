/**
 * @file DecisionFrameBuilders.h
 * @brief 定义 SignalPipeline 决策帧与快照构建的内部辅助函数。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_DECISION_FRAME_BUILDERS_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_DECISION_FRAME_BUILDERS_H_

#include "airborne_radar/signal/pipeline/SignalPipelineRuntimeTypes.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

/**
 * @brief 将 Pipeline 关联质量指标转换为决策层质量摘要。
 * @param[in] metrics Pipeline 对外关联质量指标。
 * @return 决策层消费的关联质量摘要。
 */
session::AssociationQualityInfo BuildAssociationQualityInfo(
    const AssociationQualityMetrics& metrics);

/**
 * @brief 构造当前周期探测质量摘要。
 * @param[in] input_target_count 当前周期输入目标数。
 * @param[in] metrics 当前周期关联质量指标。
 * @return 决策层消费的探测质量摘要。
 */
session::PerceptionQualityInfo BuildPerceptionQualityInfo(
    std::size_t input_target_count, const AssociationQualityMetrics& metrics);


}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_DECISION_FRAME_BUILDERS_H_
