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
#include "airborne_radar/signal/output/IDataOutputManager.h"
#include "airborne_radar/signal/association/DataAssociation.h"
#include "airborne_radar/signal/tracking/ITrackLifecycleManager.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

/**
 * @brief 收集单周期的关联质量指标与决策帧输出。
 * @param[in] control_profile 当前控制真值。
 * @param[in] cycle_index 周期号。
 * @param[in] batch_id 批次号。
 * @param[in] runtime_config 运行时配置。
 * @param[in] environment_snapshot 当前周期环境快照。
 * @param[in] input_state 输入目标列表。
 * @param[in] output_state 输出目标列表。
 * @param[in] association_result 关联匹配结果。
 * @param[in] track_measurements 跟踪量测列表。
 * @param[in] output_manager 数据输出管理器（可为 nullptr）。
 * @param[in] auto_lifecycle_manager 自动生命周期管理器（可为 nullptr）。
 * @param[out] association_quality_metrics 关联质量观测指标。
 * @param[out] decision_frame 决策输入帧。
 */
void CollectCycleOutputs(const common::RadarControlProfile& control_profile,
                         std::uint32_t cycle_index, std::uint64_t batch_id,
                         const SignalPipelineConfig& runtime_config,
                         const environment::EnvironmentSnapshot& environment_snapshot,
                         const common::TargetFeatureList& input_state,
                         const common::TargetFeatureList& output_state,
                         const association::AssociationResult& association_result,
                         const std::vector<tracking::TrackMeasurement>& track_measurements,
                         signal::output::IDataOutputManager* output_manager,
                         tracking::ITrackLifecycleManager* auto_lifecycle_manager,
                         AssociationQualityMetrics* association_quality_metrics,
                         common::DecisionInputFrame* decision_frame);

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_OUTPUT_ASSEMBLY_SUPPORT_H_
