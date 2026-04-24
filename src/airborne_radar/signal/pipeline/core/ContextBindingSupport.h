/**
 * @file ContextBindingSupport.h
 * @brief 定义 SignalPipeline 各阶段上下文绑定的内部辅助函数。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CONTEXT_BINDING_SUPPORT_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CONTEXT_BINDING_SUPPORT_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/session/RadarSceneTypes.h"
#include "1q/airborne_radar/model/JammingSemantics.h"
#include "airborne_radar/signal/association/DataAssociation.h"
#include "airborne_radar/signal/detection/TargetGeometryResolver.h"
#include "airborne_radar/signal/pipeline/core/CycleContextSupport.h"
#include "airborne_radar/signal/pipeline/core/DetectionExecution.h"
#include "airborne_radar/signal/pipeline/config/SignalPipelineRuntimeTypes.h"
#include "airborne_radar/signal/pipeline/assembly/TrackMeasurementProcessing.h"
#include "airborne_radar/signal/tracking/GaussianTrackState.h"
#include "airborne_radar/signal/tracking/TrackLifecycleTypes.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

/**
 * @brief 根据输入目标列表与运行时配置初始化周期工作区。
 * @param[in] input_state 当前周期输入目标列表。
 * @param[in] runtime_config 运行时配置。
 * @param[out] workspace 待初始化的周期工作区。
 */
CycleWorkspace BuildCycleWorkspaceBindings(
    session::RadarSceneTargetList* output_state,
    model::DecisionInputFrame* decision_frame,
    AssociationQualityMetrics* association_quality_metrics,
    std::vector<tracking::TrackMeasurement>* track_measurements, std::vector<float>* signal_term_db,
    std::vector<float>* speed_penalty_db, std::vector<float>* detection_margin_db,
    std::vector<std::uint8_t>* detection_succeeded, std::vector<std::uint64_t>* association_keys,
    std::vector<int>* measurement_slots,
    std::vector<detection::ResolvedTargetGeometry>* target_geometry,
    std::vector<tracking::MeasurementCovariance>* measurement_covariances,
    association::AssociationResult* association_result);

/**
 * @brief 构造探测阶段的输出缓存绑定视图。
 * @param[out] target_geometry 目标几何信息列表。
 * @param[out] signal_term_db 信号项列表。
 * @param[out] speed_penalty_db 速度惩罚项列表。
 * @param[out] detection_margin_db 探测裕量列表。
 * @param[out] detection_succeeded 探测成功标志列表。
 * @param[out] measurement_covariances 量测协方差列表。
 */
DetectionExecutionBuffers BuildDetectionExecutionBuffers(
    std::vector<detection::ResolvedTargetGeometry>* target_geometry,
    std::vector<float>* signal_term_db, std::vector<float>* speed_penalty_db,
    std::vector<float>* detection_margin_db, std::vector<std::uint8_t>* detection_succeeded,
    std::vector<tracking::MeasurementCovariance>* measurement_covariances);

/**
 * @brief 构造量测构建阶段的上下文绑定。
 * @param[in] input 输入目标列表。
 * @param[in] association_result 关联匹配结果。
 * @param[in] detection_succeeded 探测成功标志列表。
 * @param[in] association_keys 关联键列表。
 * @param[in] detection_margin_db 探测裕量列表。
 * @param[in] target_geometry 目标几何信息列表。
 * @param[in] measurement_covariances 量测协方差列表。
 * @param[in] jamming_detected 是否检测到干扰。
 * @param[in] dominant_jamming_semantic 主导干扰语义。
 * @param[in] jamming_severity 干扰强度。
 * @param[out] measurement_slots 各目标的量测槽位。
 * @param[out] track_measurements 跟踪量测输出。
 */
TrackMeasurementBuildContext BuildTrackMeasurementBuildContextBindings(
    const session::RadarSceneTargetList& input,
    const association::AssociationResult& association_result,
    const std::vector<std::uint8_t>& detection_succeeded,
    const std::vector<std::uint64_t>& association_keys,
    const std::vector<float>& detection_margin_db,
    const std::vector<detection::ResolvedTargetGeometry>& target_geometry,
    const std::vector<tracking::MeasurementCovariance>& measurement_covariances,
    bool jamming_detected, model::JammingSemantic dominant_jamming_semantic,
    float jamming_severity, std::vector<int>& measurement_slots,
    std::vector<tracking::TrackMeasurement>& track_measurements);

/**
 * @brief 构造滤波应用阶段的上下文绑定。
 * @param[in] input 输入目标列表。
 * @param[out] output 输出目标列表。
 * @param[in] detection_succeeded 探测成功标志列表。
 * @param[in] detection_margin_db 探测裕量列表。
 * @param[in] jamming_detected 是否检测到干扰。
 * @param[in] dominant_jamming_semantic 主导干扰语义。
 * @param[in] jamming_severity 干扰强度。
 * @param[in] track_filter 轨迹滤波器。
 * @param[in] measurement_slots 各目标的量测槽位。
 * @param[out] track_measurements 跟踪量测输出。
 */
TrackFilterApplyContext BuildTrackFilterApplyContextBindings(
    const session::RadarSceneTargetList& input, session::RadarSceneTargetList& output,
    const std::vector<std::uint8_t>& detection_succeeded,
    const std::vector<float>& detection_margin_db, bool jamming_detected,
    model::JammingSemantic dominant_jamming_semantic, float jamming_severity,
    tracking::TrackFilter& track_filter, const std::vector<int>& measurement_slots,
    std::vector<tracking::TrackMeasurement>& track_measurements);

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CONTEXT_BINDING_SUPPORT_H_
