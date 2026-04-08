/**
 * @file TrackMeasurementProcessing.h
 * @brief 定义 SignalPipeline 量测构建与滤波写回的内部辅助函数。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_TRACK_MEASUREMENT_PROCESSING_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_TRACK_MEASUREMENT_PROCESSING_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/model/TargetFeature.h"
#include "1q/airborne_radar/model/JammingSemantics.h"
#include "airborne_radar/signal/association/DataAssociation.h"
#include "airborne_radar/signal/detection/TargetGeometryResolver.h"
#include "airborne_radar/signal/tracking/GaussianTrackState.h"
#include "airborne_radar/signal/tracking/TrackFilter.h"
#include "airborne_radar/signal/tracking/TrackLifecycleTypes.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

/**
 * @brief TrackMeasurementBuildContext 汇聚量测构建阶段的输入与输出绑定。
 */
struct TrackMeasurementBuildContext {
  const model::TargetFeatureList* input{nullptr};            /**< 输入目标列表 */
  const association::AssociationResult* association_result{nullptr}; /**< 关联匹配结果 */
  const std::vector<std::uint8_t>* detection_succeeded{nullptr};     /**< 各目标探测成功标志 */
  const std::vector<std::uint64_t>* association_keys{nullptr};       /**< 各目标关联键 */
  const std::vector<float>* detection_margin_db{nullptr};            /**< 各目标探测裕量（dB） */
  const std::vector<detection::ResolvedTargetGeometry>* target_geometry{
      nullptr}; /**< 各目标几何信息 */
  const std::vector<tracking::MeasurementCovariance>* measurement_covariances{
      nullptr};                 /**< 各目标量测协方差 */
  bool jamming_detected{false}; /**< 是否检测到干扰 */
  model::JammingSemantic dominant_jamming_semantic{
      model::JammingSemantic::kNone};                           /**< 主导干扰语义 */
  float jamming_severity{0.0f};                                         /**< 干扰强度 */
  std::vector<int>* measurement_slots{nullptr};                         /**< 各目标量测槽位 */
  std::vector<tracking::TrackMeasurement>* track_measurements{nullptr}; /**< 跟踪量测输出 */
};

/**
 * @brief 根据上下文构建所有成功检测目标的跟踪量测。
 * @param[in] context 量测构建上下文。
 */
void BuildTrackMeasurementsPass(const TrackMeasurementBuildContext& context);

/**
 * @brief TrackFilterApplyContext 汇聚滤波应用阶段的输入与输出绑定。
 */
struct TrackFilterApplyContext {
  const model::TargetFeatureList* input{nullptr};        /**< 输入目标列表 */
  model::TargetFeatureList* output{nullptr};             /**< 输出目标列表 */
  const std::vector<std::uint8_t>* detection_succeeded{nullptr}; /**< 各目标探测成功标志 */
  const std::vector<float>* detection_margin_db{nullptr};        /**< 各目标探测裕量（dB） */
  bool jamming_detected{false};                                  /**< 是否检测到干扰 */
  model::JammingSemantic dominant_jamming_semantic{
      model::JammingSemantic::kNone};                           /**< 主导干扰语义 */
  float jamming_severity{0.0f};                                         /**< 干扰强度 */
  tracking::TrackFilter* track_filter{nullptr};                         /**< 轨迹滤波器 */
  const std::vector<int>* measurement_slots{nullptr};                   /**< 各目标量测槽位 */
  std::vector<tracking::TrackMeasurement>* track_measurements{nullptr}; /**< 跟踪量测输出 */
};

/**
 * @brief 根据上下文对输出目标列表执行滤波写回。
 * @param[in] context 滤波应用上下文。
 */
void ApplyTrackFilterPass(const TrackFilterApplyContext& context);

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_TRACK_MEASUREMENT_PROCESSING_H_
