/**
 * @file CycleContextSupport.h
 * @brief 定义 SignalPipeline 周期缓存初始化与配置同步的内部辅助函数。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CYCLE_CONTEXT_SUPPORT_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CYCLE_CONTEXT_SUPPORT_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/common/model/DecisionInputFrame.h"
#include "1q/airborne_radar/common/model/TargetFeature.h"
#include "airborne_radar/signal/pipeline/SignalPipelineRuntimeTypes.h"
#include "airborne_radar/signal/association/DataAssociation.h"
#include "airborne_radar/signal/detection/TargetGeometryResolver.h"
#include "airborne_radar/signal/pipeline/InternalSignalPipelineConfig.h"
#include "airborne_radar/signal/tracking/GaussianTrackState.h"
#include "airborne_radar/signal/tracking/TrackFilter.h"
#include "airborne_radar/signal/tracking/TrackLifecycleTypes.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

struct CycleWorkspace {
  common::model::TargetFeatureList* output_state{nullptr};                     /**< 输出目标列表 */
  common::model::DecisionInputFrame* decision_frame{nullptr};                  /**< 决策输入帧 */
  AssociationQualityMetrics* association_quality_metrics{nullptr};      /**< 关联质量观测指标 */
  std::vector<tracking::TrackMeasurement>* track_measurements{nullptr}; /**< 跟踪量测列表 */
  std::vector<float>* signal_term_db{nullptr};                          /**< 各目标信号项（dB） */
  std::vector<float>* speed_penalty_db{nullptr};           /**< 各目标速度惩罚项（dB） */
  std::vector<float>* detection_margin_db{nullptr};        /**< 各目标探测裕量（dB） */
  std::vector<std::uint8_t>* detection_succeeded{nullptr}; /**< 各目标探测成功标志 */
  std::vector<std::uint64_t>* association_keys{nullptr};   /**< 各目标关联键 */
  std::vector<int>* measurement_slots{nullptr};            /**< 各目标量测槽位 */
  std::vector<detection::ResolvedTargetGeometry>* target_geometry{nullptr}; /**< 各目标几何信息 */
  std::vector<tracking::MeasurementCovariance>* measurement_covariances{
      nullptr};                                                /**< 各目标量测协方差 */
  association::AssociationResult* association_result{nullptr}; /**< 关联匹配结果 */
};

/**
 * @brief 根据输入目标列表与运行时配置重置周期工作区。
 * @param[in] input_state 当前周期输入目标列表。
 * @param[in] runtime_config 运行时配置。
 * @param[out] workspace 待重置的周期工作区。
 */
void ResetCycleWorkspace(const common::model::TargetFeatureList& input_state,
                         const SignalPipelineConfig& runtime_config, CycleWorkspace* workspace);

/**
 * @brief 重置量测协方差为默认值。
 * @param[in] target_count 目标数量。
 * @param[in] kalman_measurement_noise_std Kalman 量测噪声标准差。
 * @param[out] measurement_covariances 待重置的量测协方差列表。
 */
void RefreshMeasurementCovariances(
    std::size_t target_count, float kalman_measurement_noise_std,
    std::vector<tracking::MeasurementCovariance>* measurement_covariances);

/**
 * @brief 同步运行时配置到关联引擎与轨迹滤波器。
 * @param[in] runtime_config 当前运行时配置。
 * @param[in] internal_runtime_config 当前内部运行时配置。
 * @param[in,out] association_engine 关联引擎。
 * @param[in,out] track_filter 轨迹滤波器。
 */
void SyncAssociationAndTrackFilterConfigs(
    const SignalPipelineConfig& runtime_config,
    const InternalSignalPipelineConfig& internal_runtime_config,
                                          association::DataAssociationEngine* association_engine,
                                          tracking::TrackFilter* track_filter);

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_CYCLE_CONTEXT_SUPPORT_H_
