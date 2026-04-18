/**
 * @file DetectionExecution.h
 * @brief 定义 SignalPipeline 探测执行阶段的内部辅助函数。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_DETECTION_EXECUTION_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_DETECTION_EXECUTION_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/extension/control/RadarControlProfile.h"
#include "1q/airborne_radar/environment/EnvironmentTypes.h"
#include "airborne_radar/signal/detection/TargetGeometryResolver.h"
#include "airborne_radar/signal/pipeline/config/InternalPipelineConfig.h"
#include "airborne_radar/signal/pipeline/config/SignalPipelineRuntimeTypes.h"
#include "airborne_radar/signal/tracking/GaussianTrackState.h"

namespace airborne_radar {
namespace signal {
namespace detection {
class SignalDetector;
}  // namespace detection
namespace pipeline {
namespace internal {

/**
 * @brief 探测阶段输出缓存视图。
 */
struct DetectionExecutionBuffers {
  std::vector<detection::ResolvedTargetGeometry>* target_geometry{nullptr}; /**< 各目标几何信息 */
  std::vector<float>* signal_term_db{nullptr};             /**< 各目标信号项（dB） */
  std::vector<float>* speed_penalty_db{nullptr};           /**< 各目标速度惩罚项（dB） */
  std::vector<float>* detection_margin_db{nullptr};        /**< 各目标探测裕量（dB） */
  std::vector<std::uint8_t>* detection_succeeded{nullptr}; /**< 各目标探测成功标志 */
  std::vector<tracking::MeasurementCovariance>* measurement_covariances{
      nullptr}; /**< 各目标量测协方差 */
};

void RunHeuristicDetectionPass(const model::TargetFeatureList& input,
                               const PipelineConfig& runtime_config,
                               const InternalPipelineConfig& internal_config,
                               const extension::control::RadarControlProfile& control_profile,
                               const environment::EnvironmentSnapshot& environment_snapshot,
                               DetectionExecutionBuffers* buffers);

void RunPhysicalDetectionPass(const model::TargetFeatureList& input,
                              const PipelineConfig& runtime_config,
                              const InternalPipelineConfig& internal_config,
                              const extension::control::RadarControlProfile& control_profile,
                              const environment::EnvironmentSnapshot& environment_snapshot,
                              detection::SignalDetector* signal_detector,
                              DetectionExecutionBuffers* buffers);

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_DETECTION_EXECUTION_H_
