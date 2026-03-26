/**
 * @file DetectionExecution.h
 * @brief 定义 SignalPipeline 探测执行阶段的内部辅助函数。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_DETECTION_EXECUTION_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_DETECTION_EXECUTION_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/environment/EnvironmentTypes.h"
#include "1q/airborne_radar/signal/pipeline/SignalPipelineTypes.h"
#include "airborne_radar/signal/detection/TargetGeometryResolver.h"
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
  std::vector<detection::ResolvedTargetGeometry>* target_geometry{nullptr};
  std::vector<float>* signal_term_db{nullptr};
  std::vector<float>* speed_penalty_db{nullptr};
  std::vector<float>* detection_margin_db{nullptr};
  std::vector<std::uint8_t>* detection_succeeded{nullptr};
  std::vector<tracking::MeasurementCovariance>* measurement_covariances{nullptr};
};

void RunHeuristicDetectionPass(const common::TargetFeatureList& input,
                               const SignalPipelineConfig& runtime_config,
                               const common::RadarControlProfile& control_profile,
                               const environment::EnvironmentSnapshot& environment_snapshot,
                               DetectionExecutionBuffers* buffers);

void RunPhysicalDetectionPass(const common::TargetFeatureList& input,
                              const SignalPipelineConfig& runtime_config,
                              const common::RadarControlProfile& control_profile,
                              const environment::EnvironmentSnapshot& environment_snapshot,
                              detection::SignalDetector* signal_detector,
                              DetectionExecutionBuffers* buffers);

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_DETECTION_EXECUTION_H_
