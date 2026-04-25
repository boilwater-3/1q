/**
 * @file DetectionExecution.h
 * @brief 定义 SignalPipeline 探测执行阶段的内部辅助函数。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_DETECTION_EXECUTION_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_DETECTION_EXECUTION_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/environment/EnvironmentTypes.h"
#include "1q/airborne_radar/extension/control/RadarControlProfile.h"
#include "airborne_radar/signal/detection/TargetGeometryResolver.h"
#include "airborne_radar/signal/pipeline/SignalPipelineExecutionConfig.h"
#include "airborne_radar/signal/pipeline/SignalPipelineRuntimeTypes.h"
#include "airborne_radar/signal/tracking/GaussianTrackState.h"

namespace airborne_radar {
namespace signal {
namespace detection {
class SignalDetector;
}  // namespace detection
namespace pipeline {

struct DetectionExecutionBuffers {
  std::vector<detection::ResolvedTargetGeometry>* target_geometry{nullptr};
  std::vector<float>* signal_term_db{nullptr};
  std::vector<float>* speed_penalty_db{nullptr};
  std::vector<float>* detection_margin_db{nullptr};
  std::vector<std::uint8_t>* detection_succeeded{nullptr};
  std::vector<tracking::MeasurementCovariance>* measurement_covariances{nullptr};
};

void RunHeuristicDetectionPass(const session::RadarSceneTargetList& input, const ExecutionConfig& config,
                               const extension::control::RadarControlProfile& control_profile,
                               const environment::EnvironmentSnapshot& environment_snapshot,
                               DetectionExecutionBuffers* buffers);

void RunPhysicalDetectionPass(const session::RadarSceneTargetList& input, const ExecutionConfig& config,
                              const extension::control::RadarControlProfile& control_profile,
                              const environment::EnvironmentSnapshot& environment_snapshot,
                              detection::SignalDetector* signal_detector,
                              DetectionExecutionBuffers* buffers);


}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_DETECTION_EXECUTION_H_
