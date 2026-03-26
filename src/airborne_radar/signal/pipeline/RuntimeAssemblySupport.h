/**
 * @file RuntimeAssemblySupport.h
 * @brief 定义 SignalPipeline 运行时配置与组件装配的内部辅助函数。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_RUNTIME_ASSEMBLY_SUPPORT_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_RUNTIME_ASSEMBLY_SUPPORT_H_

#include <memory>

#include "1q/airborne_radar/signal/pipeline/SignalPipelineTypes.h"
#include "airborne_radar/signal/detection/SignalDetector.h"
#include "airborne_radar/signal/tracking/ITrackLifecycleManager.h"
#include "airborne_radar/signal/tracking/KalmanPredictor.h"
#include "airborne_radar/signal/tracking/KalmanUpdater.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

SignalPipelineConfig BuildRuntimeConfigFromControlProfile(
    const SignalPipelineConfig& base_config, const common::RadarControlProfile& control_profile);

std::unique_ptr<tracking::ITrackLifecycleManager> CreateAutoLifecycleManagerForRuntimeConfig(
    const SignalPipelineConfig& runtime_config);

struct OwnedComponentSlots {
  std::unique_ptr<tracking::KalmanPredictor>* kalman_predictor{nullptr};
  std::unique_ptr<tracking::KalmanUpdater>* kalman_updater{nullptr};
  std::unique_ptr<detection::SignalDetector>* signal_detector{nullptr};
  std::unique_ptr<tracking::ITrackLifecycleManager>* auto_lifecycle_manager{nullptr};
};

void RebuildOwnedComponentsForPipeline(const SignalPipelineConfig& base_config,
                                       const common::RadarControlProfile& control_profile,
                                       OwnedComponentSlots* slots);

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_RUNTIME_ASSEMBLY_SUPPORT_H_
