/**
 * @file RuntimeAssemblySupport.h
 * @brief 定义 SignalPipeline 运行时配置与组件装配的内部辅助函数。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_RUNTIME_ASSEMBLY_SUPPORT_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_RUNTIME_ASSEMBLY_SUPPORT_H_

#include <memory>

#include "1q/airborne_radar/session/ArControlProfile.h"
#include "airborne_radar/signal/detection/SignalDetector.h"
#include "airborne_radar/signal/pipeline/SignalPipelineExecutionConfig.h"
#include "airborne_radar/signal/pipeline/SignalPipelineRuntimeTypes.h"
#include "airborne_radar/signal/tracking/IKalmanPredictor.h"
#include "airborne_radar/signal/tracking/IKalmanUpdater.h"
#include "airborne_radar/signal/tracking/ITrackLifecycleManager.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

using ExecutionConfig = ::airborne_radar::config::execution::InternalExecutionConfig;

struct ResolvedRuntimePipelineConfig {
  ExecutionConfig config{};
};

ResolvedRuntimePipelineConfig ResolveRuntimePipelineConfig(
    const ExecutionConfig& base_config,
    const session::RadarControlProfile& control_profile);

std::unique_ptr<tracking::ITrackLifecycleManager> CreateAutoLifecycleManagerForRuntimeConfig(
    const ExecutionConfig& runtime_config);

/**
 * @brief OwnedComponentSlots 汇聚 Pipeline 自持有组件的重建槽位指针。
 */
struct OwnedComponentSlots {
  std::unique_ptr<tracking::IKalmanPredictor>* kalman_predictor{nullptr};
  std::unique_ptr<tracking::IKalmanUpdater>* kalman_updater{nullptr};
  std::unique_ptr<detection::SignalDetector>* signal_detector{nullptr};
  std::unique_ptr<tracking::ITrackLifecycleManager>* auto_lifecycle_manager{nullptr};
};

void RebuildOwnedComponentsForPipeline(
    const ExecutionConfig& base_config,
    const session::RadarControlProfile& control_profile, OwnedComponentSlots* slots);

bool SyncAutoLifecycleManagerForResolvedRuntimeConfig(
    const ResolvedRuntimePipelineConfig& resolved_runtime_config,
    tracking::ITrackLifecycleManager* auto_lifecycle_manager);

bool SyncAutoLifecycleManagerForRuntimeConfig(
    const ExecutionConfig& runtime_config,
    tracking::ITrackLifecycleManager* auto_lifecycle_manager);

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_RUNTIME_ASSEMBLY_SUPPORT_H_
