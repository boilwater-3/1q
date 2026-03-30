/**
 * @file RuntimeAssemblySupport.h
 * @brief 定义 SignalPipeline 运行时配置与组件装配的内部辅助函数。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_RUNTIME_ASSEMBLY_SUPPORT_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_RUNTIME_ASSEMBLY_SUPPORT_H_

#include <memory>

#include "1q/airborne_radar/common/control/RadarControlProfile.h"
#include "airborne_radar/signal/pipeline/SignalPipelineRuntimeTypes.h"
#include "airborne_radar/signal/detection/SignalDetector.h"
#include "airborne_radar/signal/pipeline/InternalSignalPipelineConfig.h"
#include "airborne_radar/signal/tracking/ITrackLifecycleManager.h"
#include "airborne_radar/signal/tracking/KalmanPredictor.h"
#include "airborne_radar/signal/tracking/KalmanUpdater.h"

namespace airborne_radar {
namespace signal {
namespace runtime {

using pipeline::SignalPipelineConfig;

namespace internal {

struct ResolvedRuntimeSignalPipelineConfig {
  SignalPipelineConfig public_config{};
  pipeline::internal::InternalSignalPipelineConfig internal_config{};
};

/**
 * @brief 基于基础配置与控制真值构建运行时配置。
 * @param[in] base_config 基础流水线配置。
 * @param[in] base_internal_config 基础内部扩展配置。
 * @param[in] control_profile 当前控制真值。
 * @return 调整后的运行时配置。
 */
ResolvedRuntimeSignalPipelineConfig BuildRuntimeConfigFromControlProfile(
    const SignalPipelineConfig& base_config,
    const pipeline::internal::InternalSignalPipelineConfig& base_internal_config,
    const common::control::RadarControlProfile& control_profile);

/**
 * @brief 根据运行时配置自动装配生命周期管理器。
 * @param[in] runtime_config 运行时配置。
 * @return 生命周期管理器实例；未启用时返回空指针。
 */
std::unique_ptr<tracking::ITrackLifecycleManager> CreateAutoLifecycleManagerForRuntimeConfig(
    const SignalPipelineConfig& runtime_config);

std::unique_ptr<tracking::ITrackLifecycleManager> CreateAutoLifecycleManagerForRuntimeConfig(
    const SignalPipelineConfig& runtime_config,
    const pipeline::internal::InternalSignalPipelineConfig& internal_config);

/**
 * @brief OwnedComponentSlots 汇聚 Pipeline 自持有组件的重建槽位指针。
 */
struct OwnedComponentSlots {
  std::unique_ptr<tracking::KalmanPredictor>* kalman_predictor{nullptr}; /**< Kalman 预测器槽位 */
  std::unique_ptr<tracking::KalmanUpdater>* kalman_updater{nullptr};     /**< Kalman 更新器槽位 */
  std::unique_ptr<detection::SignalDetector>* signal_detector{nullptr};  /**< 物理探测器槽位 */
  std::unique_ptr<tracking::ITrackLifecycleManager>* auto_lifecycle_manager{
      nullptr}; /**< 生命周期管理器槽位 */
};

/**
 * @brief 根据基础配置与控制真值重建 Pipeline 自持有组件。
 * @param[in] base_config 基础流水线配置。
 * @param[in] base_internal_config 基础内部扩展配置。
 * @param[in] control_profile 当前控制真值。
 * @param[in,out] slots 待重建的组件槽位。
 */
void RebuildOwnedComponentsForPipeline(const SignalPipelineConfig& base_config,
                                       const pipeline::internal::InternalSignalPipelineConfig&
                                           base_internal_config,
                                       const common::control::RadarControlProfile& control_profile,
                                       OwnedComponentSlots* slots);

}  // namespace internal
}  // namespace runtime
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_RUNTIME_ASSEMBLY_SUPPORT_H_
