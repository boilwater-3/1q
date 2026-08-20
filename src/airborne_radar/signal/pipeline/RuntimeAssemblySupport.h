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

/**
 * @brief 解析后的运行时配置（已叠加控制真值）。
 */
struct ResolvedRuntimePipelineConfig {
  ExecutionConfig config{};
};

/**
 * @brief 将控制真值叠加到基础执行配置，得到运行时配置。
 * @param base_config 基础执行配置。
 * @param control_profile 当前控制真值。
 * @return 叠加控制效果后的运行时配置。
 */
ResolvedRuntimePipelineConfig ResolveRuntimePipelineConfig(
    const ExecutionConfig& base_config,
    const session::ArControlProfile& control_profile);

/**
 * @brief 按运行时配置创建自动生命周期管理器。
 * @param runtime_config 运行时配置。
 * @return 创建成功返回非空 unique_ptr；装配失败返回空 unique_ptr。
 */
std::unique_ptr<tracking::ITrackLifecycleManager> CreateAutoLifecycleManagerForRuntimeConfig(
    const ExecutionConfig& runtime_config);

/**
 * @brief OwnedComponentSlots 汇聚 Pipeline 自持有组件的重建槽位指针。
 * @note 各指针指向外部持有的 unique_ptr，本结构不拥有这些对象。
 */
struct OwnedComponentSlots {
  std::unique_ptr<tracking::IKalmanPredictor>* kalman_predictor{nullptr};       /**< Kalman 预测器槽位。 */
  std::unique_ptr<tracking::IKalmanUpdater>* kalman_updater{nullptr};           /**< Kalman 更新器槽位。 */
  std::unique_ptr<detection::SignalDetector>* signal_detector{nullptr};         /**< 信号检测器槽位。 */
  std::unique_ptr<tracking::ITrackLifecycleManager>* auto_lifecycle_manager{nullptr}; /**< 自动生命周期管理器槽位。 */
};

/**
 * @brief 重建 Pipeline 自持有的全部组件（预测器、更新器、检测器、生命周期管理器）。
 * @param base_config 基础执行配置。
 * @param control_profile 当前控制真值（用于解析运行时生命周期配置）。
 * @param[in,out] slots 组件槽位指针集合；slots 为 nullptr 或字段非法时直接返回。
 */
void RebuildOwnedComponentsForPipeline(
    const ExecutionConfig& base_config,
    const session::ArControlProfile& control_profile, OwnedComponentSlots* slots);

/**
 * @brief 将解析后的运行时配置同步到自动生命周期管理器。
 * @param resolved_runtime_config 已解析的运行时配置。
 * @param[in] auto_lifecycle_manager 待同步的管理器。
 * @return 管理器类型不匹配或同步失败返回 false。
 */
bool SyncAutoLifecycleManagerForResolvedRuntimeConfig(
    const ResolvedRuntimePipelineConfig& resolved_runtime_config,
    tracking::ITrackLifecycleManager* auto_lifecycle_manager);

/**
 * @brief 将运行时配置同步到自动生命周期管理器。
 * @param runtime_config 运行时配置。
 * @param[in] auto_lifecycle_manager 待同步的管理器。
 * @return 管理器类型不匹配或同步失败返回 false。
 */
bool SyncAutoLifecycleManagerForRuntimeConfig(
    const ExecutionConfig& runtime_config,
    tracking::ITrackLifecycleManager* auto_lifecycle_manager);

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_RUNTIME_RUNTIME_ASSEMBLY_SUPPORT_H_
