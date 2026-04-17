/**
 * @file SignalLifecycleConfig.h
 * @brief 定义生命周期域关键配置（config 主入口）。
 */

#ifndef AIRBORNE_RADAR_CONFIG_SIGNAL_LIFECYCLE_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_SIGNAL_LIFECYCLE_CONFIG_H_

#include <cstdint>

namespace airborne_radar {
namespace config {

/**
 * @brief LifecyclePolicyProfile 表示对外生命周期策略档位。
 */
enum class LifecyclePolicyProfile {
  kBalanced = 0,        /**< 平衡策略 */
  kFastConfirm = 1,     /**< 快速确认策略 */
  kHighPersistence = 2  /**< 高留存策略 */
};

/**
 * @brief SignalLifecycleConfig 描述对外语义化生命周期输入。
 */
struct SignalLifecycleConfig {
  LifecyclePolicyProfile policy_profile{
      LifecyclePolicyProfile::kBalanced}; /**< 生命周期策略档位 */
  bool enable_imm_fusion{false};          /**< 是否启用 IMM 融合策略 */
};

}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_SIGNAL_LIFECYCLE_CONFIG_H_
