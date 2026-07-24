/**
 * @file ArEnvironmentInput.h
 * @brief 机载雷达单周期环境输入类型集合。
 *
 * 周期环境事实输入与状态维护的主头文件。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_ENVIRONMENT_INPUT_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_ENVIRONMENT_INPUT_H_

#include <cstdint>
#include "1q/airborne_radar/config/ArEnvironmentConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace session {

/**
 * @brief ArEnvironmentInputPatch 表示调用方侧环境事实状态的局部更新。
 *
 * @note 本类型不直接进入 ArSession::StepWithResult()。调用方应先用
 *       ArEnvironmentInputState 合成完整 ArEnvironmentInput 快照，再写入
 *       ArCycleInput::environment。
 */
struct ONEQ_API ArEnvironmentInputPatch {
  bool has_atmospheric_observation{false};                      /**< 是否更新气象/电离层输入 */
  config::AtmosphericPhysicsConfig atmospheric_observation{};   /**< 新气象/电离层输入 */
  bool has_surface_observation{false};                          /**< 是否更新地表/植被输入 */
  config::VegetationScatterPhysicsConfig surface_observation{}; /**< 新地表/植被输入 */
};

/**
 * @brief ArEnvironmentInput 聚合 AR 单周期环境事实输入。
 */
struct ONEQ_API ArEnvironmentInput {
  config::AtmosphericPhysicsConfig atmospheric_observation{};   /**< 当前周期气象/电离层输入 */
  config::VegetationScatterPhysicsConfig surface_observation{}; /**< 当前周期地表/植被输入 */
};

/**
 * @brief ArEnvironmentInputState 维护调用方侧当前环境事实状态。
 */
class ONEQ_API ArEnvironmentInputState {
 public:
  ArEnvironmentInputState() = default;
  explicit ArEnvironmentInputState(const ArEnvironmentInput& snapshot) : snapshot_(snapshot) {}

  /**
   * @brief 用完整快照整体覆盖内部状态。
   * @param[in] snapshot 新的环境事实快照。
   * @return 返回自身引用以支持链式调用。
   */
  ArEnvironmentInputState& Reset(const ArEnvironmentInput& snapshot) {
    snapshot_ = snapshot;
    return *this;
  }

  /**
   * @brief 按 patch 中 has_* 置位的字段局部更新内部状态。
   * @param[in] patch 局部更新补丁；未置位字段保持不变。
   * @return 返回自身引用以支持链式调用。
   */
  ArEnvironmentInputState& Update(const ArEnvironmentInputPatch& patch) {
    if (patch.has_atmospheric_observation) {
      snapshot_.atmospheric_observation = patch.atmospheric_observation;
    }
    if (patch.has_surface_observation) {
      snapshot_.surface_observation = patch.surface_observation;
    }
    return *this;
  }

  /**
   * @brief 返回内部状态的拷贝快照。
   * @return 当前环境事实快照。
   */
  ArEnvironmentInput Snapshot() const { return snapshot_; }

 private:
  ArEnvironmentInput snapshot_{};
};

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_ENVIRONMENT_INPUT_H_
