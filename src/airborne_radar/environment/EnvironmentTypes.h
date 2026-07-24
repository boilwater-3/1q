/**
 * @file EnvironmentTypes.h
 * @brief 环境层内部周期上下文、快照与场景状态类型（内部实现细节，不对外暴露）。
 *
 * @note EnvironmentCycleContext / EnvironmentSnapshot / EnvironmentSceneState
 *       是环境层与信号处理管线的内部流转类型，不属于公开 API，不标记 ONEQ_API。
 *       类型保留在 airborne_radar::session 命名空间，与历史消费方一致。
 */

#ifndef AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_TYPES_H_
#define AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_TYPES_H_

#include <cstdint>

#include "1q/airborne_radar/config/ArEnvironmentConfig.h"

namespace airborne_radar {
namespace session {

/**
 * @brief EnvironmentCycleContext 描述环境层周期冻结上下文。
 */
struct EnvironmentCycleContext {
  std::uint32_t cycle_index{0U}; /**< 当前周期号 */
  float dt_sec{0.0f};            /**< 当前周期步长（单位：s） */
};

/**
 * @brief EnvironmentSnapshot 用于封装单个处理周期内的环境快照。
 */
struct EnvironmentSnapshot {
  std::uint32_t cycle_index{0U};           /**< 当前周期号 */
  float cycle_dt_sec{0.0f};                /**< 当前周期步长（单位：s） */
  float propagation_loss_db{0.0f};         /**< 传播损耗（单位：dB） */
  float clutter_power_db{0.0f};            /**< 杂波功率估计（单位：dB） */
  config::AtmosphericPhysicsConfig atmospheric_physics{}; /**< 当前周期启用的大气物理参数 */
  float effective_k_factor{4.0f / 3.0f}; /**< 当前周期自动推导的有效地球半径因子 */
};

/**
 * @brief EnvironmentSceneState 描述环境层待冻结的场景状态。
 */
struct EnvironmentSceneState {
  config::AtmosphericPhysicsConfig atmospheric_physics{}; /**< 可选物理传播参数 */
  config::VegetationScatterPhysicsConfig vegetation_scatter_physics{}; /**< 可选植被散射参数 */
};

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_TYPES_H_
