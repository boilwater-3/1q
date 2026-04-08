/**
 * @file EnvironmentTypes.h
 * @brief 定义环境层对外公开的运行期场景与快照类型。
 */

#ifndef AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_TYPES_H_
#define AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_TYPES_H_

#include <cstdint>

#include "1q/airborne_radar/environment/EnvironmentConfig.h"

namespace airborne_radar {
namespace environment {

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
  float cycle_dt_sec{0.0f};              /**< 当前周期步长（单位：s） */
  float propagation_loss_db{0.0f};       /**< 传播损耗（单位：dB） */
  float clutter_power_db{0.0f};          /**< 杂波功率估计（单位：dB） */
  JammerSourceFactList jammer_sources{}; /**< 当前周期可见的多源干扰事实 */
  bool jamming_detected{false};          /**< 是否检测到干扰 */
};

/**
 * @brief EnvironmentSceneState 描述环境层待冻结的场景状态。
 */
struct EnvironmentSceneState {
  float base_propagation_loss_db{4.0f};         /**< 基础传播损耗（dB） */
  float atmospheric_attenuation_db{1.5f};       /**< 大气附加衰减（dB） */
  float terrain_reflection_db{1.0f};            /**< 地形/多径附加项（dB） */
  float clutter_power_db{3.0f};                 /**< 杂波功率（dB） */
  AtmosphericPhysicsConfig atmospheric_physics{}; /**< 可选物理传播参数 */
  VegetationScatterPhysicsConfig vegetation_scatter_physics{}; /**< 可选植被散射参数 */
  JammerEmitterStateList jammer_emitters{};     /**< 当前场景中的干扰源输入 */
};

}  // namespace environment
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_TYPES_H_
