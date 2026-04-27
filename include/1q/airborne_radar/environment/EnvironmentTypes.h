/**
 * @file EnvironmentTypes.h
 * @brief 定义环境层对外公开的运行期场景与快照类型。
 */

#ifndef AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_TYPES_H_
#define AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_TYPES_H_

#include <cstdint>

#include "1q/api.hpp"
#include "1q/airborne_radar/environment/EnvironmentConfig.h"

namespace airborne_radar {
namespace environment {

/**
 * @brief JammerDirectionDeg 表示干扰来向角。
 */
struct ONEQ_API JammerDirectionDeg {
  float azimuth_deg{0.0f};   /**< 干扰来向方位角（单位：deg） */
  float elevation_deg{0.0f}; /**< 干扰来向俯仰角（单位：deg） */
};

/**
 * @brief JammerSourceFact 表示单周期冻结后的干扰事实输出。
 * @note 其中 overlap/prf/sidelobe 等量由库内基于场景事实与运行状态派生。
 */
struct ONEQ_API JammerSourceFact {
  JammingTechnique technique{JammingTechnique::kUnknown}; /**< 干扰技术类型 */
  float power_db{0.0f};                                   /**< 干扰功率估计（单位：dB） */
  float js_db{0.0f};                                      /**< 干扰与信号比估计（单位：dB） */
  float frequency_overlap_ratio{0.0f}; /**< 干扰与当前工作频率的重叠度，范围 [0, 1] */
  float prf_lock_risk{0.0f};           /**< 干扰对当前 PRF 锁定的风险度，范围 [0, 1] */
  bool has_direction_deg{false};       /**< 是否具有可用的干扰来向方位/俯仰角 */
  JammerDirectionDeg direction_deg{};  /**< 干扰来向角（仅在 has_direction_deg=true 时有效） */
  float angular_span_deg{0.0f};        /**< 干扰角域宽度（单位：deg） */
  bool in_sidelobe{false};             /**< 干扰是否主要经由旁瓣进入 */
  float confidence{1.0f};              /**< 干扰事实置信度，范围 [0, 1] */
};

/** @brief 单周期内可见的干扰源列表 */
using JammerSourceFactList = std::vector<JammerSourceFact>;

/**
 * @brief EnvironmentCycleContext 描述环境层周期冻结上下文。
 */
struct ONEQ_API EnvironmentCycleContext {
  std::uint32_t cycle_index{0U}; /**< 当前周期号 */
  float dt_sec{0.0f};            /**< 当前周期步长（单位：s） */
};

/**
 * @brief EnvironmentSnapshot 用于封装单个处理周期内的环境快照。
 */
struct ONEQ_API EnvironmentSnapshot {
  float cycle_dt_sec{0.0f};               /**< 当前周期步长（单位：s） */
  float propagation_loss_db{0.0f};        /**< 传播损耗（单位：dB） */
  float atmospheric_physics_loss_db{0.0f}; /**< 传播损耗中的大气物理附加项（单位：dB） */
  float clutter_power_db{0.0f};           /**< 杂波功率估计（单位：dB） */
  AtmosphericPhysicsConfig atmospheric_physics{}; /**< 当前周期启用的大气物理参数 */
  AtmosphericDerivedContext atmospheric_context{}; /**< 当前周期时间/空间天气上下文输入 */
  float effective_k_factor{4.0f / 3.0f};          /**< 当前周期自动推导的有效地球半径因子 */
  std::int32_t effective_day_of_year{172};        /**< 当前周期自动推导的年积日 */
  JammerSourceFactList jammer_sources{};  /**< 当前周期可见的多源干扰事实 */
  bool jamming_detected{false};           /**< 是否检测到干扰 */
};

/**
 * @brief EnvironmentSceneState 描述环境层待冻结的场景状态。
 */
struct ONEQ_API EnvironmentSceneState {
  AtmosphericPhysicsConfig atmospheric_physics{}; /**< 可选物理传播参数 */
  AtmosphericDerivedContext atmospheric_context{}; /**< 可选时间/空间天气上下文 */
  VegetationScatterPhysicsConfig vegetation_scatter_physics{}; /**< 可选植被散射参数 */
  JammerEmitterStateList jammer_emitters{};     /**< 当前场景中的干扰源输入 */
};

}  // namespace environment
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_TYPES_H_
