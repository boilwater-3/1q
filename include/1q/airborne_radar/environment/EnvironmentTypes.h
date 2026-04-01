/**
 * @file EnvironmentTypes.h
 * @brief 定义环境层对外公开的轻量类型。
 */

#ifndef AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_TYPES_H_
#define AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_TYPES_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/common/model/DecisionSourceInfo.h"

namespace airborne_radar {
namespace environment {

/**
 * @brief JammingTechnique 与 `common::model::JammingTechnique` 保持统一的别名。
 * @details 权威定义位于 `common/DecisionSourceInfo.h`；
 *          此别名保证环境层代码可继续通过
 *          `environment::JammingTechnique` 引用，无需修改已有代码。
 */
using JammingTechnique = common::model::JammingTechnique;

/**
 * @brief JammerSourceFact 表示单个干扰源的事实描述。
 */
struct JammerSourceFact {
  JammingTechnique technique{JammingTechnique::kUnknown}; /**< 干扰技术类型 */
  float power_db{0.0f};                                   /**< 干扰功率估计（单位：dB） */
  float js_db{0.0f};                                      /**< 干扰与信号比估计（单位：dB） */
  float frequency_overlap_ratio{0.0f}; /**< 干扰与当前工作频率的重叠度，范围 [0, 1] */
  float prf_lock_risk{0.0f};           /**< 干扰对当前 PRF 锁定的风险度，范围 [0, 1] */
  float azimuth_deg{0.0f};             /**< 干扰来向方位角（单位：deg） */
  float elevation_deg{0.0f};           /**< 干扰来向俯仰角（单位：deg） */
  float angular_span_deg{0.0f};        /**< 干扰角域宽度（单位：deg） */
  bool in_sidelobe{false};             /**< 干扰是否主要经由旁瓣进入 */
  float confidence{1.0f};              /**< 干扰事实置信度，范围 [0, 1] */
};

/** @brief 单周期内可见的干扰源列表 */
using JammerSourceFactList = std::vector<JammerSourceFact>;

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
 * @brief JammerEmitterState 与 JammerSourceFact 共享相同的字段集，作为别名使用。
 * @details 场景输入（JammerEmitterState）与快照输出（JammerSourceFact）结构完全一致，
 *          统一为同一类型以消除逐字段复制。
 */
using JammerEmitterState = JammerSourceFact;

/** @brief 场景中的干扰源列表 */
using JammerEmitterStateList = std::vector<JammerEmitterState>;

/**
 * @brief EnvironmentSceneState 描述环境层待冻结的场景状态。
 */
struct EnvironmentSceneState {
  float base_propagation_loss_db{4.0f};     /**< 基础传播损耗（dB） */
  float atmospheric_attenuation_db{1.5f};   /**< 大气附加衰减（dB） */
  float terrain_reflection_db{1.0f};        /**< 地形/多径附加项（dB） */
  float clutter_power_db{3.0f};             /**< 杂波功率（dB） */
  JammerEmitterStateList jammer_emitters{}; /**< 当前场景中的干扰源输入 */
};

/**
 * @brief EnvironmentModelConfig 描述环境模型参数。
 */
struct EnvironmentModelConfig {
  float base_propagation_loss_db{4.0f};   /**< 基础传播损耗（dB） */
  float atmospheric_attenuation_db{1.5f}; /**< 大气附加衰减（dB） */
  float terrain_reflection_db{1.0f};      /**< 地形/多径附加项（dB） */
  float clutter_power_db{3.0f};           /**< 杂波功率（dB） */
  JammerSourceFactList jammer_sources{};  /**< 多源干扰事实输入 */
};

}  // namespace environment
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_TYPES_H_
