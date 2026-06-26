/**
 * @file RadarEnvironmentInput.h
 * @brief 定义 AR 单周期环境输入聚合类型。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_RADAR_ENVIRONMENT_INPUT_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_RADAR_ENVIRONMENT_INPUT_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/config/RadarEnvironmentConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace session {

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
  config::JammingTechnique technique{config::JammingTechnique::kUnknown}; /**< 干扰技术类型 */
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
 * @brief RadarEnvironmentInputPatch 表示调用方侧环境事实状态的局部更新。
 *
 * @note 本类型不直接进入 RadarSession::StepWithResult()。调用方应先用
 *       RadarEnvironmentInputState 合成完整 RadarEnvironmentInput 快照，再写入
 *       RadarCycleInput::environment。
 */
struct ONEQ_API RadarEnvironmentInputPatch {
  bool has_atmospheric_observation{false};                         /**< 是否更新气象/电离层输入 */
  config::AtmosphericPhysicsConfig atmospheric_observation{}; /**< 新气象/电离层输入 */
  bool has_atmospheric_context{false};                             /**< 是否更新时间/空间天气输入 */
  config::AtmosphericDerivedContext atmospheric_context{};    /**< 新时间/空间天气输入 */
  bool has_surface_observation{false};                             /**< 是否更新地表/植被输入 */
  config::VegetationScatterPhysicsConfig surface_observation{}; /**< 新地表/植被输入 */
  bool has_jammer_sources{false};                                    /**< 是否更新干扰源列表 */
  config::JammerEmitterStateList jammer_sources{};              /**< 新干扰源列表 */
};

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
  std::uint32_t cycle_index{0U};                   /**< 当前周期号 */
  float cycle_dt_sec{0.0f};                        /**< 当前周期步长（单位：s） */
  float propagation_loss_db{0.0f};                 /**< 传播损耗（单位：dB） */
  float atmospheric_physics_loss_db{0.0f};         /**< 传播损耗中的大气物理附加项（单位：dB） */
  float clutter_power_db{0.0f};                    /**< 杂波功率估计（单位：dB） */
  config::AtmosphericPhysicsConfig atmospheric_physics{};  /**< 当前周期启用的大气物理参数 */
  config::AtmosphericDerivedContext atmospheric_context{}; /**< 当前周期时间/空间天气上下文输入 */
  float effective_k_factor{4.0f / 3.0f};           /**< 当前周期自动推导的有效地球半径因子 */
  std::int32_t effective_day_of_year{172};         /**< 当前周期自动推导的年积日 */
  JammerSourceFactList jammer_sources{};           /**< 当前周期可见的多源干扰事实 */
  bool jamming_detected{false};                    /**< 是否检测到干扰 */
};

/**
 * @brief EnvironmentSceneState 描述环境层待冻结的场景状态。
 */
struct ONEQ_API EnvironmentSceneState {
  config::AtmosphericPhysicsConfig atmospheric_physics{};              /**< 可选物理传播参数 */
  config::AtmosphericDerivedContext atmospheric_context{};             /**< 可选时间/空间天气上下文 */
  config::VegetationScatterPhysicsConfig vegetation_scatter_physics{}; /**< 可选植被散射参数 */
  config::JammerEmitterStateList jammer_emitters{};                    /**< 当前场景中的干扰源输入 */
};

/**
 * @brief RadarEnvironmentInput 聚合 AR 单周期环境事实输入。
 */
struct ONEQ_API RadarEnvironmentInput {
  config::AtmosphericPhysicsConfig atmospheric_observation{}; /**< 当前周期气象/电离层输入 */
  config::AtmosphericDerivedContext atmospheric_context{};    /**< 当前周期时间/空间天气输入 */
  config::VegetationScatterPhysicsConfig surface_observation{}; /**< 当前周期地表/植被输入 */
  config::JammerEmitterStateList jammer_sources{};              /**< 当前周期干扰源事实输入 */
};

/**
 * @brief RadarEnvironmentInputState 维护调用方侧当前环境事实状态。
 */
class ONEQ_API RadarEnvironmentInputState {
 public:
  RadarEnvironmentInputState() = default;
  explicit RadarEnvironmentInputState(const RadarEnvironmentInput& snapshot)
      : snapshot_(snapshot) {}

  RadarEnvironmentInputState& Reset(const RadarEnvironmentInput& snapshot) {
    snapshot_ = snapshot;
    return *this;
  }

  RadarEnvironmentInputState& Update(const RadarEnvironmentInputPatch& patch) {
    if (patch.has_atmospheric_observation) {
      snapshot_.atmospheric_observation = patch.atmospheric_observation;
    }
    if (patch.has_atmospheric_context) {
      snapshot_.atmospheric_context = patch.atmospheric_context;
    }
    if (patch.has_surface_observation) {
      snapshot_.surface_observation = patch.surface_observation;
    }
    if (patch.has_jammer_sources) {
      snapshot_.jammer_sources = patch.jammer_sources;
    }
    return *this;
  }

  RadarEnvironmentInput Snapshot() const { return snapshot_; }

 private:
  RadarEnvironmentInput snapshot_{};
};

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_RADAR_ENVIRONMENT_INPUT_H_
