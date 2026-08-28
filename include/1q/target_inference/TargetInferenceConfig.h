/**
 * @file TargetInferenceConfig.h
 * @brief 定义目标推演引擎配置（弹道模型与判决阈值）。
 */

#ifndef ONEQ_TARGET_INFERENCE_TARGET_INFERENCE_CONFIG_H_
#define ONEQ_TARGET_INFERENCE_TARGET_INFERENCE_CONFIG_H_

#include <cstdint>
#include <string>

#include "1q/api.hpp"

namespace target_inference {

/**
 * @brief 目标推演配置。
 * @note 弹道模型为中心引力 + 可选指数大气阻力（ballistic coefficient ≤ 0 关闭）；
 *       助推段未建模——发射点定义为回推弹道与地表交点（见 algorithms.md 边界）。
 */
struct ONEQ_API TargetInferenceConfig {
  double earth_mu_m3_per_s2{3.986004418e14}; /**< 地球引力参数（单位：m³/s²）。 */
  double earth_radius_m{6378137.0};          /**< 地球半径（单位：m）。 */
  double prediction_horizon_sec{300.0};      /**< 轨迹预测时域（单位：s）。 */
  double integration_step_sec{1.0};          /**< 弹道积分步长（单位：s）。 */
  double waypoint_interval_sec{10.0};        /**< 航迹点输出间隔（单位：s）。 */
  double launch_speed_threshold_m_per_s{30.0}; /**< 回推速度停机门（单位：m/s）。 */
  double launch_max_backtrack_sec{900.0};    /**< 回推时长上限（单位：s）。 */
  double drag_ballistic_coefficient_m2_per_kg{0.0}; /**< 弹道系数（≤ 0 关闭阻力）。 */
  double kinematic_type_weight{0.5};         /**< 运动学先验 vs 外部证据的融合权重。 */
  double high_energy_speed_threshold_m_per_s{1200.0}; /**< 高能量速度门（单位：m/s）。 */
  double high_altitude_threshold_m{30000.0}; /**< 高空门（单位：m）。 */
  /** 落点预报分发通道名（如事件信号名）。装配层接好外发通道后填入，验收行据此写
   *  「分发状态=已发布」；留空 = 未接外发，写「已封装待分发」。 */
  std::string impact_distribution_channel{};
  /** 落点预报分发来源实体 ID（验收判定标准 第25项：发布方实体 ID，不用通道名代替；
   *  0 = 装配层未标注，已发布行省略该字段）。 */
  std::uint64_t impact_distribution_source_id{0U};
  /** 落点预报分发对象实体 ID（接收方；0 = 未标注，省略）。 */
  std::uint64_t impact_distribution_target_id{0U};
};

}  // namespace target_inference

#endif  // ONEQ_TARGET_INFERENCE_TARGET_INFERENCE_CONFIG_H_
