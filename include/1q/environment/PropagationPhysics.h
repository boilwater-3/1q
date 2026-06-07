/**
 * @file PropagationPhysics.h
 * @brief 定义大气传播物理算法公开 API。
 *
 * 将 common/atmosphere/AtmospherePhysics 中的 internal 接口提升为公开 API，
 * 供 AR/ESR/EOS 各模块统一消费。
 */

#ifndef ONEQ_ENVIRONMENT_PROPAGATION_PHYSICS_H_
#define ONEQ_ENVIRONMENT_PROPAGATION_PHYSICS_H_

#include "1q/api.hpp"
#include "1q/environment/AtmosphericTypes.h"

namespace oneq {
namespace environment {

/**
 * @brief 大气传播物理模型输入。
 */
struct ONEQ_API PropagationInputs {
  float frequency_hz{10.0e9f};       /**< 雷达频率（单位：Hz） */
  float path_length_m{10.0e3f};      /**< 传播路径长度（单位：m） */
  float radar_altitude_m{1.0e3f};    /**< 雷达高度（单位：m） */
  float target_altitude_m{1.0e3f};   /**< 目标高度（单位：m） */
  float elevation_deg{5.0f};         /**< 传播仰角（单位：deg） */
  AtmosphericObservation observation{}; /**< 大气观测输入 */
  bool has_space_weather_context{false};        /**< 是否提供空间天气上下文 */
  SpaceWeatherContext space_weather_context{};  /**< 空间天气高级上下文（可选） */
};

/**
 * @brief 大气传播物理模型输出。
 */
struct ONEQ_API PropagationResult {
  float blake_loss_db{0.0f};            /**< Blake 传播损耗（单位：dB） */
  float refractivity_index{1.0003f};    /**< 折射率 n */
  float refractivity_index_h{1.0003f};  /**< 高度修正折射率 n(h) */
  float neutral_density_kg_m3{1.225f};  /**< 中性大气密度（单位：kg/m^3） */
  float total_physics_loss_db{0.0f};    /**< 聚合物理附加损耗（单位：dB） */
};

/**
 * @brief 计算传播路径的物理附加损耗。
 *
 * 当 PropagationInputs.has_space_weather_context 为 true 时，
 * 使用 SpaceWeatherContext 中的 k_factor/day_of_year/solar_flux/geomagnetic_ap；
 * 否则从 AtmosphericObservation 推导 k_factor，其余使用默认值。
 *
 * @param[in] inputs 传播物理模型输入。
 * @return 传播物理模型输出。
 */
ONEQ_API PropagationResult EvaluatePropagation(const PropagationInputs& inputs);

/**
 * @brief 从大气状态和传感器参数构建 PropagationInputs。
 *
 * 消除各模块手工填充字段的重复代码。
 * @param frequency_hz 频率（Hz）
 * @param path_length_m 路径长度（m）
 * @param radar_altitude_m 雷达高度（m）
 * @param target_altitude_m 目标高度（m）
 * @param elevation_deg 仰角（deg）
 * @param observation 大气观测输入
 * @return 构造好的 PropagationInputs
 */
ONEQ_API PropagationInputs BuildPropagationInputs(
    float frequency_hz, float path_length_m, float radar_altitude_m,
    float target_altitude_m, float elevation_deg,
    const AtmosphericObservation& observation);

/**
 * @brief REOS 对齐入口：Blake 大气损耗。
 */
ONEQ_API float BlakeAtmosphericLoss(float altitude_m, float frequency_hz,
                                    float elevation_deg, float range_m, float k_factor);

/**
 * @brief REOS 对齐入口：折射率 n。
 */
ONEQ_API float RefractivityIndex(float tc_celsius, float tk_kelvin, float pd_hpa,
                                 float p_hpa, float h_rel, int water_or_ice);

}  // namespace environment
}  // namespace oneq

#endif  // ONEQ_ENVIRONMENT_PROPAGATION_PHYSICS_H_
