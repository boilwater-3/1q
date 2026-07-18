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
 * @param[in] frequency_hz 频率（单位：Hz）。
 * @param[in] path_length_m 路径长度（单位：m）。
 * @param[in] radar_altitude_m 雷达高度（单位：m）。
 * @param[in] target_altitude_m 目标高度（单位：m）。
 * @param[in] elevation_deg 仰角（单位：deg）。
 * @param[in] observation 大气观测输入。
 * @return 构造好的 PropagationInputs（has_space_weather_context 为 false）。
 */
ONEQ_API PropagationInputs BuildPropagationInputs(
    float frequency_hz, float path_length_m, float radar_altitude_m,
    float target_altitude_m, float elevation_deg,
    const AtmosphericObservation& observation);

/**
 * @brief REOS 对齐入口：Blake 大气损耗（单精度，转发到内部 r4 实现）。
 * @param[in] altitude_m 雷达高度（单位：m）。
 * @param[in] frequency_hz 雷达频率（单位：Hz）。
 * @param[in] elevation_deg 传播仰角（单位：deg）。
 * @param[in] range_m 传播距离（单位：m）。
 * @param[in] k_factor 有效地球半径因子。
 * @return Blake 双程大气损耗（单位：dB）。
 */
ONEQ_API float BlakeAtmosphericLoss(float altitude_m, float frequency_hz,
                                    float elevation_deg, float range_m, float k_factor);

/**
 * @brief 同一热力学温度的摄氏/开氏成对表示。
 * @note `TryRefractivityIndex` 校验两者满足 kelvin ≈ celsius + 273.15。
 */
struct ONEQ_API RefractivityTemperaturePair {
  float celsius{15.0f}; /**< 摄氏温度（单位：°C） */
  float kelvin{288.15f}; /**< 开氏温度（单位：K） */
};

/** @brief 强类型折射率输入，避免六个裸标量的温标位置混淆。 */
struct ONEQ_API RefractivityInputs {
  RefractivityTemperaturePair temperature{}; /**< 同一温度的成对温标 */
  float partial_pressure_hpa{0.0f};           /**< REOS `pd` 分压参数（hPa） */
  float total_pressure_hpa{1013.25f};         /**< 总气压（hPa） */
  float relative_humidity{0.5f};              /**< 相对湿度 [0, 1] */
  int water_or_ice{0};                        /**< 0=水，1=冰 */
};

/**
 * @brief 校验强类型输入并计算大气折射率 n。
 * @param[in] inputs 强类型折射率输入。
 * @param[out] refractivity_index 成功时写入折射率；失败时保持原值。
 * @return 温标一致、其余标量合法且计算结果有限时返回 true，否则返回 false。
 */
ONEQ_API bool TryRefractivityIndex(const RefractivityInputs& inputs,
                                   float* refractivity_index);

/**
 * @brief REOS 对齐兼容入口：折射率 n（单精度，转发到内部 r4 实现）。
 * @note 保留历史六标量数值语义；新代码应使用 `TryRefractivityIndex`。
 * @param[in] tc_celsius 摄氏温度（单位：°C）。
 * @param[in] tk_kelvin 开氏温度（单位：K）。
 * @param[in] pd_hpa 水汽分压（单位：hPa）。
 * @param[in] p_hpa 总气压（单位：hPa）。
 * @param[in] h_rel 相对湿度 [0, 1]。
 * @param[in] water_or_ice 介质标志（水/冰），沿用 REOS 约定。
 * @return 大气折射率 n。
 */
ONEQ_API float RefractivityIndex(float tc_celsius, float tk_kelvin, float pd_hpa,
                                 float p_hpa, float h_rel, int water_or_ice);

}  // namespace environment
}  // namespace oneq

#endif  // ONEQ_ENVIRONMENT_PROPAGATION_PHYSICS_H_
