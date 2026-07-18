/**
 * @file SarEnvironmentConfig.h
 * @brief 定义 SAR 环境与传播默认配置。
 */

#ifndef ONEQ_SAR_CONFIG_SAR_ENVIRONMENT_CONFIG_H_
#define ONEQ_SAR_CONFIG_SAR_ENVIRONMENT_CONFIG_H_

#include "1q/api.hpp"

namespace sar {
namespace config {

/**
 * @brief SAR 环境默认配置。
 *
 * @note 内部生成 raw echo 时，地形参考高程定义局部坐标原点高度；
 *       `use_flat_earth_geometry` 在等距柱状局部近似与 WGS-84 ENU 之间选择。
 *       大气衰减和表面后向散射字段仍为保留字段，仅参与 replay/config 保真，
 *       暂不影响计算结果。
 */
struct ONEQ_API SarEnvironmentConfig {
  double terrain_reference_altitude_m{0.0};   /**< 内部生成路径的局部坐标原点高度（m） */
  double atmospheric_loss_db_per_km{0.0};     /**< 保留字段：大气损耗，当前不驱动任何计算阶段 */
  double surface_backscatter_sigma0_db{-12.0}; /**< 保留字段：表面后向散射系数，当前不驱动任何计算阶段 */
  bool use_flat_earth_geometry{true};         /**< true 使用局部扁平近似；false 使用 WGS-84 ENU */
  bool enable_atmospheric_attenuation{false}; /**< 保留字段：大气衰减开关，当前不驱动任何计算阶段 */
};

}  // namespace config
}  // namespace sar

#endif  // ONEQ_SAR_CONFIG_SAR_ENVIRONMENT_CONFIG_H_
