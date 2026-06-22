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
 */
struct ONEQ_API SarEnvironmentConfig {
  double terrain_reference_altitude_m{0.0};
  double atmospheric_loss_db_per_km{0.0};
  double surface_backscatter_sigma0_db{-12.0};
  bool use_flat_earth_geometry{true};
  bool enable_atmospheric_attenuation{false};
};

}  // namespace config
}  // namespace sar

#endif  // ONEQ_SAR_CONFIG_SAR_ENVIRONMENT_CONFIG_H_
