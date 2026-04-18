/**
 * @file EsrEnvironmentConfig.h
 * @brief 定义 ESR 环境域配置。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_ENVIRONMENT_CONFIG_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_ENVIRONMENT_CONFIG_H_

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/config/EsrEnvironmentPreset.h"
#include "1q/foundation/atmospheric_types.h"

namespace electronic_surveillance_radar {
namespace config {

using EsrAtmosphericPhysicsConfig = oneq::foundation::AtmosphericObservation;
using EsrAtmosphericDerivedContext = oneq::foundation::SpaceWeatherContext;

/**
 * @brief EsrEnvironmentConfig 描述环境域配置。
 */
struct ONEQ_API EsrEnvironmentConfig {
  EsrEnvironmentPreset preset{EsrEnvironmentPreset::kStandard}; /**< 环境预设 */
  bool use_preset_defaults{true};                     /**< true 表示使用 preset 的默认参数映射 */
  EsrAtmosphericPhysicsConfig atmospheric_physics{};  /**< 详细参数：基础气象观测 */
  EsrAtmosphericDerivedContext atmospheric_context{}; /**< 详细参数：空间天气上下文 */
};

}  // namespace config
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_ENVIRONMENT_CONFIG_H_
