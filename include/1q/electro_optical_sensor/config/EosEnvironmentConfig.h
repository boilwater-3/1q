/**
 * @file EosEnvironmentConfig.h
 * @brief 定义 EOS 环境域配置。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_ENVIRONMENT_CONFIG_H_
#define ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_ENVIRONMENT_CONFIG_H_

#include "1q/electro_optical_sensor/environment/EosEnvironmentConfig.h"
#include "1q/electro_optical_sensor/foundation/EosRadiativeTransfer.h"

namespace electro_optical_sensor {
namespace config {

/**
 * @brief EosEnvironmentPreset 描述高层环境预设。
 */
enum class EosEnvironmentPreset {
  kStandard = 0,
  kHumid,
  kDusty,
  kTurbulent,
  kMaritime
};

/**
 * @brief EosEnvironmentConfig 描述环境域配置。
 */
struct EosEnvironmentConfig {
  environment::EosEnvironmentModelType model_type{
      environment::EosEnvironmentModelType::kSimplified}; /**< 环境模型类型 */
  EosEnvironmentPreset preset{EosEnvironmentPreset::kStandard}; /**< 环境预设 */
  bool use_preset_defaults{true}; /**< true 表示使用 preset 的默认参数映射 */
  foundation::radiative_transfer::RadiativeTransferModel radiative_transfer_model{
      foundation::radiative_transfer::RadiativeTransferModel::kDerivedBeerLambert};
  float aerosol_density_factor{1.0f}; /**< 详细参数：气溶胶密度因子 */
  float turbulence_factor{1.0f}; /**< 详细参数：湍流强度因子 */
  bool enable_optical_countermeasure_extension{false}; /**< 预留：是否启用光学对抗场景扩展 */
};

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_ENVIRONMENT_CONFIG_H_
