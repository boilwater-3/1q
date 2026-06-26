/**
 * @file EosEnvironmentInput.h
 * @brief 定义 EOS 单周期环境输入、模型输入输出类型。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_ENVIRONMENT_INPUT_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_ENVIRONMENT_INPUT_H_

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/config/EosEnvironmentConfig.h"

namespace electro_optical_sensor {
namespace session {

/**
 * @brief DayNightType 表示昼夜环境类型。
 */
enum class ONEQ_API DayNightType {
  kDay = 0,
  kNight,
  kTwilight
};

/**
 * @brief EosEnvironmentInput 描述 EOS 单周期环境高层观测输入。
 */
struct ONEQ_API EosEnvironmentInput {
  float solar_altitude_deg{45.0f};
  float solar_azimuth_deg{180.0f};
  float solar_irradiance_w_m2{800.0f};
  float cloud_coverage_ratio{0.2f};
  float ambient_wind_speed_mps{0.0f};
  DayNightType day_night_type{DayNightType::kDay};
  float background_temperature_k{290.0f};
};

/**
 * @brief EosEnvironmentModelInputs 描述环境模型输入。
 */
struct ONEQ_API EosEnvironmentModelInputs {
  config::EosEnvironmentModelType model_type{config::EosEnvironmentModelType::kSimplified};
  foundation::radiative_transfer::RadiativeTransferModel radiative_transfer_model{
      foundation::radiative_transfer::RadiativeTransferModel::kDerivedBeerLambert};
  float base_aerosol_density_factor{1.0f};
  float base_turbulence_factor{1.0f};
  float platform_altitude_m{0.0f};
  float cloud_coverage_ratio{0.0f};
  float wind_speed_mps{0.0f};
  bool has_atmospheric_observation{false};
  oneq::environment::AtmosphericObservation atmospheric_observation{};
};

/**
 * @brief EosEnvironmentModelResult 描述环境模型输出。
 */
struct ONEQ_API EosEnvironmentModelResult {
  float aerosol_density_factor{1.0f};
  float turbulence_factor{1.0f};
  float path_radiance_scale_bias{1.0f};
};

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_ENVIRONMENT_INPUT_H_
