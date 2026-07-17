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
  float base_aerosol_density_factor{1.0f};
  float base_turbulence_factor{1.0f};
  float platform_altitude_m{0.0f};
  float cloud_coverage_ratio{0.0f};
  float wind_speed_mps{0.0f};
  oneq::environment::AtmosphericObservation atmospheric_physics{};
};

/**
 * @brief EosEnvironmentModelResult 描述环境模型输出。
 */
struct ONEQ_API EosEnvironmentModelResult {
  float aerosol_density_factor{1.0f};
  float turbulence_factor{1.0f};
  float path_radiance_scale_bias{1.0f};
};

/**
 * @brief EosEnvironmentInputPatch 表示调用方侧环境事实状态的局部更新。
 *
 * @note 本类型不直接进入 EosSession::StepWithResult()。调用方应先用
 *       EosEnvironmentInputState 合成完整 EosEnvironmentInput 快照，再写入
 *       EosCycleInput::environment。
 */
struct ONEQ_API EosEnvironmentInputPatch {
  bool has_solar_altitude_deg{false};              /**< 是否更新太阳高度角 */
  float solar_altitude_deg{45.0f};                 /**< 新太阳高度角（单位：deg） */
  bool has_solar_azimuth_deg{false};               /**< 是否更新太阳方位角 */
  float solar_azimuth_deg{180.0f};                 /**< 新太阳方位角（单位：deg） */
  bool has_solar_irradiance_w_m2{false};           /**< 是否更新太阳辐照度 */
  float solar_irradiance_w_m2{800.0f};             /**< 新太阳辐照度（单位：W/m^2） */
  bool has_cloud_coverage_ratio{false};            /**< 是否更新云量 */
  float cloud_coverage_ratio{0.2f};                /**< 新云量，范围 [0, 1] */
  bool has_ambient_wind_speed_mps{false};          /**< 是否更新环境风速 */
  float ambient_wind_speed_mps{0.0f};              /**< 新环境风速（单位：m/s） */
  bool has_day_night_type{false};                  /**< 是否更新昼夜类型 */
  DayNightType day_night_type{DayNightType::kDay}; /**< 新昼夜类型 */
  bool has_background_temperature_k{false};        /**< 是否更新背景温度 */
  float background_temperature_k{290.0f};          /**< 新背景温度（单位：K） */
};

/**
 * @brief EosEnvironmentInputState 维护调用方侧当前环境事实状态。
 */
class ONEQ_API EosEnvironmentInputState {
 public:
  EosEnvironmentInputState() = default;
  explicit EosEnvironmentInputState(const EosEnvironmentInput& snapshot) : snapshot_(snapshot) {}

  EosEnvironmentInputState& Reset(const EosEnvironmentInput& snapshot) {
    snapshot_ = snapshot;
    return *this;
  }

  EosEnvironmentInputState& Update(const EosEnvironmentInputPatch& patch) {
    if (patch.has_solar_altitude_deg) {
      snapshot_.solar_altitude_deg = patch.solar_altitude_deg;
    }
    if (patch.has_solar_azimuth_deg) {
      snapshot_.solar_azimuth_deg = patch.solar_azimuth_deg;
    }
    if (patch.has_solar_irradiance_w_m2) {
      snapshot_.solar_irradiance_w_m2 = patch.solar_irradiance_w_m2;
    }
    if (patch.has_cloud_coverage_ratio) {
      snapshot_.cloud_coverage_ratio = patch.cloud_coverage_ratio;
    }
    if (patch.has_ambient_wind_speed_mps) {
      snapshot_.ambient_wind_speed_mps = patch.ambient_wind_speed_mps;
    }
    if (patch.has_day_night_type) {
      snapshot_.day_night_type = patch.day_night_type;
    }
    if (patch.has_background_temperature_k) {
      snapshot_.background_temperature_k = patch.background_temperature_k;
    }
    return *this;
  }

  EosEnvironmentInput Snapshot() const { return snapshot_; }

 private:
  EosEnvironmentInput snapshot_{};
};

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_ENVIRONMENT_INPUT_H_
