/**
 * @file EosEnvironmentInputState.h
 * @brief 定义 EOS 调用方侧环境输入状态维护对象。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_SESSION_EOS_ENVIRONMENT_INPUT_STATE_H_
#define ELECTRO_OPTICAL_SENSOR_SESSION_EOS_ENVIRONMENT_INPUT_STATE_H_

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/session/EosEnvironmentInputPatch.h"

namespace electro_optical_sensor {
namespace session {

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

#endif  // ELECTRO_OPTICAL_SENSOR_SESSION_EOS_ENVIRONMENT_INPUT_STATE_H_
