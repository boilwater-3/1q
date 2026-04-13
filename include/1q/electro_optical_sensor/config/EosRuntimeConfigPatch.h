/**
 * @file EosRuntimeConfigPatch.h
 * @brief 定义 EOS 会话运行期配置补丁结构。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_RUNTIME_CONFIG_PATCH_H_
#define ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_RUNTIME_CONFIG_PATCH_H_

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/config/EosWorkMode.h"
#include "1q/electro_optical_sensor/environment/EosEnvironmentTypes.h"

namespace electro_optical_sensor {
namespace session {

/**
 * @brief EosRuntimeConfigPatch 描述运行期可变参数补丁。
 */
struct ONEQ_API EosRuntimeConfigPatch {
  bool has_work_mode{false};
  EosWorkMode work_mode{EosWorkMode::kFused};

  bool has_scan_rate_deg_per_sec{false};
  float scan_rate_deg_per_sec{20.0f};

  bool has_frame_rate_hz{false};
  float frame_rate_hz{30.0f};

  bool has_minimum_snr_db{false};
  float minimum_snr_db{6.0f};

  bool has_enable_straylight_filter{false};
  bool enable_straylight_filter{false};

  bool has_visible_reference_irradiance_w_m2{false};
  float visible_reference_irradiance_w_m2{800.0f};

  bool has_environment_runtime_config{false};
  environment::EosEnvironmentRuntimeConfigPatch environment_runtime_config{};
};

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_RUNTIME_CONFIG_PATCH_H_
