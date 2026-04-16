/**
 * @file EosRuntimeConfigPatch.h
 * @brief 定义 EOS 会话运行期配置补丁结构。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_RUNTIME_CONFIG_PATCH_H_
#define ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_RUNTIME_CONFIG_PATCH_H_

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/config/EosDetectionPolicyConfig.h"
#include "1q/electro_optical_sensor/config/EosEnvironmentPolicyConfig.h"
#include "1q/electro_optical_sensor/config/EosStrayLightPolicyConfig.h"
#include "1q/electro_optical_sensor/config/EosWorkMode.h"
#include "1q/electro_optical_sensor/environment/EosEnvironmentConfig.h"

namespace electro_optical_sensor {
namespace session {

/**
 * @brief EosRuntimeConfigPatch 描述运行期可变高层策略补丁。
 */
struct ONEQ_API EosRuntimeConfigPatch {
  bool has_work_mode{false};
  EosWorkMode work_mode{EosWorkMode::kFused};

  bool has_scan_rate_deg_per_sec{false};
  float scan_rate_deg_per_sec{20.0f};

  bool has_frame_rate_hz{false};
  float frame_rate_hz{30.0f};

  bool has_detection_profile{false};
  config::EosDetectionProfile detection_profile{config::EosDetectionProfile::kBalanced};

  bool has_stray_light_profile{false};
  config::EosStrayLightProfile stray_light_profile{config::EosStrayLightProfile::kDisabled};

  bool has_environment_model_type{false};
  environment::EosEnvironmentModelType environment_model_type{
      environment::EosEnvironmentModelType::kSimplified};

  bool has_environment_preset{false};
  config::EosEnvironmentPreset environment_preset{config::EosEnvironmentPreset::kStandard};
};

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_RUNTIME_CONFIG_PATCH_H_
