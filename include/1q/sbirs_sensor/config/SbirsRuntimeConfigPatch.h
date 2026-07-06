/**
 * @file SbirsRuntimeConfigPatch.h
 * @brief 定义 SBIRS-inspired 运行期配置补丁。
 */

#ifndef ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_RUNTIME_CONFIG_PATCH_H_
#define ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_RUNTIME_CONFIG_PATCH_H_

#include "1q/api.hpp"
#include "1q/sbirs_sensor/config/SbirsEnvironmentConfig.h"
#include "1q/sbirs_sensor/config/SbirsMissionConfig.h"
#include "1q/sbirs_sensor/config/SbirsPolicyConfig.h"

namespace sbirs_sensor {
namespace config {

struct ONEQ_API SbirsRuntimeConfigPatch {
  bool has_mission{false};
  SbirsMissionConfig mission{};

  bool has_policy{false};
  SbirsPolicyConfig policy{};

  bool has_environment{false};
  SbirsEnvironmentConfig environment{};

  bool has_work_mode{false};
  SbirsWorkMode work_mode{SbirsWorkMode::kSearchAndStare};

  bool has_scan_rate_deg_per_sec{false};
  float scan_rate_deg_per_sec{10.0f};

  bool has_sensor_enabled{false};
  bool sensor_enabled{true};
};

}  // namespace config
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_RUNTIME_CONFIG_PATCH_H_
