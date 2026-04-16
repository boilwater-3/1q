/**
 * @file EosScanPolicyConfig.h
 * @brief 定义 EOS 扫描策略配置。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SCAN_POLICY_CONFIG_H_
#define ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SCAN_POLICY_CONFIG_H_

#include "1q/electro_optical_sensor/config/EosWorkMode.h"

namespace electro_optical_sensor {
namespace config {

/**
 * @brief EosScanPolicyConfig 描述任务级扫描策略。
 */
struct EosScanPolicyConfig {
  session::EosWorkMode work_mode{session::EosWorkMode::kFused};
  float horizontal_fov_deg{6.0f};
  float vertical_fov_deg{4.0f};
  float scan_rate_deg_per_sec{20.0f};
  float frame_rate_hz{30.0f};
};

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SCAN_POLICY_CONFIG_H_
