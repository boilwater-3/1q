/**
 * @file EsrRuntimeConfigPatch.h
 * @brief 定义 ESR 会话运行期配置补丁结构。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_RUNTIME_CONFIG_PATCH_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_RUNTIME_CONFIG_PATCH_H_

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/config/EsrEnvironmentPolicyConfig.h"
#include "1q/electronic_surveillance_radar/config/EsrScanPolicyConfig.h"
#include "1q/electronic_surveillance_radar/config/EsrWorkMode.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief EsrRuntimeConfigPatch 描述运行期可变参数补丁。
 */
struct ONEQ_API EsrRuntimeConfigPatch {
  bool has_sensor_enabled{false};
  bool sensor_enabled{true};

  bool has_work_mode{false};
  config::EsrWorkMode work_mode{config::EsrWorkMode::kEsm};

  bool has_scan_rate_hz{false};
  float scan_rate_hz{1.0f};

  bool has_scan_start_position{false};
  config::EsrScanStartPosition scan_start_position{config::EsrScanStartPosition::kLeftTop};

  bool has_scan_sequence{false};
  config::EsrScanSequence scan_sequence{config::EsrScanSequence::kAzimuthFirst};

  bool has_scan_center_az_deg{false};
  float scan_center_az_deg{0.0f};

  bool has_scan_center_el_deg{false};
  float scan_center_el_deg{0.0f};

  bool has_use_explicit_scan_bounds{false};
  bool use_explicit_scan_bounds{false};

  bool has_scan_start_az_deg{false};
  float scan_start_az_deg{-60.0f};
  bool has_scan_end_az_deg{false};
  float scan_end_az_deg{60.0f};
  bool has_scan_start_el_deg{false};
  float scan_start_el_deg{-10.0f};
  bool has_scan_end_el_deg{false};
  float scan_end_el_deg{10.0f};

  bool has_environment_preset{false};
  config::EsrEnvironmentPreset environment_preset{config::EsrEnvironmentPreset::kStandard};
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_RUNTIME_CONFIG_PATCH_H_
