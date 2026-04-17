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
  bool has_sensor_enabled{false}; /**< 是否显式设置传感器开关状态 */
  bool sensor_enabled{true};      /**< 传感器开关状态 */

  bool has_work_mode{false};                   /**< 是否显式设置工作模式 */
  config::EsrWorkMode work_mode{config::EsrWorkMode::kEsm}; /**< 工作模式值 */

  bool has_scan_rate_hz{false}; /**< 是否显式设置扫描数据率 */
  float scan_rate_hz{1.0f};     /**< 扫描数据率（单位：Hz） */

  bool has_scan_start_position{false}; /**< 是否显式设置扫描起始位置 */
  config::EsrScanStartPosition scan_start_position{config::EsrScanStartPosition::kLeftTop};

  bool has_scan_sequence{false}; /**< 是否显式设置扫描顺序 */
  config::EsrScanSequence scan_sequence{config::EsrScanSequence::kAzimuthFirst};

  bool has_scan_center_az_deg{false}; /**< 是否显式设置扫描中心方位角 */
  float scan_center_az_deg{0.0f};     /**< 扫描中心方位角（单位：deg） */

  bool has_scan_center_el_deg{false}; /**< 是否显式设置扫描中心俯仰角 */
  float scan_center_el_deg{0.0f};     /**< 扫描中心俯仰角（单位：deg） */

  bool has_use_explicit_scan_bounds{false}; /**< 是否显式设置扫描边界模式 */
  bool use_explicit_scan_bounds{false};      /**< 是否使用显式扫描起止角 */

  bool has_scan_start_az_deg{false}; /**< 是否显式设置扫描起始方位角 */
  float scan_start_az_deg{-60.0f};   /**< 扫描起始方位角（单位：deg） */
  bool has_scan_end_az_deg{false};   /**< 是否显式设置扫描结束方位角 */
  float scan_end_az_deg{60.0f};      /**< 扫描结束方位角（单位：deg） */
  bool has_scan_start_el_deg{false}; /**< 是否显式设置扫描起始俯仰角 */
  float scan_start_el_deg{-10.0f};   /**< 扫描起始俯仰角（单位：deg） */
  bool has_scan_end_el_deg{false};   /**< 是否显式设置扫描结束俯仰角 */
  float scan_end_el_deg{10.0f};      /**< 扫描结束俯仰角（单位：deg） */

  bool has_environment_preset{false}; /**< 是否显式设置环境预设 */
  config::EsrEnvironmentPreset environment_preset{config::EsrEnvironmentPreset::kStandard};
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_RUNTIME_CONFIG_PATCH_H_
