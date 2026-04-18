/**
 * @file EsrMissionConfig.h
 * @brief 定义 ESR 任务域配置。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_MISSION_CONFIG_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_MISSION_CONFIG_H_

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/config/EsrScanPolicyConfig.h"
#include "1q/electronic_surveillance_radar/config/EsrWorkMode.h"

namespace electronic_surveillance_radar {
namespace config {

/**
 * @brief EsrMissionConfig 描述 ESR 任务控制与扫描语义输入。
 */
struct ONEQ_API EsrMissionConfig {
  bool power_on{true};                      /**< 设备开关机状态 */
  EsrWorkMode work_mode{EsrWorkMode::kEsm}; /**< 当前工作模式 */
  EsrScanPolicyConfig scan{};               /**< 扫描策略语义输入 */
};

}  // namespace config
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_MISSION_CONFIG_H_
