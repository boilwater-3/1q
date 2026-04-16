/**
 * @file EsrMissionControlConfig.h
 * @brief 定义 ESR 任务运行态控制参数结构。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_MISSION_CONTROL_CONFIG_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_MISSION_CONTROL_CONFIG_H_

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/config/EsrScanPolicyConfig.h"
#include "1q/electronic_surveillance_radar/config/EsrWorkMode.h"

namespace electronic_surveillance_radar {
namespace config {

/**
 * @brief EsrMissionControlConfig 描述 ESR 任务运行态控制参数。
 */
struct ONEQ_API EsrMissionControlConfig {
  bool power_on{true};                      /**< 设备开关机状态 */
  EsrWorkMode work_mode{EsrWorkMode::kEsm}; /**< 当前工作模式 */
  EsrScanPolicyConfig scan{};               /**< 扫描策略语义输入 */
};

}  // namespace config
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_MISSION_CONTROL_CONFIG_H_
