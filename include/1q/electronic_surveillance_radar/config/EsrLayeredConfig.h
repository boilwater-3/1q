/**
 * @file EsrLayeredConfig.h
 * @brief 定义 ESR 分层参数入口结构。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_LAYERED_CONFIG_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_LAYERED_CONFIG_H_

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/config/EsrHardwareConfig.h"
#include "1q/electronic_surveillance_radar/config/EsrMissionControlConfig.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief EsrLayeredConfig 描述 ESR 分层参数入口。
 */
struct ONEQ_API EsrLayeredConfig {
  EsrHardwareConfig hardware{};      /**< 装备固有参数 */
  EsrMissionControlConfig mission{}; /**< 任务控制参数 */
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_LAYERED_CONFIG_H_
