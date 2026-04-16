/**
 * @file EsrWorkMode.h
 * @brief 定义 ESR 工作模式类型。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_WORK_MODE_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_WORK_MODE_H_

#include "1q/api.hpp"

namespace electronic_surveillance_radar {
namespace config {

/**
 * @brief EsrWorkMode 描述 ESR 工作模式。
 */
enum class ONEQ_API EsrWorkMode {
  kEsm = 0, /**< 常规电子支援侦察模式 */
  kHgesm,   /**< 高增益电子支援侦察模式 */
  kRwr      /**< 告警接收机模式 */
};

}  // namespace config
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_WORK_MODE_H_
