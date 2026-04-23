/**
 * @file EsrEnvironmentInput.h
 * @brief 定义 ESR 单周期环境输入聚合类型。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_ENVIRONMENT_INPUT_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_ENVIRONMENT_INPUT_H_

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentTypes.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief EsrEnvironmentInput 聚合 ESR 单周期环境事实输入。
 */
struct ONEQ_API EsrEnvironmentInput {
  environment::EsrEnvironmentObservation observation{}; /**< 本周期环境高层观测输入 */
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_ENVIRONMENT_INPUT_H_
