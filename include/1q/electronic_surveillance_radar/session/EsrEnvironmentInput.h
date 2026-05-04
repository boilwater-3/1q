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

/** @brief EsrEnvironmentInput 表示 ESR 单周期环境事实输入。 */
using EsrEnvironmentInput = environment::EsrEnvironmentObservation;

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_ENVIRONMENT_INPUT_H_
