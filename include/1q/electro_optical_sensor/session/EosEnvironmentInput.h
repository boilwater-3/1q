/**
 * @file EosEnvironmentInput.h
 * @brief 定义 EOS 单周期环境输入聚合类型。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_SESSION_EOS_ENVIRONMENT_INPUT_H_
#define ELECTRO_OPTICAL_SENSOR_SESSION_EOS_ENVIRONMENT_INPUT_H_

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/environment/EosEnvironmentTypes.h"

namespace electro_optical_sensor {
namespace session {

using DayNightType = environment::DayNightType;

/** @brief EosEnvironmentInput 表示 EOS 单周期环境事实输入。 */
using EosEnvironmentInput = environment::EosEnvironmentObservation;

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_SESSION_EOS_ENVIRONMENT_INPUT_H_
