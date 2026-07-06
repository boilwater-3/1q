/**
 * @file SbirsEnvironmentModel.h
 * @brief SBIRS-inspired 气象衰减模型。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_ENVIRONMENT_SBIRS_ENVIRONMENT_MODEL_H_
#define ONEQ_SRC_SBIRS_SENSOR_ENVIRONMENT_SBIRS_ENVIRONMENT_MODEL_H_

#include "1q/sbirs_sensor/config/SbirsEnvironmentConfig.h"

namespace sbirs_sensor {
namespace environment {

float ResolveWeatherAttenuation(const config::SbirsEnvironmentConfig& environment);
float ResolveEffectiveTransmittance(const config::SbirsEnvironmentConfig& environment);

}  // namespace environment
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_ENVIRONMENT_SBIRS_ENVIRONMENT_MODEL_H_
