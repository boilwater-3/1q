/**
 * @file SbirsEnvironmentInput.h
 * @brief 定义 SBIRS-inspired 周期环境输入。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_ENVIRONMENT_INPUT_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_ENVIRONMENT_INPUT_H_

#include "1q/api.hpp"
#include "1q/sbirs_sensor/config/SbirsEnvironmentConfig.h"

namespace sbirs_sensor {
namespace session {

struct ONEQ_API SbirsEnvironmentInput {
  bool has_environment_override{false};
  config::SbirsEnvironmentConfig environment{};
};

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_ENVIRONMENT_INPUT_H_
