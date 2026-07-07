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

/**
 * @brief 单周期环境输入，可携带覆盖 session 配置的环境参数。
 * @note `has_environment_override` 为真时使用 `environment` 覆盖；否则沿用 session 配置。
 */
struct ONEQ_API SbirsEnvironmentInput {
  bool has_environment_override{false};      /**< 是否覆盖环境参数 */
  config::SbirsEnvironmentConfig environment{}; /**< 环境参数覆盖值 */
};

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_ENVIRONMENT_INPUT_H_
