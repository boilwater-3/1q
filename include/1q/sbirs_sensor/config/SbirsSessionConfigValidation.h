/**
 * @file SbirsSessionConfigValidation.h
 * @brief 定义 SBIRS-inspired 配置校验入口。
 */

#ifndef ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_SESSION_CONFIG_VALIDATION_H_
#define ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_SESSION_CONFIG_VALIDATION_H_

#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/foundation/validation_types.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"

namespace sbirs_sensor {
namespace config {

struct ONEQ_API ValidationIssue {
  oneq::foundation::ValidationSeverity severity{oneq::foundation::ValidationSeverity::kInfo};
  std::string message{};
};

using ValidationIssueList = std::vector<ValidationIssue>;

ONEQ_API ValidationIssueList ValidateSbirsSessionConfig(const SbirsSessionConfig& config);

}  // namespace config
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_SESSION_CONFIG_VALIDATION_H_
