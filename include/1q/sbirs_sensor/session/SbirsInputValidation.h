/**
 * @file SbirsInputValidation.h
 * @brief 定义 SBIRS-inspired 输入校验类型与入口。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_INPUT_VALIDATION_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_INPUT_VALIDATION_H_

#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/foundation/validation_types.h"

namespace sbirs_sensor {
namespace session {

using ValidationSeverity = oneq::foundation::ValidationSeverity;
using ValidationLocation = oneq::foundation::ValidationLocation;
using ValidationLocationKind = oneq::foundation::ValidationLocationKind;

struct ONEQ_API ValidationIssue {
  ValidationSeverity severity{ValidationSeverity::kInfo};
  ValidationLocation location{};
  std::string message{};
};

using ValidationIssueList = std::vector<ValidationIssue>;

struct SbirsCycleInput;

ONEQ_API ValidationIssueList ValidateSbirsCycleInput(const SbirsCycleInput& input);
ONEQ_API bool HasValidationError(const ValidationIssueList& issues);

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_INPUT_VALIDATION_H_
