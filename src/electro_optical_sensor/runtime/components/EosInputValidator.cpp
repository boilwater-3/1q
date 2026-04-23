#include "electro_optical_sensor/runtime/components/EosInputValidator.h"

namespace electro_optical_sensor {
namespace runtime {
namespace components {

session::ValidationIssueList EosInputValidator::Validate(
    const ::electro_optical_sensor::session::EosCycleInput& input) const {
  return session::ValidateEosCycleInput(input);
}

bool EosInputValidator::HasError(const session::ValidationIssueList& issues) const {
  return session::HasValidationError(issues);
}

}  // namespace components
}  // namespace runtime
}  // namespace electro_optical_sensor
