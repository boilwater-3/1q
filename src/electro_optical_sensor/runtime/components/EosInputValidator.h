#ifndef ELECTRO_OPTICAL_SENSOR_RUNTIME_COMPONENTS_EOS_INPUT_VALIDATOR_H_
#define ELECTRO_OPTICAL_SENSOR_RUNTIME_COMPONENTS_EOS_INPUT_VALIDATOR_H_

#include "1q/electro_optical_sensor/session/EosInputValidation.h"

namespace electro_optical_sensor {
namespace runtime {
namespace components {

class EosInputValidator {
 public:
  session::ValidationIssueList Validate(
      const ::electro_optical_sensor::session::EosCycleInput& input) const;

  bool HasError(const session::ValidationIssueList& issues) const;
};

}  // namespace components
}  // namespace runtime
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_RUNTIME_COMPONENTS_EOS_INPUT_VALIDATOR_H_
