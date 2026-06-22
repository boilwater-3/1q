#ifndef ONEQ_SRC_SAR_SESSION_SAR_RUNTIME_CONFIG_VALIDATION_H_
#define ONEQ_SRC_SAR_SESSION_SAR_RUNTIME_CONFIG_VALIDATION_H_

#include "1q/sar/config/SarSessionConfig.h"
#include "1q/sar/session/SarCycleResult.h"

namespace sar {
namespace session {

bool ValidateRuntimeConfigForStep(const config::SarSessionConfig& config,
                                  bool has_external_raw_iq,
                                  SarCycleResult* result);

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SRC_SAR_SESSION_SAR_RUNTIME_CONFIG_VALIDATION_H_

