#ifndef ELECTRONIC_SURVEILLANCE_RADAR_RUNTIME_COMPONENTS_ESR_OUTPUT_FORMATTER_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_RUNTIME_COMPONENTS_ESR_OUTPUT_FORMATTER_H_

#include "1q/electronic_surveillance_radar/session/EsrCycleResult.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "common/runtime/RuntimeCycleExecutor.h"
#include "electronic_surveillance_radar/output/EsrOutputManager.h"

namespace electronic_surveillance_radar {
namespace runtime {
namespace components {

class EsrOutputFormatter {
 public:
  explicit EsrOutputFormatter(output::EsrOutputManager& output_manager);

  session::EsrOutputFrame BuildEmptyFrame(
      const oneq::internal::runtime::RuntimeCycleStamp& stamp) const;

  void LogCycleSummary(const session::EsrCycleInput& cycle_input,
                       const oneq::internal::runtime::RuntimeCycleStamp& stamp,
                       const session::EsrOutputFrame& output_frame) const;

 private:
  output::EsrOutputManager& output_manager_;
};

}  // namespace components
}  // namespace runtime
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_RUNTIME_COMPONENTS_ESR_OUTPUT_FORMATTER_H_
