#ifndef ELECTRONIC_SURVEILLANCE_RADAR_RUNTIME_COMPONENTS_ESR_SIGNAL_PROCESSOR_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_RUNTIME_COMPONENTS_ESR_SIGNAL_PROCESSOR_H_

#include "1q/electronic_surveillance_radar/extension/IInterceptPipeline.h"

namespace electronic_surveillance_radar {
namespace runtime {
namespace components {

class EsrSignalProcessor {
 public:
  explicit EsrSignalProcessor(extension::IInterceptPipeline& pipeline);

  extension::InterceptCycleResult Execute(const session::EsrCycleInput& cycle_input,
                                          const environment::IEsrEnvironmentService& environment)
      const;

 private:
  extension::IInterceptPipeline& pipeline_;
};

}  // namespace components
}  // namespace runtime
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_RUNTIME_COMPONENTS_ESR_SIGNAL_PROCESSOR_H_
