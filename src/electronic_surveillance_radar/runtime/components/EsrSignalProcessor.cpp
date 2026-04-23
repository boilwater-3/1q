#include "electronic_surveillance_radar/runtime/components/EsrSignalProcessor.h"

namespace electronic_surveillance_radar {
namespace runtime {
namespace components {

EsrSignalProcessor::EsrSignalProcessor(extension::IInterceptPipeline& pipeline)
    : pipeline_(pipeline) {}

extension::InterceptCycleResult EsrSignalProcessor::Execute(
    const session::EsrCycleInput& cycle_input,
    const environment::IEsrEnvironmentService& environment) const {
  return pipeline_.RunCycle(cycle_input, environment);
}

}  // namespace components
}  // namespace runtime
}  // namespace electronic_surveillance_radar
