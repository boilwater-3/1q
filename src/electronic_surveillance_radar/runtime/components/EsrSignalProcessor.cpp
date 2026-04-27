#include "electronic_surveillance_radar/runtime/components/EsrSignalProcessor.h"

namespace electronic_surveillance_radar {
namespace runtime {
namespace components {

EsrSignalProcessor::EsrSignalProcessor(extension::IInterceptPipeline& pipeline)
    : pipeline_(pipeline) {}

extension::InterceptPipelineResult EsrSignalProcessor::Execute(
    const session::EsrCycleInput& cycle_input,
    const environment::IEsrEnvironmentService& environment) const {
  return pipeline_.Execute(cycle_input, environment);
}

}  // namespace components
}  // namespace runtime
}  // namespace electronic_surveillance_radar
