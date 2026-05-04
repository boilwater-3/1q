#include "electronic_surveillance_radar/runtime/components/EsrEnvironmentUpdater.h"

namespace electronic_surveillance_radar {
namespace runtime {
namespace components {

EsrEnvironmentUpdater::EsrEnvironmentUpdater(
    environment::IEsrEnvironmentService& environment_service)
    : environment_service_(environment_service) {}

void EsrEnvironmentUpdater::FreezeEnvironment(
    const session::EsrCycleInput& cycle_input,
    const oneq::internal::runtime::RuntimeCycleStamp& stamp) const {
  environment::EsrEnvironmentCycleContext environment_context;
  environment_context.cycle_index = stamp.cycle_index;
  environment_context.dt_sec = cycle_input.dt_sec;
  environment_context.observation = cycle_input.environment;
  environment_service_.BeginCycle(environment_context);
}

}  // namespace components
}  // namespace runtime
}  // namespace electronic_surveillance_radar
