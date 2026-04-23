#ifndef ELECTRONIC_SURVEILLANCE_RADAR_RUNTIME_COMPONENTS_ESR_ENVIRONMENT_UPDATER_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_RUNTIME_COMPONENTS_ESR_ENVIRONMENT_UPDATER_H_

#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/environment/IEsrEnvironmentService.h"
#include "common/runtime/RuntimeCycleExecutor.h"

namespace electronic_surveillance_radar {
namespace runtime {
namespace components {

class EsrEnvironmentUpdater {
 public:
  explicit EsrEnvironmentUpdater(environment::IEsrEnvironmentService& environment_service);

  void FreezeEnvironment(const session::EsrCycleInput& cycle_input,
                         const oneq::internal::runtime::RuntimeCycleStamp& stamp) const;

 private:
  environment::IEsrEnvironmentService& environment_service_;
};

}  // namespace components
}  // namespace runtime
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_RUNTIME_COMPONENTS_ESR_ENVIRONMENT_UPDATER_H_
