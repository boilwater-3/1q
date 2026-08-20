#include "electronic_surveillance_radar/session/EsrSessionCompositionRoot.h"

#include "electronic_surveillance_radar/runtime/EsrController.h"
#include "electronic_surveillance_radar/environment/IEsrEnvironmentService.h"
#include "electronic_surveillance_radar/session/EsrSessionConfigResolver.h"
#include "electronic_surveillance_radar/environment/EsrEnvironmentService.h"
#include "electronic_surveillance_radar/pipeline/InterceptPipeline.h"

namespace electronic_surveillance_radar {
namespace session {

namespace {

EsrSessionComposition BuildCompositionBase(const config::EsrSessionConfig& config) {
  EsrInternalExecutionConfig exec = MapSessionToInternal(config);
  EsrSessionComposition composition;
  composition.execution_config = std::move(exec);
  return composition;
}

}  // namespace

EsrSessionComposition EsrSessionCompositionRoot::ComposeDefault(const config::EsrSessionConfig& config) {
  EsrSessionComposition composition = BuildCompositionBase(config);
  composition.owned_pipeline.reset(new pipeline::InterceptPipeline(
      composition.execution_config));
  composition.owned_environment_service.reset(
      new environment::EsrEnvironmentService(composition.execution_config.environment));
  composition.owned_controller.reset(new extension::EsrController(
      *composition.owned_pipeline, *composition.owned_environment_service));
  composition.pipeline = composition.owned_pipeline.get();
  composition.environment_service = composition.owned_environment_service.get();
  composition.controller = composition.owned_controller.get();
  return composition;
}

}  // namespace session
}  // namespace electronic_surveillance_radar
