#include "electro_optical_sensor/core/session/EosSessionCompositionRoot.h"

#include <memory>

#include "1q/electro_optical_sensor/extension/EosController.h"
#include "1q/electro_optical_sensor/extension/IEosEnvironmentService.h"
#include "electro_optical_sensor/core/pipeline/EosPipeline.h"

namespace electro_optical_sensor {
namespace session {
namespace internal {

namespace {

std::shared_ptr<extension::IEosEnvironmentService> MakeNonOwningEnvironmentServiceHandle(
    extension::IEosEnvironmentService& environment_service) {
  return std::shared_ptr<extension::IEosEnvironmentService>(
      &environment_service, [](extension::IEosEnvironmentService*) {});
}

}  // namespace

EosSessionComposition EosSessionCompositionRoot::ComposeDefault() {
  EosSessionComposition composition;
  composition.owned_pipeline.reset(
      new core::pipeline::EosPipeline(::electro_optical_sensor::extension::EosPipelineConfig{}));
  composition.owned_controller.reset(new extension::EosController(*composition.owned_pipeline));
  composition.pipeline = composition.owned_pipeline.get();
  composition.controller = composition.owned_controller.get();
  return composition;
}

EosSessionComposition EosSessionCompositionRoot::ComposeWithPipeline(
    ::electro_optical_sensor::extension::IEosPipeline& pipeline) {
  EosSessionComposition composition;
  composition.owned_controller.reset(new extension::EosController(pipeline));
  composition.pipeline = &pipeline;
  composition.controller = composition.owned_controller.get();
  return composition;
}

EosSessionComposition EosSessionCompositionRoot::ComposeWithEnvironmentService(
    extension::IEosEnvironmentService& environment_service) {
  EosSessionComposition composition;
  composition.owned_pipeline.reset(new core::pipeline::EosPipeline(
      ::electro_optical_sensor::extension::EosPipelineConfig{},
      MakeNonOwningEnvironmentServiceHandle(environment_service)));
  composition.owned_controller.reset(new extension::EosController(*composition.owned_pipeline));
  composition.pipeline = composition.owned_pipeline.get();
  composition.controller = composition.owned_controller.get();
  return composition;
}

EosSessionComposition EosSessionCompositionRoot::ComposeWithController(
    extension::EosController& controller) {
  EosSessionComposition composition;
  composition.pipeline = &controller.GetPipeline();
  composition.controller = &controller;
  return composition;
}

}  // namespace internal
}  // namespace session
}  // namespace electro_optical_sensor
