#include "electro_optical_sensor/core/session/EosSessionCompositionRoot.h"

#include <memory>

#include "1q/electro_optical_sensor/core/controller/EosController.h"
#include "1q/electro_optical_sensor/environment/IEosEnvironmentService.h"
#include "electro_optical_sensor/core/pipeline/EosPipeline.h"

namespace electro_optical_sensor {
namespace core {
namespace session {
namespace internal {

namespace {

std::shared_ptr<environment::IEosEnvironmentService> MakeNonOwningEnvironmentServiceHandle(
    environment::IEosEnvironmentService& environment_service) {
  return std::shared_ptr<environment::IEosEnvironmentService>(
      &environment_service, [](environment::IEosEnvironmentService*) {});
}

}  // namespace

EosSessionComposition EosSessionCompositionRoot::ComposeDefault() {
  EosSessionComposition composition;
  composition.owned_pipeline.reset(
      new core::pipeline::EosPipeline(::electro_optical_sensor::pipeline::EosPipelineConfig{}));
  composition.owned_controller.reset(new controller::EosController(*composition.owned_pipeline));
  composition.pipeline = composition.owned_pipeline.get();
  composition.controller = composition.owned_controller.get();
  return composition;
}

EosSessionComposition EosSessionCompositionRoot::ComposeWithPipeline(
    ::electro_optical_sensor::pipeline::IEosPipeline& pipeline) {
  EosSessionComposition composition;
  composition.owned_controller.reset(new controller::EosController(pipeline));
  composition.pipeline = &pipeline;
  composition.controller = composition.owned_controller.get();
  return composition;
}

EosSessionComposition EosSessionCompositionRoot::ComposeWithEnvironmentService(
    environment::IEosEnvironmentService& environment_service) {
  EosSessionComposition composition;
  composition.owned_pipeline.reset(new core::pipeline::EosPipeline(
      ::electro_optical_sensor::pipeline::EosPipelineConfig{},
      MakeNonOwningEnvironmentServiceHandle(environment_service)));
  composition.owned_controller.reset(new controller::EosController(*composition.owned_pipeline));
  composition.pipeline = composition.owned_pipeline.get();
  composition.controller = composition.owned_controller.get();
  return composition;
}

EosSessionComposition EosSessionCompositionRoot::ComposeWithController(
    controller::EosController& controller) {
  EosSessionComposition composition;
  composition.pipeline = &controller.GetPipeline();
  composition.controller = &controller;
  return composition;
}

}  // namespace internal
}  // namespace session
}  // namespace core
}  // namespace electro_optical_sensor
