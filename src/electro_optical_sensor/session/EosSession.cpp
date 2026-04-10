#include "1q/electro_optical_sensor/session/EosSession.h"

#include <utility>

#include "1q/electro_optical_sensor/extension/EosController.h"
#include "1q/electro_optical_sensor/extension/IEosEnvironmentService.h"
#include "1q/electro_optical_sensor/extension/IEosPipeline.h"
#include "electro_optical_sensor/session/EosSessionCompositionRoot.h"
#include "electro_optical_sensor/runtime/EosCycleOrchestrator.h"

namespace electro_optical_sensor {
namespace session {

struct EosSession::Impl {
  Impl(internal::EosSessionComposition composition, const EosSessionConfig& config)
      : owned_pipeline(std::move(composition.owned_pipeline)),
        owned_controller(std::move(composition.owned_controller)),
        cycle_orchestrator(config, *owned_pipeline, *owned_controller) {
  }

  std::unique_ptr<::electro_optical_sensor::extension::IEosPipeline> owned_pipeline;
  std::unique_ptr<extension::EosController> owned_controller;
  internal::EosCycleOrchestrator cycle_orchestrator;
};

EosSession::EosSession(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

EosSession::~EosSession() noexcept = default;
EosSession::EosSession(EosSession&&) noexcept = default;
EosSession& EosSession::operator=(EosSession&&) noexcept = default;

EosSession EosSessionFactory::Create(const EosSessionConfig& config) {
  return EosSession(
      std::unique_ptr<EosSession::Impl>(new EosSession::Impl(
          internal::EosSessionCompositionRoot::ComposeDefault(), config)));
}

EosSession EosSessionFactory::CreateWithPipeline(
    const EosSessionConfig& config, ::electro_optical_sensor::extension::IEosPipeline& pipeline) {
  return EosSession(
      std::unique_ptr<EosSession::Impl>(new EosSession::Impl(
          internal::EosSessionCompositionRoot::ComposeWithPipeline(pipeline), config)));
}

EosSession EosSessionFactory::CreateWithEnvironmentService(
    const EosSessionConfig& config,
    extension::IEosEnvironmentService& environment_service) {
  return EosSession(std::unique_ptr<EosSession::Impl>(new EosSession::Impl(
      internal::EosSessionCompositionRoot::ComposeWithEnvironmentService(environment_service),
      config)));
}

EosSession EosSessionFactory::CreateWithController(
    const EosSessionConfig& config, extension::EosController& controller) {
  return EosSession(
      std::unique_ptr<EosSession::Impl>(new EosSession::Impl(
          internal::EosSessionCompositionRoot::ComposeWithController(controller), config)));
}

common::EosOutputFrame EosSession::Step(const EosCycleInput& input) {
  return StepWithResult(input).output_frame;
}

EosCycleResult EosSession::StepWithResult(const EosCycleInput& input) {
  return impl_->cycle_orchestrator.Step(input);
}

void EosSession::ApplyRuntimeConfig(const EosRuntimeConfigPatch& patch) {
  impl_->cycle_orchestrator.ApplyRuntimeConfig(patch);
}

}  // namespace session
}  // namespace electro_optical_sensor
