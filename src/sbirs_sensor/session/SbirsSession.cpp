#include "1q/sbirs_sensor/session/SbirsSession.h"

#include "sbirs_sensor/runtime/SbirsPipelineConfigMapper.h"
#include "sbirs_sensor/runtime/SbirsRuntimeConfigResolver.h"
#include "sbirs_sensor/session/SbirsSessionCompositionRoot.h"

namespace sbirs_sensor {
namespace session {

struct SbirsSession::Impl {
  config::SbirsSessionConfig config{};
  std::unique_ptr<runtime::SbirsController> controller{};
};

SbirsSession::SbirsSession() : impl_(new Impl) {
  impl_->controller = CreateSbirsController(impl_->config);
}

SbirsSession::SbirsSession(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

SbirsSession::~SbirsSession() noexcept = default;
SbirsSession::SbirsSession(SbirsSession&&) noexcept = default;
SbirsSession& SbirsSession::operator=(SbirsSession&&) noexcept = default;

SbirsOutputFrame SbirsSession::Step(const SbirsCycleInput& input) {
  return StepWithResult(input).output_frame;
}

SbirsCycleResult SbirsSession::StepWithResult(const SbirsCycleInput& input) {
  return impl_->controller->RunOnce(input);
}

void SbirsSession::ApplyRuntimeConfig(const config::SbirsRuntimeConfigPatch& patch) {
  TryApplyRuntimeConfig(patch);
}

bool SbirsSession::TryApplyRuntimeConfig(const config::SbirsRuntimeConfigPatch& patch) {
  const runtime::SbirsRuntimeConfigResolution resolution =
      runtime::ResolveSbirsRuntimeConfigPatch(impl_->config, patch);
  if (!resolution.is_valid || !resolution.has_requested_update) {
    return false;
  }
  impl_->config = resolution.resolved_config;
  impl_->controller->ApplyConfig(runtime::MapSessionToInternal(impl_->config), resolution.impact);
  return true;
}

SbirsSession SbirsSession::Create(const config::SbirsSessionConfig& config) {
  std::unique_ptr<Impl> impl(new Impl);
  impl->config = config;
  impl->controller = CreateSbirsController(config);
  return SbirsSession(std::move(impl));
}

SbirsSession SbirsSession::CreateWithValidation(const config::SbirsSessionConfig& config,
                                                config::ValidationIssueList* issues) {
  if (issues != nullptr) {
    *issues = config::ValidateSbirsSessionConfig(config);
  }
  return Create(config);
}

}  // namespace session
}  // namespace sbirs_sensor
