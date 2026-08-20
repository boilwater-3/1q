#include "1q/sbirs_sensor/session/SbirsSession.h"

#include "1q/sbirs_sensor/session/SbirsDetectionLifecycleRecorder.h"
#include "1q/sbirs_sensor/session/SbirsExclusionCauseRecorder.h"
#include "sbirs_sensor/runtime/SbirsPipelineConfigMapper.h"
#include "sbirs_sensor/runtime/SbirsRuntimeConfigResolver.h"
#include "sbirs_sensor/session/SbirsSessionCompositionRoot.h"

namespace sbirs_sensor {
namespace session {

struct SbirsSession::Impl {
  config::SbirsSessionConfig config{};
  std::unique_ptr<runtime::SbirsController> controller{};
  SbirsDetectionLifecycleRecorder* lifecycle_recorder{nullptr};
  SbirsExclusionCauseRecorder* exclusion_cause_recorder{nullptr};
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
  SbirsCycleResult result = impl_->controller->RunOnce(input);
  if (impl_->lifecycle_recorder != nullptr) {
    impl_->lifecycle_recorder->Update(input, result);
  }
  if (impl_->exclusion_cause_recorder != nullptr) {
    impl_->exclusion_cause_recorder->Update(input, result);
  }
  return result;
}

void SbirsSession::AttachDetectionLifecycleRecorder(SbirsDetectionLifecycleRecorder* recorder) noexcept {
  impl_->lifecycle_recorder = recorder;
}

void SbirsSession::AttachExclusionCauseRecorder(SbirsExclusionCauseRecorder* recorder) noexcept {
  impl_->exclusion_cause_recorder = recorder;
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

SbirsSession SbirsSession::CreateWithDiagnostics(const config::SbirsSessionConfig& config,
                                                 SbirsIssueList* issues) {
  if (issues != nullptr) {
    *issues = config::ValidateSbirsSessionConfig(config);
  }
  return Create(config);
}

}  // namespace session
}  // namespace sbirs_sensor
