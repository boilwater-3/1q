#include "1q/sar/session/SarSession.h"

#include <memory>
#include <utility>

#include "1q/sar/session/SarProductLifecycleRecorder.h"
#include "sar/runtime/SarController.h"
#include "sar/session/SarSessionCompositionRoot.h"

namespace sar {
namespace session {

struct SarSession::Impl {
  explicit Impl(SarSessionComposition composition)
      : owned_pipeline(std::move(composition.owned_pipeline)),
        owned_controller(std::move(composition.owned_controller)),
        controller(*composition.controller) {}

  std::unique_ptr<pipeline::SarProcessingPipeline> owned_pipeline;
  std::unique_ptr<extension::SarController> owned_controller;
  extension::SarController& controller;
  SarProductLifecycleRecorder* lifecycle_recorder{nullptr};
};

SarSession::SarSession(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

SarSession::SarSession()
    : impl_(new Impl(SarSessionCompositionRoot::ComposeDefault(config::SarSessionConfig{}))) {}

SarSession::~SarSession() noexcept = default;
SarSession::SarSession(SarSession&&) noexcept = default;
SarSession& SarSession::operator=(SarSession&&) noexcept = default;

SarSession SarSession::Create(const config::SarSessionConfig& config) {
  return SarSession(std::unique_ptr<SarSession::Impl>(
      new SarSession::Impl(SarSessionCompositionRoot::ComposeDefault(config))));
}

SarSession SarSession::CreateWithDiagnostics(const config::SarSessionConfig& config,
                                             SarIssueList* issues) {
  const SarIssueList found = config::ValidateSarSessionConfig(config);
  if (issues != nullptr) {
    *issues = found;
  }
  return Create(config);
}

SarOutputFrame SarSession::Step(const SarCycleInput& input) {
  return StepWithResult(input).output_frame;
}

SarCycleResult SarSession::StepWithResult(const SarCycleInput& input) {
  impl_->controller.RunOnce(input);
  SarCycleResult result = impl_->controller.BuildCycleResult(input);
  if (impl_->lifecycle_recorder != nullptr) {
    impl_->lifecycle_recorder->Update(result);
  }
  return result;
}

void SarSession::AttachProductLifecycleRecorder(SarProductLifecycleRecorder* recorder) noexcept {
  impl_->lifecycle_recorder = recorder;
}

bool SarSession::TryApplyRuntimeConfig(const config::SarRuntimeConfigPatch& patch) {
  // 立即提交类（见 docs/common/contract.md「运行期配置提交策略」）：调用即生效、单向
  // 落定、无 session 层回滚；执行期合法性由 controller 在 Step 内 gate。
  return impl_->controller.TryApplyRuntimeConfig(patch);
}

}  // namespace session
}  // namespace sar
