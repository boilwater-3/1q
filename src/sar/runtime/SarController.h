#ifndef ONEQ_SRC_SAR_RUNTIME_SAR_CONTROLLER_H_
#define ONEQ_SRC_SAR_RUNTIME_SAR_CONTROLLER_H_

#include <cstdint>
#include <memory>

#include "1q/sar/config/SarRuntimeConfigPatch.h"
#include "1q/sar/config/SarSessionConfig.h"
#include "1q/sar/session/SarCycleInput.h"
#include "1q/sar/session/SarCycleResult.h"
#include "sar/pipeline/SarProcessingPipeline.h"

namespace sar {
namespace pipeline {
class SarProcessingPipeline;
}
namespace extension {

struct SarControllerRuntimeState {
  const void* owner_identity{nullptr};
  std::uint32_t schema_version{0U};
  config::SarSessionConfig runtime_config{};
  session::SarOutputFrame previous_output{};
  bool has_previous_output{false};
  session::SarCycleResult latest_result{};
  pipeline::SarProcessingPipelineRuntimeState pipeline_state{};
};

class SarController {
 public:
  SarController(pipeline::SarProcessingPipeline& pipeline,
                const config::SarSessionConfig& initial_config);
  ~SarController();

  SarController(const SarController&) = delete;
  SarController& operator=(const SarController&) = delete;

  void RunOnce(const session::SarCycleInput& input);
  session::SarCycleResult BuildCycleResult(const session::SarCycleInput& input) const;

  bool TryApplyRuntimeConfig(const config::SarRuntimeConfigPatch& patch);
  SarControllerRuntimeState CaptureRuntimeState() const;
  bool RestoreRuntimeState(const SarControllerRuntimeState& state);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace extension
}  // namespace sar

#endif  // ONEQ_SRC_SAR_RUNTIME_SAR_CONTROLLER_H_
