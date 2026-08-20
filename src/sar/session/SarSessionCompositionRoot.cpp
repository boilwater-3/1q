#include "sar/session/SarSessionCompositionRoot.h"

#include "sar/pipeline/SarProcessingPipeline.h"
#include "sar/runtime/SarController.h"

namespace sar {
namespace session {

SarSessionComposition SarSessionCompositionRoot::ComposeDefault(
    const config::SarSessionConfig& config) {
  SarSessionComposition composition;
  composition.owned_pipeline.reset(new pipeline::SarProcessingPipeline(config));
  composition.owned_controller.reset(
      new extension::SarController(*composition.owned_pipeline, config));
  composition.controller = composition.owned_controller.get();
  return composition;
}

}  // namespace session
}  // namespace sar
