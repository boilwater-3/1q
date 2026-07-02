#ifndef ONEQ_SRC_SAR_SESSION_SAR_SESSION_COMPOSITION_ROOT_H_
#define ONEQ_SRC_SAR_SESSION_SAR_SESSION_COMPOSITION_ROOT_H_

#include <memory>

#include "1q/sar/config/SarSessionConfig.h"

namespace sar {
namespace pipeline {
class SarProcessingPipeline;
}
namespace extension {
class SarController;
}
namespace session {

struct SarSessionComposition {
  std::unique_ptr<pipeline::SarProcessingPipeline> owned_pipeline;
  std::unique_ptr<extension::SarController> owned_controller;
  extension::SarController* controller{nullptr};
};

class SarSessionCompositionRoot {
 public:
  static SarSessionComposition ComposeDefault(const config::SarSessionConfig& config);
};

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SRC_SAR_SESSION_SAR_SESSION_COMPOSITION_ROOT_H_
