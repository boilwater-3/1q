/**
 * @file SarProductLifecycleRecorder.h
 * @brief 定义 SAR 产品生命周期记录器。
 */

#ifndef ONEQ_SAR_SESSION_SAR_PRODUCT_LIFECYCLE_RECORDER_H_
#define ONEQ_SAR_SESSION_SAR_PRODUCT_LIFECYCLE_RECORDER_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/sar/session/SarCycleResult.h"

namespace sar {
namespace session {

enum class SarProductLifecycleEventKind {
  kImageProduced = 0,
  kProductUpdated = 1,
  kProductLost = 2,
  kProcessingFailed = 3,
  kNoProduct = 4
};

enum class SarProductLifecycleReason {
  kNone = 0,
  kNoImageProduct = 1,
  kAbortReason = 2,
  kError = 3,
  kCycleNotExecuted = 4
};

struct ONEQ_API SarProductLifecycleEvent {
  std::uint32_t cycle_index{0U};
  SarProductLifecycleEventKind kind{SarProductLifecycleEventKind::kProductUpdated};
  SarProductLifecycleReason reason{SarProductLifecycleReason::kNone};
  SarProcessingStage completed_stage{SarProcessingStage::kNone};
  std::string abort_reason{};
  double estimated_snr_db{0.0};
};

struct ONEQ_API SarProductLifecycleRecorderConfig {
  bool emit_no_product_events{false};
};

/**
 * @brief 按成像产品生命周期记录图像生成/更新/丢失/处理失败/无产品事件。
 *
 * 私有状态与实现见 .cpp，避免在 public header 暴露判定逻辑。
 */
class ONEQ_API SarProductLifecycleRecorder {
 public:
  explicit SarProductLifecycleRecorder(
      SarProductLifecycleRecorderConfig config = SarProductLifecycleRecorderConfig{});

  std::vector<SarProductLifecycleEvent> Update(const SarCycleResult& result);

  void Reset();

 private:
  SarProductLifecycleRecorderConfig config_;
  bool has_product_{false};
};

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SAR_SESSION_SAR_PRODUCT_LIFECYCLE_RECORDER_H_
