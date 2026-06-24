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

class ONEQ_API SarProductLifecycleRecorder {
 public:
  explicit SarProductLifecycleRecorder(
      SarProductLifecycleRecorderConfig config = SarProductLifecycleRecorderConfig{})
      : config_(config) {}

  std::vector<SarProductLifecycleEvent> Update(const SarCycleResult& result) {
    std::vector<SarProductLifecycleEvent> events;
    const bool has_product = HasImageProduct(result.output_frame);
    if (result.has_error || !result.executed_this_cycle) {
      events.push_back(MakeEvent(result, SarProductLifecycleEventKind::kProcessingFailed,
                                 InferFailureReason(result)));
      has_product_ = false;
      return events;
    }
    if (has_product) {
      events.push_back(MakeEvent(result,
                                 has_product_ ? SarProductLifecycleEventKind::kProductUpdated
                                              : SarProductLifecycleEventKind::kImageProduced,
                                 SarProductLifecycleReason::kNone));
      has_product_ = true;
      return events;
    }
    if (has_product_) {
      events.push_back(MakeEvent(result, SarProductLifecycleEventKind::kProductLost,
                                 SarProductLifecycleReason::kNoImageProduct));
    } else if (config_.emit_no_product_events) {
      events.push_back(MakeEvent(result, SarProductLifecycleEventKind::kNoProduct,
                                 SarProductLifecycleReason::kNoImageProduct));
    }
    has_product_ = false;
    return events;
  }

  void Reset() { has_product_ = false; }

 private:
  static bool HasImageProduct(const SarOutputFrame& frame) {
    return frame.has_l1_image || frame.has_l3_bp_image;
  }

  static SarProductLifecycleReason InferFailureReason(const SarCycleResult& result) {
    if (!result.executed_this_cycle) {
      return SarProductLifecycleReason::kCycleNotExecuted;
    }
    if (!result.abort_reason.empty()) {
      return SarProductLifecycleReason::kAbortReason;
    }
    return SarProductLifecycleReason::kError;
  }

  static SarProductLifecycleEvent MakeEvent(const SarCycleResult& result,
                                            SarProductLifecycleEventKind kind,
                                            SarProductLifecycleReason reason) {
    SarProductLifecycleEvent event;
    event.cycle_index = result.input_cycle_index;
    event.kind = kind;
    event.reason = reason;
    event.completed_stage = result.output_frame.completed_stage;
    event.abort_reason = result.abort_reason;
    event.estimated_snr_db = result.output_frame.estimated_snr_db;
    return event;
  }

  SarProductLifecycleRecorderConfig config_;
  bool has_product_{false};
};

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SAR_SESSION_SAR_PRODUCT_LIFECYCLE_RECORDER_H_
