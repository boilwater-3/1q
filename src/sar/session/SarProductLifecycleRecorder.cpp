#include "1q/sar/session/SarProductLifecycleRecorder.h"

#include "sar/session/SarDiagnosticUtils.h"

namespace sar {
namespace session {

namespace {

bool HasImageProduct(const SarOutputFrame& frame) {
  return frame.has_l1_image || frame.has_l3_bp_image;
}

// 统一问题列表模型（规则 14）：可推导字段 has_error 已删除，改为遍历 issues 判定。
bool HasErrorIssue(const SarIssueList& issues) {
  for (const SarIssue& issue : issues) {
    if (issue.severity == SarIssueSeverity::kError) {
      return true;
    }
  }
  return false;
}

SarProductLifecycleReason InferFailureReason(const SarCycleResult& result) {
  if (result.abort_reason != SarPipelineAbortReason::kNone) {
    return SarProductLifecycleReason::kAbortReason;
  }
  return SarProductLifecycleReason::kError;
}

SarProductLifecycleEvent MakeEvent(const SarCycleResult& result, SarProductLifecycleEventKind kind,
                                   SarProductLifecycleReason reason) {
  SarProductLifecycleEvent event;
  event.cycle_index = result.input_cycle_index;
  event.kind = kind;
  event.reason = reason;
  event.completed_stage = result.output_frame.completed_stage;
  event.abort_reason = AbortReasonToDiagnosticCode(result.abort_reason);
  event.estimated_snr_db = result.output_frame.estimated_snr_db;
  return event;
}

}  // namespace

SarProductLifecycleRecorder::SarProductLifecycleRecorder(SarProductLifecycleRecorderConfig config)
    : config_(config) {}

std::vector<SarProductLifecycleEvent> SarProductLifecycleRecorder::Update(const SarCycleResult& result) {
  std::vector<SarProductLifecycleEvent> events;
  if (result.status == SarCycleStatus::kCompleted) {
    const bool has_product = HasImageProduct(result.output_frame);
    if (HasErrorIssue(result.issues)) {
      events.push_back(
          MakeEvent(result, SarProductLifecycleEventKind::kProcessingFailed, InferFailureReason(result)));
      has_product_ = false;
    } else if (has_product) {
      events.push_back(MakeEvent(result,
                                 has_product_ ? SarProductLifecycleEventKind::kProductSustained
                                              : SarProductLifecycleEventKind::kImageProduced,
                                 SarProductLifecycleReason::kNone));
      has_product_ = true;
    } else {
      if (has_product_) {
        events.push_back(
            MakeEvent(result, SarProductLifecycleEventKind::kProductLost, SarProductLifecycleReason::kNoImageProduct));
      } else if (config_.emit_no_product_events) {
        events.push_back(
            MakeEvent(result, SarProductLifecycleEventKind::kNoProduct, SarProductLifecycleReason::kNoImageProduct));
      }
      has_product_ = false;
    }
    last_events_ = events;
  }
  return events;
}

void SarProductLifecycleRecorder::Reset() {
  has_product_ = false;
  last_events_.clear();
}

const std::vector<SarProductLifecycleEvent>& SarProductLifecycleRecorder::GetLastEvents() const noexcept {
  return last_events_;
}

}  // namespace session
}  // namespace sar
