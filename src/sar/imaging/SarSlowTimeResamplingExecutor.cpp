#include "sar/imaging/SarSlowTimeResamplingExecutor.h"

#include <cmath>

namespace sar {
namespace imaging {

namespace {

SlowTimeResamplingExecutionResult Reject(const SlowTimeResamplingRequest& request,
                                         SlowTimeResamplingRejectionReason reason) {
  SlowTimeResamplingExecutionResult result;
  result.request_id = request.request_id;
  result.reason = reason;
  return result;
}

bool IsValidTimeAxis(const std::vector<double>& times) {
  if (times.size() < 2U) {
    return false;
  }
  for (std::size_t index = 0U; index < times.size(); ++index) {
    if (!std::isfinite(times[index]) || (index > 0U && times[index] <= times[index - 1U])) {
      return false;
    }
  }
  return true;
}

bool IsValidRawHistory(const signal::ComplexMatrix& raw, std::size_t expected_rows) {
  return raw.rows == expected_rows && raw.rows >= 2U && raw.cols > 0U &&
         raw.values.size() == raw.rows * raw.cols;
}

}  // namespace

SlowTimeResamplingExecutionResult ExecuteSlowTimeResamplingRequest(
    const SlowTimeResamplingRequest& request) {
  if (request.request_id == 0U) {
    return Reject(request, SlowTimeResamplingRejectionReason::kInvalidRequestId);
  }
  if (!std::isfinite(request.expected_interval_s) || request.expected_interval_s <= 0.0) {
    return Reject(request, SlowTimeResamplingRejectionReason::kInvalidExpectedInterval);
  }
  if (!IsValidTimeAxis(request.explicit_times_s)) {
    return Reject(request, SlowTimeResamplingRejectionReason::kInvalidTimeAxis);
  }
  if (!IsValidRawHistory(request.raw_history, request.explicit_times_s.size())) {
    return Reject(request, SlowTimeResamplingRejectionReason::kInvalidRawHistory);
  }

  SlowTimeResamplingExecutionResult result;
  result.request_id = request.request_id;
  if (!DiagnoseSlowTimeGaps(request.explicit_times_s, request.expected_interval_s,
                            &result.gap_diagnostics)) {
    return Reject(request, SlowTimeResamplingRejectionReason::kInvalidTimeAxis);
  }
  if (!result.gap_diagnostics.resampling_allowed) {
    result.reason = SlowTimeResamplingRejectionReason::kMissingPulseGap;
    return result;
  }

  signal::ComplexMatrix output;
  SlowTimeResamplingDiagnostics diagnostics;
  if (!ResampleRawHistorySlowTimeLinear(request.explicit_times_s, request.raw_history, &output,
                                        &diagnostics)) {
    return Reject(request, SlowTimeResamplingRejectionReason::kResamplingFailure);
  }
  result.status = SlowTimeResamplingExecutionStatus::kSucceeded;
  result.reason = SlowTimeResamplingRejectionReason::kNone;
  result.resampling_diagnostics = diagnostics;
  result.resampled_raw_history = output;
  return result;
}

}  // namespace imaging
}  // namespace sar
