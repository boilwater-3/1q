/**
 * @file SarSlowTimeResamplingExecutor.h
 * @brief SAR 内部慢时间重采样显式请求执行器。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_SLOW_TIME_RESAMPLING_EXECUTOR_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_SLOW_TIME_RESAMPLING_EXECUTOR_H_

#include <cstdint>
#include <vector>

#include "sar/imaging/SarSlowTimeResampling.h"

namespace sar {
namespace imaging {

enum class SlowTimeResamplingExecutionStatus {
  kSucceeded = 0,
  kRejected = 1,
};

enum class SlowTimeResamplingRejectionReason {
  kNone = 0,
  kInvalidRequestId = 1,
  kInvalidExpectedInterval = 2,
  kInvalidTimeAxis = 3,
  kInvalidRawHistory = 4,
  kMissingPulseGap = 5,
  kResamplingFailure = 6,
};

struct SlowTimeResamplingRequest {
  std::uint64_t request_id{0U};
  std::vector<double> explicit_times_s;
  double expected_interval_s{0.0};
  signal::ComplexMatrix raw_history;
};

struct SlowTimeResamplingExecutionResult {
  std::uint64_t request_id{0U};
  SlowTimeResamplingExecutionStatus status{SlowTimeResamplingExecutionStatus::kRejected};
  SlowTimeResamplingRejectionReason reason{SlowTimeResamplingRejectionReason::kNone};
  SlowTimeGapDiagnostics gap_diagnostics;
  SlowTimeResamplingDiagnostics resampling_diagnostics;
  signal::ComplexMatrix resampled_raw_history;
};

SlowTimeResamplingExecutionResult ExecuteSlowTimeResamplingRequest(
    const SlowTimeResamplingRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_SLOW_TIME_RESAMPLING_EXECUTOR_H_
