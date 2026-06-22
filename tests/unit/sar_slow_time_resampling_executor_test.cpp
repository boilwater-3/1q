#include <gtest/gtest.h>

#include <limits>

#include "sar/imaging/SarSlowTimeResamplingExecutor.h"

namespace sar {
namespace imaging {
namespace {

SlowTimeResamplingRequest BaseRequest() {
  SlowTimeResamplingRequest request;
  request.request_id = 42U;
  request.explicit_times_s = {0.0, 0.2, 0.55, 0.8, 1.0};
  request.expected_interval_s = 0.25;
  request.raw_history.rows = request.explicit_times_s.size();
  request.raw_history.cols = 2U;
  request.raw_history.values.resize(request.raw_history.rows * request.raw_history.cols);
  for (std::size_t row = 0U; row < request.raw_history.rows; ++row) {
    const double time = request.explicit_times_s[row];
    request.raw_history(row, 0U) = signal::ComplexSample(1.0 + time, -time);
    request.raw_history(row, 1U) = signal::ComplexSample(2.0 + 3.0 * time, 1.0 + time);
  }
  return request;
}

TEST(SarSlowTimeResamplingExecutorTest, ExecutesExplicitValidRequestAtomically) {
  const SlowTimeResamplingRequest request = BaseRequest();
  const SlowTimeResamplingExecutionResult result = ExecuteSlowTimeResamplingRequest(request);
  EXPECT_EQ(result.request_id, request.request_id);
  EXPECT_EQ(result.status, SlowTimeResamplingExecutionStatus::kSucceeded);
  EXPECT_EQ(result.reason, SlowTimeResamplingRejectionReason::kNone);
  EXPECT_TRUE(result.gap_diagnostics.resampling_allowed);
  EXPECT_TRUE(result.resampling_diagnostics.valid);
  EXPECT_EQ(result.resampled_raw_history.rows, request.raw_history.rows);
  EXPECT_EQ(result.resampled_raw_history.cols, request.raw_history.cols);
}

TEST(SarSlowTimeResamplingExecutorTest, RejectsMissingGapWithoutPartialOutput) {
  SlowTimeResamplingRequest request = BaseRequest();
  request.explicit_times_s = {0.0, 0.25, 0.75, 1.0, 1.25};
  const SlowTimeResamplingExecutionResult result = ExecuteSlowTimeResamplingRequest(request);
  EXPECT_EQ(result.status, SlowTimeResamplingExecutionStatus::kRejected);
  EXPECT_EQ(result.reason, SlowTimeResamplingRejectionReason::kMissingPulseGap);
  EXPECT_FALSE(result.gap_diagnostics.resampling_allowed);
  EXPECT_TRUE(result.resampled_raw_history.values.empty());
}

TEST(SarSlowTimeResamplingExecutorTest, ReturnsSpecificStructuralRejections) {
  SlowTimeResamplingRequest request = BaseRequest();
  request.request_id = 0U;
  EXPECT_EQ(ExecuteSlowTimeResamplingRequest(request).reason,
            SlowTimeResamplingRejectionReason::kInvalidRequestId);

  request = BaseRequest();
  request.expected_interval_s = 0.0;
  EXPECT_EQ(ExecuteSlowTimeResamplingRequest(request).reason,
            SlowTimeResamplingRejectionReason::kInvalidExpectedInterval);

  request = BaseRequest();
  request.explicit_times_s[2] = request.explicit_times_s[1];
  EXPECT_EQ(ExecuteSlowTimeResamplingRequest(request).reason,
            SlowTimeResamplingRejectionReason::kInvalidTimeAxis);

  request = BaseRequest();
  request.raw_history.values.pop_back();
  EXPECT_EQ(ExecuteSlowTimeResamplingRequest(request).reason,
            SlowTimeResamplingRejectionReason::kInvalidRawHistory);
}

TEST(SarSlowTimeResamplingExecutorTest, IsDeterministicAndDoesNotModifyRequest) {
  const SlowTimeResamplingRequest request = BaseRequest();
  const SlowTimeResamplingExecutionResult first = ExecuteSlowTimeResamplingRequest(request);
  const SlowTimeResamplingExecutionResult second = ExecuteSlowTimeResamplingRequest(request);
  EXPECT_EQ(first.status, second.status);
  EXPECT_EQ(first.reason, second.reason);
  EXPECT_EQ(first.resampled_raw_history.values, second.resampled_raw_history.values);
  EXPECT_EQ(request.request_id, 42U);
  EXPECT_EQ(request.explicit_times_s[2], 0.55);
  EXPECT_EQ(request.raw_history.rows, 5U);
}

}  // namespace
}  // namespace imaging
}  // namespace sar
