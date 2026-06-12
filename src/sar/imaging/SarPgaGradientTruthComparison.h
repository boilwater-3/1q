/**
 * @file SarPgaGradientTruthComparison.h
 * @brief Wrapped-error comparison for bounded PGA gradient estimates.
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_PGA_GRADIENT_TRUTH_COMPARISON_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_PGA_GRADIENT_TRUTH_COMPARISON_H_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace sar {
namespace imaging {

enum class PgaGradientComparisonStatus { kPassed = 0, kFailed = 1, kRejected = 2 };
enum class PgaGradientComparisonReason {
  kNone = 0,
  kInvalidRequestId = 1,
  kInvalidVectors = 2,
  kInvalidTolerance = 3,
  kInsufficientJointlyValidPairs = 4,
};

struct PgaGradientComparisonRequest {
  std::uint64_t request_id{0U};
  std::vector<double> estimated_wrapped_gradient_rad;
  std::vector<std::uint8_t> estimator_valid_pair_mask;
  std::vector<double> truth_wrapped_gradient_rad;
  std::vector<std::uint8_t> truth_valid_pair_mask;
  std::size_t minimum_jointly_valid_pair_count{0U};
  double maximum_abs_wrapped_error_tolerance_rad{0.0};
  double rms_wrapped_error_tolerance_rad{0.0};
};

struct PgaGradientComparisonResult {
  std::uint64_t request_id{0U};
  PgaGradientComparisonStatus status{PgaGradientComparisonStatus::kRejected};
  PgaGradientComparisonReason reason{PgaGradientComparisonReason::kNone};
  std::size_t jointly_valid_pair_count{0U};
  std::vector<std::uint8_t> jointly_valid_pair_mask;
  double maximum_abs_wrapped_error_rad{0.0};
  double rms_wrapped_error_rad{0.0};
};

PgaGradientComparisonResult ComparePgaGradientTruth(
    const PgaGradientComparisonRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_PGA_GRADIENT_TRUTH_COMPARISON_H_
