#include "sar/imaging/SarPgaGradientTruthComparison.h"

#include <algorithm>
#include <cmath>

#include "sar/signal/SarAngleWrap.h"

namespace sar {
namespace imaging {

namespace {

PgaGradientComparisonResult Reject(const PgaGradientComparisonRequest& request,
                                   PgaGradientComparisonReason reason) {
  PgaGradientComparisonResult result;
  result.request_id = request.request_id;
  result.reason = reason;
  return result;
}

bool IsBinaryMask(const std::vector<std::uint8_t>& mask) {
  for (std::uint8_t value : mask) {
    if (value != 0U && value != 1U) {
      return false;
    }
  }
  return true;
}

}  // namespace

PgaGradientComparisonResult ComparePgaGradientTruth(
    const PgaGradientComparisonRequest& request) {
  if (request.request_id == 0U) {
    return Reject(request, PgaGradientComparisonReason::kInvalidRequestId);
  }
  const std::size_t count = request.estimated_wrapped_gradient_rad.size();
  if (count == 0U || request.estimator_valid_pair_mask.size() != count ||
      request.truth_wrapped_gradient_rad.size() != count ||
      request.truth_valid_pair_mask.size() != count ||
      !IsBinaryMask(request.estimator_valid_pair_mask) ||
      !IsBinaryMask(request.truth_valid_pair_mask) ||
      request.minimum_jointly_valid_pair_count == 0U) {
    return Reject(request, PgaGradientComparisonReason::kInvalidVectors);
  }
  if (!std::isfinite(request.maximum_abs_wrapped_error_tolerance_rad) ||
      request.maximum_abs_wrapped_error_tolerance_rad < 0.0 ||
      !std::isfinite(request.rms_wrapped_error_tolerance_rad) ||
      request.rms_wrapped_error_tolerance_rad < 0.0) {
    return Reject(request, PgaGradientComparisonReason::kInvalidTolerance);
  }
  for (std::size_t index = 0U; index < count; ++index) {
    if (!std::isfinite(request.estimated_wrapped_gradient_rad[index]) ||
        !std::isfinite(request.truth_wrapped_gradient_rad[index])) {
      return Reject(request, PgaGradientComparisonReason::kInvalidVectors);
    }
  }

  std::vector<std::uint8_t> joint_mask(count, 0U);
  std::size_t joint_count = 0U;
  double maximum_abs_error = 0.0;
  double sum_squared_error = 0.0;
  for (std::size_t index = 0U; index < count; ++index) {
    if (request.estimator_valid_pair_mask[index] == 0U ||
        request.truth_valid_pair_mask[index] == 0U) {
      continue;
    }
    joint_mask[index] = 1U;
    ++joint_count;
    const double error = signal::WrapPhase(request.estimated_wrapped_gradient_rad[index] -
                                   request.truth_wrapped_gradient_rad[index]);
    maximum_abs_error = std::max(maximum_abs_error, std::abs(error));
    sum_squared_error += error * error;
  }
  if (joint_count < request.minimum_jointly_valid_pair_count) {
    return Reject(request, PgaGradientComparisonReason::kInsufficientJointlyValidPairs);
  }

  PgaGradientComparisonResult result;
  result.request_id = request.request_id;
  result.reason = PgaGradientComparisonReason::kNone;
  result.jointly_valid_pair_count = joint_count;
  result.jointly_valid_pair_mask = joint_mask;
  result.maximum_abs_wrapped_error_rad = maximum_abs_error;
  result.rms_wrapped_error_rad =
      std::sqrt(sum_squared_error / static_cast<double>(joint_count));
  result.status =
      maximum_abs_error <= request.maximum_abs_wrapped_error_tolerance_rad &&
              result.rms_wrapped_error_rad <= request.rms_wrapped_error_tolerance_rad
          ? PgaGradientComparisonStatus::kPassed
          : PgaGradientComparisonStatus::kFailed;
  return result;
}

}  // namespace imaging
}  // namespace sar
