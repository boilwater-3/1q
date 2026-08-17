#include "sar/imaging/SarPgaPhaseGradientEstimator.h"

#include <cmath>
#include <complex>

namespace sar {
namespace imaging {

namespace {

PgaPhaseGradientEstimatorResult Reject(const PgaPhaseGradientEstimatorRequest& request,
                                       PgaPhaseGradientEstimatorReason reason) {
  PgaPhaseGradientEstimatorResult result;
  result.request_id = request.request_id;
  result.reason = reason;
  return result;
}

}  // namespace

PgaPhaseGradientEstimatorResult EstimatePgaPhaseGradient(
    const PgaPhaseGradientEstimatorRequest& request) {
  if (request.request_id == 0U) {
    return Reject(request, PgaPhaseGradientEstimatorReason::kInvalidRequestId);
  }
  if (request.aperture_profile.size() < 2U ||
      request.minimum_valid_pair_count == 0U) {
    return Reject(request, PgaPhaseGradientEstimatorReason::kInvalidProfile);
  }
  for (const signal::ComplexSample& sample : request.aperture_profile) {
    if (!std::isfinite(sample.real()) || !std::isfinite(sample.imag())) {
      return Reject(request, PgaPhaseGradientEstimatorReason::kInvalidProfile);
    }
  }
  if (request.support_mask.size() != request.aperture_profile.size()) {
    return Reject(request, PgaPhaseGradientEstimatorReason::kInvalidSupportMask);
  }
  for (std::uint8_t value : request.support_mask) {
    if (value != 0U && value != 1U) {
      return Reject(request, PgaPhaseGradientEstimatorReason::kInvalidSupportMask);
    }
  }

  std::vector<std::uint8_t> valid_mask(request.aperture_profile.size() - 1U, 0U);
  std::vector<double> gradients(request.aperture_profile.size() - 1U, 0.0);
  std::size_t valid_count = 0U;
  for (std::size_t index = 0U; index + 1U < request.aperture_profile.size(); ++index) {
    if (request.support_mask[index] == 0U || request.support_mask[index + 1U] == 0U ||
        std::abs(request.aperture_profile[index]) == 0.0 ||
        std::abs(request.aperture_profile[index + 1U]) == 0.0) {
      continue;
    }
    const signal::ComplexSample product =
        std::conj(request.aperture_profile[index]) * request.aperture_profile[index + 1U];
    gradients[index] = std::arg(product);
    valid_mask[index] = 1U;
    ++valid_count;
  }
  if (valid_count < request.minimum_valid_pair_count) {
    return Reject(request, PgaPhaseGradientEstimatorReason::kInsufficientValidPairs);
  }

  PgaPhaseGradientEstimatorResult result;
  result.request_id = request.request_id;
  result.status = PgaPhaseGradientEstimatorStatus::kSucceeded;
  result.reason = PgaPhaseGradientEstimatorReason::kNone;
  result.valid_pair_count = valid_count;
  result.valid_pair_mask = valid_mask;
  result.wrapped_gradient_rad = gradients;
  return result;
}

}  // namespace imaging
}  // namespace sar
