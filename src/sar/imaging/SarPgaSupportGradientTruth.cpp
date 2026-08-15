#include "sar/imaging/SarPgaSupportGradientTruth.h"

#include <cmath>

#include "sar/signal/SarAngleWrap.h"

namespace sar {
namespace imaging {

namespace {

PgaSupportGradientTruthResult Reject(const PgaSupportGradientTruthRequest& request,
                                     PgaSupportGradientTruthReason reason) {
  PgaSupportGradientTruthResult result;
  result.request_id = request.request_id;
  result.reason = reason;
  return result;
}

}  // namespace

PgaSupportGradientTruthResult ExecutePgaSupportGradientTruth(
    const PgaSupportGradientTruthRequest& request) {
  if (request.request_id == 0U) {
    return Reject(request, PgaSupportGradientTruthReason::kInvalidRequestId);
  }
  if (!std::isfinite(request.peak_relative_threshold) ||
      request.peak_relative_threshold <= 0.0 ||
      request.peak_relative_threshold > 1.0 ||
      request.minimum_supported_samples == 0U) {
    return Reject(request, PgaSupportGradientTruthReason::kInvalidThreshold);
  }
  if (request.aperture_profile.size() < 2U) {
    return Reject(request, PgaSupportGradientTruthReason::kInvalidProfile);
  }

  std::size_t peak_index = 0U;
  double peak_magnitude = -1.0;
  for (std::size_t index = 0U; index < request.aperture_profile.size(); ++index) {
    const signal::ComplexSample sample = request.aperture_profile[index];
    if (!std::isfinite(sample.real()) || !std::isfinite(sample.imag())) {
      return Reject(request, PgaSupportGradientTruthReason::kInvalidProfile);
    }
    const double magnitude = std::abs(sample);
    if (magnitude > peak_magnitude) {
      peak_magnitude = magnitude;
      peak_index = index;
    }
  }
  if (peak_magnitude <= 0.0) {
    return Reject(request, PgaSupportGradientTruthReason::kZeroEnergy);
  }

  std::vector<std::uint8_t> support_mask(request.aperture_profile.size(), 0U);
  std::size_t supported_count = 0U;
  const double threshold = peak_magnitude * request.peak_relative_threshold;
  for (std::size_t index = 0U; index < request.aperture_profile.size(); ++index) {
    if (std::abs(request.aperture_profile[index]) >= threshold) {
      support_mask[index] = 1U;
      ++supported_count;
    }
  }
  if (supported_count < request.minimum_supported_samples) {
    return Reject(request, PgaSupportGradientTruthReason::kInsufficientSupport);
  }
  if (request.injected_phase_rad.size() != request.aperture_profile.size()) {
    return Reject(request, PgaSupportGradientTruthReason::kInvalidPhaseTruth);
  }
  for (double phase : request.injected_phase_rad) {
    if (!std::isfinite(phase)) {
      return Reject(request, PgaSupportGradientTruthReason::kInvalidPhaseTruth);
    }
  }

  std::vector<double> gradient;
  gradient.reserve(request.injected_phase_rad.size() - 1U);
  for (std::size_t index = 0U; index + 1U < request.injected_phase_rad.size(); ++index) {
    gradient.push_back(signal::WrapPhase(request.injected_phase_rad[index + 1U] -
                                 request.injected_phase_rad[index]));
  }

  PgaSupportGradientTruthResult result;
  result.request_id = request.request_id;
  result.status = PgaSupportGradientTruthStatus::kSucceeded;
  result.reason = PgaSupportGradientTruthReason::kNone;
  result.peak_index = peak_index;
  result.supported_sample_count = supported_count;
  result.support_mask = support_mask;
  result.wrapped_forward_gradient_rad = gradient;
  return result;
}

}  // namespace imaging
}  // namespace sar
