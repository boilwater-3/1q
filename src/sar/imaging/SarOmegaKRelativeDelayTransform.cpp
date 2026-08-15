#include "sar/imaging/SarOmegaKRelativeDelayTransform.h"

#include <cmath>

namespace sar {
namespace imaging {

namespace {

OmegaKRelativeDelayResult Reject(const OmegaKRelativeDelayRequest& request,
                                 OmegaKRelativeDelayReason reason) {
  OmegaKRelativeDelayResult result;
  result.request_id = request.request_id;
  result.reason = reason;
  return result;
}

bool IsValidSpectrum(const signal::ComplexMatrix& spectrum, std::size_t expected_cols) {
  if (spectrum.rows == 0U || spectrum.cols != expected_cols ||
      spectrum.values.size() != spectrum.rows * spectrum.cols) {
    return false;
  }
  for (const signal::ComplexSample& sample : spectrum.values) {
    if (!std::isfinite(sample.real()) || !std::isfinite(sample.imag())) {
      return false;
    }
  }
  return true;
}

}  // namespace

OmegaKRelativeDelayResult ExecuteOmegaKRelativeDelayTransform(
    const OmegaKRelativeDelayRequest& request) {
  if (request.request_id == 0U) {
    return Reject(request, OmegaKRelativeDelayReason::kInvalidRequestId);
  }
  OmegaKReducedRangeAxisDiagnostics axis;
  if (!DiagnoseOmegaKReducedRangeAxis(request.reduced_range_frequencies_hz, &axis)) {
    return Reject(request, OmegaKRelativeDelayReason::kInvalidFrequencyAxis);
  }
  if (!IsValidSpectrum(request.reduced_spectrum, request.reduced_range_frequencies_hz.size())) {
    return Reject(request, OmegaKRelativeDelayReason::kInvalidSpectrum);
  }
  signal::ComplexMatrix relative_delay_domain;
  if (!signal::FftRows(request.reduced_spectrum, true, &relative_delay_domain)) {
    return Reject(request, OmegaKRelativeDelayReason::kTransformFailure);
  }

  OmegaKRelativeDelayResult result;
  result.request_id = request.request_id;
  result.status = OmegaKRelativeDelayStatus::kSucceeded;
  result.reason = OmegaKRelativeDelayReason::kNone;
  result.axis_diagnostics = axis;
  result.relative_delay_domain = relative_delay_domain;
  return result;
}

}  // namespace imaging
}  // namespace sar
