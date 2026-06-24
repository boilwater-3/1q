#include "sar/imaging/SarOmegaKReferencePhaseCompensation.h"

#include <cmath>
#include <complex>

namespace sar {
namespace imaging {

namespace {

OmegaKPhaseCompensationResult Reject(const OmegaKPhaseCompensationRequest& request,
                                     OmegaKPhaseCompensationReason reason) {
  OmegaKPhaseCompensationResult result;
  result.request_id = request.request_id;
  result.reason = reason;
  return result;
}

bool IsFiniteVector(const std::vector<double>& values) {
  if (values.empty()) {
    return false;
  }
  for (double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

bool IsValidMatrix(const signal::ComplexMatrix& matrix, std::size_t expected_rows,
                   std::size_t expected_cols) {
  if (matrix.rows != expected_rows || matrix.cols != expected_cols ||
      matrix.values.size() != matrix.rows * matrix.cols) {
    return false;
  }
  for (const signal::ComplexSample& sample : matrix.values) {
    if (!std::isfinite(sample.real()) || !std::isfinite(sample.imag())) {
      return false;
    }
  }
  return true;
}

}  // namespace

OmegaKPhaseCompensationResult ExecuteOmegaKReferencePhaseCompensation(
    const OmegaKPhaseCompensationRequest& request) {
  if (request.request_id == 0U) {
    return Reject(request, OmegaKPhaseCompensationReason::kInvalidRequestId);
  }
  if (request.sign == OmegaKPhaseApplicationSign::kUnspecified) {
    return Reject(request, OmegaKPhaseCompensationReason::kInvalidSign);
  }
  if (!IsFiniteVector(request.absolute_slant_ranges_m) ||
      !IsFiniteVector(request.azimuth_coordinates)) {
    return Reject(request, OmegaKPhaseCompensationReason::kInvalidAxis);
  }
  if (request.range_phase_radians.size() != request.absolute_slant_ranges_m.size() ||
      !IsFiniteVector(request.range_phase_radians)) {
    return Reject(request, OmegaKPhaseCompensationReason::kInvalidPhase);
  }
  if (!IsValidMatrix(request.referenced_intermediate, request.azimuth_coordinates.size(),
                     request.absolute_slant_ranges_m.size())) {
    return Reject(request, OmegaKPhaseCompensationReason::kInvalidMatrix);
  }

  const double direction =
      request.sign == OmegaKPhaseApplicationSign::kPositive ? 1.0 : -1.0;
  signal::ComplexMatrix compensated = request.referenced_intermediate;
  for (std::size_t row = 0U; row < compensated.rows; ++row) {
    for (std::size_t col = 0U; col < compensated.cols; ++col) {
      const double phase = direction * request.range_phase_radians[col];
      compensated.values[row * compensated.cols + col] *=
          signal::ComplexSample(std::cos(phase), std::sin(phase));
    }
  }

  OmegaKPhaseCompensationResult result;
  result.request_id = request.request_id;
  result.status = OmegaKPhaseCompensationStatus::kSucceeded;
  result.reason = OmegaKPhaseCompensationReason::kNone;
  result.absolute_slant_ranges_m = request.absolute_slant_ranges_m;
  result.azimuth_coordinates = request.azimuth_coordinates;
  result.compensated_intermediate = compensated;
  return result;
}

}  // namespace imaging
}  // namespace sar
