#include "sar/imaging/SarOmegaKAzimuthInverseTransform.h"

#include <cmath>

namespace sar {
namespace imaging {

namespace {

OmegaKAzimuthInverseResult Reject(const OmegaKAzimuthInverseRequest& request,
                                  OmegaKAzimuthInverseReason reason) {
  OmegaKAzimuthInverseResult result;
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

OmegaKAzimuthInverseResult ExecuteOmegaKAzimuthInverseTransform(
    const OmegaKAzimuthInverseRequest& request) {
  if (request.request_id == 0U) {
    return Reject(request, OmegaKAzimuthInverseReason::kInvalidRequestId);
  }
  if (!IsFiniteVector(request.absolute_slant_ranges_m) ||
      !IsFiniteVector(request.output_azimuth_coordinates)) {
    return Reject(request, OmegaKAzimuthInverseReason::kInvalidAxis);
  }
  if (!std::isfinite(request.additional_normalization) ||
      request.additional_normalization <= 0.0) {
    return Reject(request, OmegaKAzimuthInverseReason::kInvalidNormalization);
  }
  if (!IsValidMatrix(request.compensated_intermediate,
                     request.output_azimuth_coordinates.size(),
                     request.absolute_slant_ranges_m.size())) {
    return Reject(request, OmegaKAzimuthInverseReason::kInvalidMatrix);
  }

  signal::ComplexMatrix candidate;
  if (!signal::FftCols(request.compensated_intermediate, true, &candidate)) {
    return Reject(request, OmegaKAzimuthInverseReason::kTransformFailure);
  }
  for (signal::ComplexSample& sample : candidate.values) {
    sample *= request.additional_normalization;
  }

  OmegaKAzimuthInverseResult result;
  result.request_id = request.request_id;
  result.status = OmegaKAzimuthInverseStatus::kSucceeded;
  result.reason = OmegaKAzimuthInverseReason::kNone;
  result.absolute_slant_ranges_m = request.absolute_slant_ranges_m;
  result.output_azimuth_coordinates = request.output_azimuth_coordinates;
  result.additional_normalization = request.additional_normalization;
  result.numerical_image_candidate = candidate;
  return result;
}

}  // namespace imaging
}  // namespace sar
