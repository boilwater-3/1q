#include "sar/imaging/SarOmegaKReferenceMapping.h"

#include <cmath>

namespace sar {
namespace imaging {

namespace {

OmegaKReferenceMappingResult Reject(const OmegaKReferenceMappingRequest& request,
                                    OmegaKReferenceMappingReason reason) {
  OmegaKReferenceMappingResult result;
  result.request_id = request.request_id;
  result.reason = reason;
  return result;
}

bool IsFiniteVector(const std::vector<double>& values) {
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

OmegaKReferenceMappingResult ExecuteOmegaKReferenceMapping(
    const OmegaKReferenceMappingRequest& request) {
  if (request.request_id == 0U) {
    return Reject(request, OmegaKReferenceMappingReason::kInvalidRequestId);
  }
  if (!std::isfinite(request.propagation_speed_mps) ||
      request.propagation_speed_mps <= 0.0 ||
      !std::isfinite(request.reference_slant_range_m) ||
      request.reference_slant_range_m < 0.0 ||
      !std::isfinite(request.transform_normalization) ||
      request.transform_normalization <= 0.0) {
    return Reject(request, OmegaKReferenceMappingReason::kInvalidPhysicalMetadata);
  }
  if (request.delay_sign == OmegaKDelaySign::kUnspecified ||
      request.reference_phase_sign == OmegaKReferencePhaseSign::kUnspecified) {
    return Reject(request, OmegaKReferenceMappingReason::kInvalidConvention);
  }
  if (request.relative_delays_s.empty() || request.azimuth_coordinates.empty() ||
      !IsFiniteVector(request.relative_delays_s) ||
      !IsFiniteVector(request.azimuth_coordinates)) {
    return Reject(request, OmegaKReferenceMappingReason::kInvalidAxis);
  }
  if (!IsValidMatrix(request.relative_delay_domain, request.azimuth_coordinates.size(),
                     request.relative_delays_s.size())) {
    return Reject(request, OmegaKReferenceMappingReason::kInvalidMatrix);
  }

  const double direction =
      request.delay_sign == OmegaKDelaySign::kPositiveIncreasesRange ? 1.0 : -1.0;
  std::vector<double> absolute_ranges;
  absolute_ranges.reserve(request.relative_delays_s.size());
  for (double delay_s : request.relative_delays_s) {
    const double range_m =
        request.reference_slant_range_m +
        direction * 0.5 * request.propagation_speed_mps * delay_s;
    if (!std::isfinite(range_m) || range_m < 0.0) {
      return Reject(request, OmegaKReferenceMappingReason::kInvalidAbsoluteRange);
    }
    absolute_ranges.push_back(range_m);
  }

  OmegaKReferenceMappingResult result;
  result.request_id = request.request_id;
  result.status = OmegaKReferenceMappingStatus::kSucceeded;
  result.reason = OmegaKReferenceMappingReason::kNone;
  result.absolute_slant_ranges_m = absolute_ranges;
  result.azimuth_coordinates = request.azimuth_coordinates;
  result.referenced_intermediate = request.relative_delay_domain;
  return result;
}

}  // namespace imaging
}  // namespace sar
