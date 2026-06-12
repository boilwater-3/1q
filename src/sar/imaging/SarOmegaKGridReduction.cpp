#include "sar/imaging/SarOmegaKGridReduction.h"

#include <cmath>

namespace sar {
namespace imaging {

namespace {

OmegaKGridReductionResult Reject(const OmegaKGridReductionRequest& request,
                                 OmegaKGridReductionReason reason) {
  OmegaKGridReductionResult result;
  result.request_id = request.request_id;
  result.reason = reason;
  return result;
}

}  // namespace

OmegaKGridReductionResult ExecuteOmegaKExplicitGridReduction(
    const OmegaKGridReductionRequest& request) {
  if (request.request_id == 0U) {
    return Reject(request, OmegaKGridReductionReason::kInvalidRequestId);
  }
  const std::size_t rows = request.geometry.azimuth_frequency_bin_count;
  const std::size_t cols = request.geometry.range_frequency_bin_count;
  if (rows == 0U || cols < 2U || request.geometry.range_frequencies_hz.size() != cols ||
      request.geometry.source_range_frequency_queries_hz.size() != rows * cols) {
    return Reject(request, OmegaKGridReductionReason::kInvalidGeometry);
  }
  const std::vector<std::size_t>& indices =
      request.common_support.largest_contiguous_original_column_indices;
  if (!request.common_support.valid || !request.common_support.usable_for_interpolation ||
      indices.size() < 2U ||
      indices.size() != request.common_support.largest_contiguous_column_count ||
      request.common_support.common_valid_column_mask.size() != cols) {
    return Reject(request, OmegaKGridReductionReason::kInvalidCommonSupport);
  }
  if (request.source_spectrum.rows != rows || request.source_spectrum.cols != cols ||
      request.source_spectrum.values.size() != rows * cols) {
    return Reject(request, OmegaKGridReductionReason::kInvalidSourceSpectrum);
  }

  StoltInterpolationRequest interpolation;
  interpolation.source_range_frequencies_hz = request.geometry.range_frequencies_hz;
  interpolation.source_spectrum = request.source_spectrum;
  interpolation.source_frequency_queries_hz.rows = rows;
  interpolation.source_frequency_queries_hz.cols = indices.size();
  interpolation.source_frequency_queries_hz.values.resize(rows * indices.size());
  for (std::size_t reduced_col = 0U; reduced_col < indices.size(); ++reduced_col) {
    const std::size_t original_col = indices[reduced_col];
    if (original_col >= cols ||
        !request.common_support.common_valid_column_mask[original_col]) {
      return Reject(request, OmegaKGridReductionReason::kInvalidCommonSupport);
    }
    const double target_hz = request.geometry.range_frequencies_hz[original_col];
    if (!interpolation.target_range_frequencies_hz.empty() &&
        !(target_hz > interpolation.target_range_frequencies_hz.back())) {
      return Reject(request, OmegaKGridReductionReason::kInvalidCommonSupport);
    }
    interpolation.target_range_frequencies_hz.push_back(target_hz);
    for (std::size_t row = 0U; row < rows; ++row) {
      interpolation.source_frequency_queries_hz(row, reduced_col) =
          signal::ComplexSample(
              request.geometry.source_range_frequency_queries_hz[row * cols + original_col],
              0.0);
    }
  }

  const StoltInterpolationResult interpolated = InterpolateOmegaKStoltLinear(interpolation);
  if (interpolated.status != StoltInterpolationStatus::kSucceeded) {
    return Reject(request, OmegaKGridReductionReason::kInterpolationFailure);
  }
  OmegaKGridReductionResult result;
  result.request_id = request.request_id;
  result.status = OmegaKGridReductionStatus::kSucceeded;
  result.reason = OmegaKGridReductionReason::kNone;
  result.reduced_target_frequencies_hz = interpolation.target_range_frequencies_hz;
  result.original_column_indices = indices;
  result.interpolation_diagnostics = interpolated.diagnostics;
  result.reduced_spectrum = interpolated.interpolated_spectrum;
  return result;
}

}  // namespace imaging
}  // namespace sar
