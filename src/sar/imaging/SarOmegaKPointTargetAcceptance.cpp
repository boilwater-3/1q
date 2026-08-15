#include "sar/imaging/SarOmegaKPointTargetAcceptance.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "sar/signal/SarAngleWrap.h"

namespace sar {
namespace imaging {

namespace {

constexpr double kMinimumReportedDb = -300.0;

OmegaKPointTargetAcceptanceResult Reject(const OmegaKPointTargetAcceptanceRequest& request,
                                         OmegaKPointTargetAcceptanceReason reason) {
  OmegaKPointTargetAcceptanceResult result;
  result.request_id = request.request_id;
  result.reason = reason;
  return result;
}

bool IsStrictlyIncreasingFinite(const std::vector<double>& axis) {
  if (axis.empty()) {
    return false;
  }
  for (std::size_t index = 0U; index < axis.size(); ++index) {
    if (!std::isfinite(axis[index]) ||
        (index > 0U && axis[index] <= axis[index - 1U])) {
      return false;
    }
  }
  return true;
}

bool IsFiniteNonnegative(double value) {
  return std::isfinite(value) && value >= 0.0;
}

bool IsValidTolerances(const OmegaKPointTargetTolerances& value) {
  return IsFiniteNonnegative(value.maximum_range_error_m) &&
         IsFiniteNonnegative(value.maximum_azimuth_error) &&
         IsFiniteNonnegative(value.maximum_abs_phase_error_rad) &&
         IsFiniteNonnegative(value.maximum_relative_magnitude_error) &&
         std::isfinite(value.maximum_range_pslr_db) &&
         std::isfinite(value.maximum_azimuth_pslr_db) &&
         std::isfinite(value.maximum_range_islr_db) &&
         std::isfinite(value.maximum_azimuth_islr_db);
}

bool IsValidCandidate(const signal::ComplexMatrix& matrix, std::size_t rows,
                      std::size_t cols) {
  if (matrix.rows != rows || matrix.cols != cols ||
      matrix.values.size() != rows * cols) {
    return false;
  }
  for (const signal::ComplexSample& sample : matrix.values) {
    if (!std::isfinite(sample.real()) || !std::isfinite(sample.imag())) {
      return false;
    }
  }
  return true;
}

double RatioDb(double numerator, double denominator) {
  if (numerator <= 0.0) {
    return kMinimumReportedDb;
  }
  return 10.0 * std::log10(numerator / denominator);
}

void MeasureCut(const std::vector<double>& powers, std::size_t peak,
                std::size_t half_width, double* pslr_db, double* islr_db) {
  const std::size_t first = peak > half_width ? peak - half_width : 0U;
  const std::size_t last = std::min(powers.size() - 1U, peak + half_width);
  double main_power = 0.0;
  double side_power = 0.0;
  double maximum_side_power = 0.0;
  for (std::size_t index = 0U; index < powers.size(); ++index) {
    if (index >= first && index <= last) {
      main_power += powers[index];
    } else {
      side_power += powers[index];
      maximum_side_power = std::max(maximum_side_power, powers[index]);
    }
  }
  *pslr_db = RatioDb(maximum_side_power, powers[peak]);
  *islr_db = RatioDb(side_power, main_power);
}

}  // namespace

OmegaKPointTargetAcceptanceResult EvaluateOmegaKPointTargetCandidate(
    const OmegaKPointTargetAcceptanceRequest& request) {
  if (request.request_id == 0U || !IsValidTolerances(request.tolerances) ||
      !std::isfinite(request.truth.absolute_slant_range_m) ||
      !std::isfinite(request.truth.azimuth_coordinate) ||
      !std::isfinite(request.truth.peak_phase_rad) ||
      !std::isfinite(request.truth.peak_magnitude) ||
      request.truth.peak_magnitude <= 0.0) {
    return Reject(request, OmegaKPointTargetAcceptanceReason::kInvalidRequest);
  }
  if (!request.truth.independently_generated) {
    return Reject(request, OmegaKPointTargetAcceptanceReason::kTruthNotIndependent);
  }
  if (!request.truth.inside_common_support) {
    return Reject(request, OmegaKPointTargetAcceptanceReason::kOutsideCommonSupport);
  }
  if (!IsStrictlyIncreasingFinite(request.absolute_slant_ranges_m) ||
      !IsStrictlyIncreasingFinite(request.azimuth_coordinates) ||
      !IsValidCandidate(request.numerical_image_candidate,
                        request.azimuth_coordinates.size(),
                        request.absolute_slant_ranges_m.size())) {
    return Reject(request, OmegaKPointTargetAcceptanceReason::kInvalidCandidate);
  }

  std::size_t peak_index = 0U;
  double peak_power = -1.0;
  for (std::size_t index = 0U; index < request.numerical_image_candidate.values.size();
       ++index) {
    const double power = std::norm(request.numerical_image_candidate.values[index]);
    if (power > peak_power) {
      peak_power = power;
      peak_index = index;
    }
  }
  const std::size_t peak_row = peak_index / request.numerical_image_candidate.cols;
  const std::size_t peak_col = peak_index % request.numerical_image_candidate.cols;
  const signal::ComplexSample peak = request.numerical_image_candidate.values[peak_index];

  std::vector<double> range_cut(request.numerical_image_candidate.cols);
  std::vector<double> azimuth_cut(request.numerical_image_candidate.rows);
  for (std::size_t col = 0U; col < range_cut.size(); ++col) {
    range_cut[col] = std::norm(request.numerical_image_candidate(peak_row, col));
  }
  for (std::size_t row = 0U; row < azimuth_cut.size(); ++row) {
    azimuth_cut[row] = std::norm(request.numerical_image_candidate(row, peak_col));
  }

  OmegaKPointTargetAcceptanceResult result;
  result.request_id = request.request_id;
  result.reason = OmegaKPointTargetAcceptanceReason::kNone;
  result.peak_row = peak_row;
  result.peak_col = peak_col;
  result.range_error_m = std::abs(request.absolute_slant_ranges_m[peak_col] -
                                  request.truth.absolute_slant_range_m);
  result.azimuth_error = std::abs(request.azimuth_coordinates[peak_row] -
                                  request.truth.azimuth_coordinate);
  result.wrapped_phase_error_rad =
      std::abs(signal::WrapPhase(std::arg(peak) - request.truth.peak_phase_rad));
  result.relative_magnitude_error =
      std::abs(std::abs(peak) - request.truth.peak_magnitude) /
      request.truth.peak_magnitude;
  MeasureCut(range_cut, peak_col, request.truth.range_mainlobe_half_width,
             &result.range_pslr_db, &result.range_islr_db);
  MeasureCut(azimuth_cut, peak_row, request.truth.azimuth_mainlobe_half_width,
             &result.azimuth_pslr_db, &result.azimuth_islr_db);

  const bool passed =
      result.range_error_m <= request.tolerances.maximum_range_error_m &&
      result.azimuth_error <= request.tolerances.maximum_azimuth_error &&
      result.wrapped_phase_error_rad <= request.tolerances.maximum_abs_phase_error_rad &&
      result.relative_magnitude_error <=
          request.tolerances.maximum_relative_magnitude_error &&
      result.range_pslr_db <= request.tolerances.maximum_range_pslr_db &&
      result.azimuth_pslr_db <= request.tolerances.maximum_azimuth_pslr_db &&
      result.range_islr_db <= request.tolerances.maximum_range_islr_db &&
      result.azimuth_islr_db <= request.tolerances.maximum_azimuth_islr_db;
  result.status = passed ? OmegaKPointTargetAcceptanceStatus::kPassed
                         : OmegaKPointTargetAcceptanceStatus::kFailed;
  return result;
}

}  // namespace imaging
}  // namespace sar
