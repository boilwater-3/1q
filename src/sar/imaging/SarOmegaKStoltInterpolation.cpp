#include "sar/imaging/SarOmegaKStoltInterpolation.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace sar {
namespace imaging {

namespace {

struct FrequencyBin {
  double frequency_hz{0.0};
  std::size_t source_index{0U};
};

bool IsFiniteComplex(const signal::ComplexSample& sample) {
  return std::isfinite(sample.real()) && std::isfinite(sample.imag());
}

bool HasValidShape(const signal::ComplexMatrix& matrix) {
  return matrix.rows > 0U && matrix.cols > 0U &&
         matrix.values.size() == matrix.rows * matrix.cols;
}

StoltInterpolationResult Reject(StoltInterpolationRejectionReason reason,
                                const StoltInterpolationDiagnostics& diagnostics) {
  StoltInterpolationResult result;
  result.reason = reason;
  result.diagnostics = diagnostics;
  return result;
}

}  // namespace

StoltInterpolationResult InterpolateOmegaKStoltLinear(
    const StoltInterpolationRequest& request) {
  StoltInterpolationDiagnostics diagnostics;
  if (request.source_range_frequencies_hz.size() < 2U) {
    return Reject(StoltInterpolationRejectionReason::kInvalidFrequencyAxis, diagnostics);
  }

  std::vector<FrequencyBin> sorted_bins;
  sorted_bins.reserve(request.source_range_frequencies_hz.size());
  for (std::size_t index = 0U; index < request.source_range_frequencies_hz.size(); ++index) {
    if (!std::isfinite(request.source_range_frequencies_hz[index])) {
      return Reject(StoltInterpolationRejectionReason::kInvalidFrequencyAxis, diagnostics);
    }
    FrequencyBin bin;
    bin.frequency_hz = request.source_range_frequencies_hz[index];
    bin.source_index = index;
    sorted_bins.push_back(bin);
  }
  std::sort(sorted_bins.begin(), sorted_bins.end(),
            [](const FrequencyBin& first, const FrequencyBin& second) {
              return first.frequency_hz < second.frequency_hz;
            });
  for (std::size_t index = 1U; index < sorted_bins.size(); ++index) {
    if (!(sorted_bins[index].frequency_hz > sorted_bins[index - 1U].frequency_hz)) {
      return Reject(StoltInterpolationRejectionReason::kInvalidFrequencyAxis, diagnostics);
    }
  }

  if (!HasValidShape(request.source_spectrum) ||
      request.source_spectrum.cols != sorted_bins.size()) {
    return Reject(StoltInterpolationRejectionReason::kInvalidSpectrum, diagnostics);
  }
  for (const signal::ComplexSample& sample : request.source_spectrum.values) {
    if (!IsFiniteComplex(sample)) {
      return Reject(StoltInterpolationRejectionReason::kInvalidSpectrum, diagnostics);
    }
  }
  if (!HasValidShape(request.source_frequency_queries_hz) ||
      request.source_frequency_queries_hz.rows != request.source_spectrum.rows ||
      request.target_range_frequencies_hz.size() != request.source_frequency_queries_hz.cols) {
    return Reject(StoltInterpolationRejectionReason::kInvalidQueries, diagnostics);
  }
  for (const double frequency_hz : request.target_range_frequencies_hz) {
    if (!std::isfinite(frequency_hz)) {
      return Reject(StoltInterpolationRejectionReason::kInvalidQueries, diagnostics);
    }
  }

  diagnostics.row_count = request.source_spectrum.rows;
  diagnostics.range_bin_count = request.source_frequency_queries_hz.cols;
  diagnostics.query_count = request.source_frequency_queries_hz.values.size();
  signal::ComplexMatrix output;
  output.rows = request.source_frequency_queries_hz.rows;
  output.cols = request.source_frequency_queries_hz.cols;
  output.values.assign(output.rows * output.cols, signal::ComplexSample(0.0, 0.0));
  const double minimum_hz = sorted_bins.front().frequency_hz;
  const double maximum_hz = sorted_bins.back().frequency_hz;
  for (std::size_t row = 0U; row < request.source_spectrum.rows; ++row) {
    for (std::size_t col = 0U; col < request.source_frequency_queries_hz.cols; ++col) {
      const double query_hz = request.source_frequency_queries_hz(row, col).real();
      if (!std::isfinite(query_hz) ||
          request.source_frequency_queries_hz(row, col).imag() != 0.0) {
        return Reject(StoltInterpolationRejectionReason::kInvalidQueries, diagnostics);
      }
      diagnostics.maximum_abs_shift_hz =
          std::max(diagnostics.maximum_abs_shift_hz,
                   std::abs(query_hz - request.target_range_frequencies_hz[col]));
      if (query_hz < minimum_hz || query_hz > maximum_hz) {
        ++diagnostics.out_of_support_query_count;
        return Reject(StoltInterpolationRejectionReason::kOutOfSupportQuery, diagnostics);
      }

      const auto upper = std::lower_bound(
          sorted_bins.begin(), sorted_bins.end(), query_hz,
          [](const FrequencyBin& bin, double frequency_hz) {
            return bin.frequency_hz < frequency_hz;
          });
      if (upper != sorted_bins.end() && upper->frequency_hz == query_hz) {
        output(row, col) = request.source_spectrum(row, upper->source_index);
        ++diagnostics.exact_hit_count;
        continue;
      }
      if (upper == sorted_bins.begin() || upper == sorted_bins.end()) {
        return Reject(StoltInterpolationRejectionReason::kInterpolationFailure, diagnostics);
      }
      const FrequencyBin& right = *upper;
      const FrequencyBin& left = *(upper - 1);
      const double interval_hz = right.frequency_hz - left.frequency_hz;
      const double weight = (query_hz - left.frequency_hz) / interval_hz;
      output(row, col) = (1.0 - weight) * request.source_spectrum(row, left.source_index) +
                         weight * request.source_spectrum(row, right.source_index);
      diagnostics.maximum_interpolation_interval_hz =
          std::max(diagnostics.maximum_interpolation_interval_hz, interval_hz);
      ++diagnostics.linear_interpolation_count;
    }
  }

  StoltInterpolationResult result;
  result.status = StoltInterpolationStatus::kSucceeded;
  result.reason = StoltInterpolationRejectionReason::kNone;
  result.diagnostics = diagnostics;
  result.interpolated_spectrum = output;
  return result;
}

}  // namespace imaging
}  // namespace sar
