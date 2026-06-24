#include "sar/imaging/SarOmegaKCommonSupport.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace sar {
namespace imaging {

namespace {

bool IsFiniteVector(const std::vector<double>& values) {
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

}  // namespace

bool DiagnoseOmegaKCommonStoltSupport(
    const OmegaKGeometryDiagnostics& geometry,
    OmegaKCommonSupportDiagnostics* diagnostics) {
  if (diagnostics == nullptr) {
    return false;
  }
  *diagnostics = OmegaKCommonSupportDiagnostics{};
  const std::size_t rows = geometry.azimuth_frequency_bin_count;
  const std::size_t cols = geometry.range_frequency_bin_count;
  if (rows == 0U || cols < 2U || geometry.range_frequencies_hz.size() != cols ||
      geometry.source_range_frequency_queries_hz.size() != rows * cols ||
      geometry.stolt_shifts_hz.size() != rows * cols ||
      !IsFiniteVector(geometry.range_frequencies_hz) ||
      !IsFiniteVector(geometry.source_range_frequency_queries_hz) ||
      !IsFiniteVector(geometry.stolt_shifts_hz)) {
    return false;
  }

  const auto range_minmax = std::minmax_element(geometry.range_frequencies_hz.begin(),
                                                 geometry.range_frequencies_hz.end());
  const double minimum_hz = *range_minmax.first;
  const double maximum_hz = *range_minmax.second;
  diagnostics->azimuth_row_count = rows;
  diagnostics->target_range_bin_count = cols;
  diagnostics->total_query_count = rows * cols;
  diagnostics->out_of_support_query_count_per_column.assign(cols, 0U);
  diagnostics->common_valid_column_mask.assign(cols, true);
  for (std::size_t row = 0U; row < rows; ++row) {
    for (std::size_t col = 0U; col < cols; ++col) {
      const std::size_t index = row * cols + col;
      const double query_hz = geometry.source_range_frequency_queries_hz[index];
      diagnostics->maximum_abs_stolt_shift_hz =
          std::max(diagnostics->maximum_abs_stolt_shift_hz,
                   std::abs(geometry.stolt_shifts_hz[index]));
      if (query_hz < minimum_hz || query_hz > maximum_hz) {
        ++diagnostics->out_of_support_query_count_per_column[col];
        diagnostics->common_valid_column_mask[col] = false;
      }
    }
  }

  for (const bool valid : diagnostics->common_valid_column_mask) {
    if (valid) {
      ++diagnostics->common_valid_column_count;
    }
  }
  diagnostics->discarded_column_count = cols - diagnostics->common_valid_column_count;
  diagnostics->common_valid_ratio =
      static_cast<double>(diagnostics->common_valid_column_count) / static_cast<double>(cols);

  std::vector<std::pair<double, std::size_t>> sorted_columns;
  sorted_columns.reserve(cols);
  for (std::size_t col = 0U; col < cols; ++col) {
    sorted_columns.push_back(std::make_pair(geometry.range_frequencies_hz[col], col));
  }
  std::sort(sorted_columns.begin(), sorted_columns.end());
  std::size_t current_start = 0U;
  std::size_t current_count = 0U;
  std::size_t best_start = 0U;
  std::size_t best_count = 0U;
  for (std::size_t sorted_index = 0U; sorted_index < sorted_columns.size(); ++sorted_index) {
    if (diagnostics->common_valid_column_mask[sorted_columns[sorted_index].second]) {
      if (current_count == 0U) {
        current_start = sorted_index;
      }
      ++current_count;
      if (current_count > best_count) {
        best_start = current_start;
        best_count = current_count;
      }
    } else {
      current_count = 0U;
    }
  }

  diagnostics->largest_contiguous_column_count = best_count;
  if (best_count > 0U) {
    diagnostics->largest_contiguous_minimum_frequency_hz = sorted_columns[best_start].first;
    diagnostics->largest_contiguous_maximum_frequency_hz =
        sorted_columns[best_start + best_count - 1U].first;
    for (std::size_t index = 0U; index < best_count; ++index) {
      diagnostics->largest_contiguous_original_column_indices.push_back(
          sorted_columns[best_start + index].second);
    }
  }
  diagnostics->usable_for_interpolation = best_count >= 2U;
  diagnostics->valid = true;
  return true;
}

}  // namespace imaging
}  // namespace sar
