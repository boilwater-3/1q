#include "sar/imaging/SarSlowTimeResampling.h"

#include <algorithm>
#include <cmath>

namespace sar {
namespace imaging {

namespace {

bool IsFinite(const std::complex<double>& value) {
  return std::isfinite(value.real()) && std::isfinite(value.imag());
}

}  // namespace

bool ResampleSlowTimeLinear(const std::vector<double>& explicit_times_s,
                            const std::vector<std::complex<double>>& input_samples,
                            std::vector<std::complex<double>>* output_samples,
                            SlowTimeResamplingDiagnostics* diagnostics) {
  if (output_samples == nullptr || diagnostics == nullptr) {
    return false;
  }
  output_samples->clear();
  *diagnostics = SlowTimeResamplingDiagnostics{};
  if (explicit_times_s.size() < 2U || explicit_times_s.size() != input_samples.size()) {
    return false;
  }
  for (std::size_t index = 0U; index < explicit_times_s.size(); ++index) {
    if (!std::isfinite(explicit_times_s[index]) || !IsFinite(input_samples[index]) ||
        (index > 0U && explicit_times_s[index] <= explicit_times_s[index - 1U])) {
      return false;
    }
  }

  diagnostics->sample_count = explicit_times_s.size();
  diagnostics->duration_s = explicit_times_s.back() - explicit_times_s.front();
  diagnostics->nominal_interval_s =
      diagnostics->duration_s / static_cast<double>(explicit_times_s.size() - 1U);
  diagnostics->nominal_prf_hz = 1.0 / diagnostics->nominal_interval_s;
  diagnostics->uniform_tolerance_s =
      std::max(1.0e-12, std::abs(diagnostics->nominal_interval_s) * 1.0e-9);
  diagnostics->minimum_actual_interval_s = diagnostics->duration_s;
  diagnostics->nominal_times_s.reserve(explicit_times_s.size());

  double sum_interval_deviation_squared = 0.0;
  for (std::size_t index = 1U; index < explicit_times_s.size(); ++index) {
    const double interval = explicit_times_s[index] - explicit_times_s[index - 1U];
    const double deviation = interval - diagnostics->nominal_interval_s;
    diagnostics->minimum_actual_interval_s =
        std::min(diagnostics->minimum_actual_interval_s, interval);
    diagnostics->maximum_actual_interval_s =
        std::max(diagnostics->maximum_actual_interval_s, interval);
    diagnostics->maximum_abs_interval_deviation_s =
        std::max(diagnostics->maximum_abs_interval_deviation_s, std::abs(deviation));
    sum_interval_deviation_squared += deviation * deviation;
  }
  diagnostics->interval_deviation_rms_s =
      std::sqrt(sum_interval_deviation_squared /
                static_cast<double>(explicit_times_s.size() - 1U));

  double sum_time_deviation_squared = 0.0;
  for (std::size_t index = 0U; index < explicit_times_s.size(); ++index) {
    const double nominal_time =
        index + 1U == explicit_times_s.size()
            ? explicit_times_s.back()
            : explicit_times_s.front() +
                  static_cast<double>(index) * diagnostics->nominal_interval_s;
    const double deviation = explicit_times_s[index] - nominal_time;
    diagnostics->nominal_times_s.push_back(nominal_time);
    diagnostics->maximum_abs_time_axis_deviation_s =
        std::max(diagnostics->maximum_abs_time_axis_deviation_s, std::abs(deviation));
    sum_time_deviation_squared += deviation * deviation;
  }
  diagnostics->time_axis_deviation_rms_s =
      std::sqrt(sum_time_deviation_squared / static_cast<double>(explicit_times_s.size()));
  diagnostics->uniform_within_tolerance =
      diagnostics->maximum_abs_interval_deviation_s <= diagnostics->uniform_tolerance_s;

  output_samples->reserve(input_samples.size());
  output_samples->push_back(input_samples.front());
  std::size_t left = 0U;
  for (std::size_t query_index = 1U; query_index + 1U < explicit_times_s.size();
       ++query_index) {
    const double query_time = diagnostics->nominal_times_s[query_index];
    while (left + 1U < explicit_times_s.size() && explicit_times_s[left + 1U] < query_time) {
      ++left;
    }
    if (left + 1U >= explicit_times_s.size() || query_time < explicit_times_s[left] ||
        query_time > explicit_times_s[left + 1U]) {
      output_samples->clear();
      *diagnostics = SlowTimeResamplingDiagnostics{};
      return false;
    }
    const double weight =
        (query_time - explicit_times_s[left]) /
        (explicit_times_s[left + 1U] - explicit_times_s[left]);
    output_samples->push_back(input_samples[left] * (1.0 - weight) +
                              input_samples[left + 1U] * weight);
  }
  output_samples->push_back(input_samples.back());
  diagnostics->valid = true;
  return true;
}

bool ResampleRawHistorySlowTimeLinear(const std::vector<double>& explicit_times_s,
                                      const signal::ComplexMatrix& input,
                                      signal::ComplexMatrix* output,
                                      SlowTimeResamplingDiagnostics* diagnostics) {
  if (output == nullptr || diagnostics == nullptr) {
    return false;
  }
  *output = signal::ComplexMatrix{};
  *diagnostics = SlowTimeResamplingDiagnostics{};
  if (input.rows < 2U || input.cols == 0U || input.values.size() != input.rows * input.cols ||
      explicit_times_s.size() != input.rows) {
    return false;
  }

  signal::ComplexMatrix result;
  result.rows = input.rows;
  result.cols = input.cols;
  result.values.resize(input.rows * input.cols);
  std::vector<std::complex<double>> input_column(input.rows);
  std::vector<std::complex<double>> output_column;
  SlowTimeResamplingDiagnostics shared_diagnostics;
  for (std::size_t col = 0U; col < input.cols; ++col) {
    for (std::size_t row = 0U; row < input.rows; ++row) {
      input_column[row] = input(row, col);
    }
    SlowTimeResamplingDiagnostics column_diagnostics;
    if (!ResampleSlowTimeLinear(explicit_times_s, input_column, &output_column,
                                &column_diagnostics)) {
      return false;
    }
    if (col == 0U) {
      shared_diagnostics = column_diagnostics;
    }
    for (std::size_t row = 0U; row < input.rows; ++row) {
      result(row, col) = output_column[row];
    }
  }

  *output = result;
  *diagnostics = shared_diagnostics;
  return true;
}

}  // namespace imaging
}  // namespace sar
