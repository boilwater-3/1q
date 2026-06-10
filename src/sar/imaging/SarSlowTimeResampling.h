/**
 * @file SarSlowTimeResampling.h
 * @brief 时变 PRF 慢时间轴诊断与线性重采样。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_SLOW_TIME_RESAMPLING_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_SLOW_TIME_RESAMPLING_H_

#include <complex>
#include <cstddef>
#include <vector>

#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

struct SlowTimeResamplingDiagnostics {
  bool valid{false};
  bool uniform_within_tolerance{false};
  std::size_t sample_count{0U};
  double duration_s{0.0};
  double nominal_interval_s{0.0};
  double nominal_prf_hz{0.0};
  double minimum_actual_interval_s{0.0};
  double maximum_actual_interval_s{0.0};
  double maximum_abs_interval_deviation_s{0.0};
  double interval_deviation_rms_s{0.0};
  double maximum_abs_time_axis_deviation_s{0.0};
  double time_axis_deviation_rms_s{0.0};
  double uniform_tolerance_s{0.0};
  std::vector<double> nominal_times_s;
};

struct SlowTimeGapDiagnostics {
  bool valid{false};
  bool resampling_allowed{false};
  std::size_t sample_count{0U};
  double expected_interval_s{0.0};
  double expected_prf_hz{0.0};
  double minimum_actual_gap_s{0.0};
  double maximum_actual_gap_s{0.0};
  double maximum_gap_ratio{0.0};
  std::size_t rejected_gap_count{0U};
  std::size_t suspected_missing_pulse_count{0U};
  std::size_t first_rejected_gap_index{static_cast<std::size_t>(-1)};
};

bool DiagnoseSlowTimeGaps(const std::vector<double>& explicit_times_s,
                          double expected_interval_s,
                          SlowTimeGapDiagnostics* diagnostics);

bool ResampleSlowTimeLinear(const std::vector<double>& explicit_times_s,
                            const std::vector<std::complex<double>>& input_samples,
                            std::vector<std::complex<double>>* output_samples,
                            SlowTimeResamplingDiagnostics* diagnostics);

bool ResampleRawHistorySlowTimeLinear(const std::vector<double>& explicit_times_s,
                                      const signal::ComplexMatrix& input,
                                      signal::ComplexMatrix* output,
                                      SlowTimeResamplingDiagnostics* diagnostics);

bool ResampleRawHistorySlowTimeLinearGuarded(
    const std::vector<double>& explicit_times_s, double expected_interval_s,
    const signal::ComplexMatrix& input, signal::ComplexMatrix* output,
    SlowTimeGapDiagnostics* gap_diagnostics,
    SlowTimeResamplingDiagnostics* resampling_diagnostics);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_SLOW_TIME_RESAMPLING_H_
