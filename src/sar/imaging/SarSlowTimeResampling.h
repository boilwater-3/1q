/**
 * @file SarSlowTimeResampling.h
 * @brief 时变 PRF 慢时间轴诊断与线性重采样。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_SLOW_TIME_RESAMPLING_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_SLOW_TIME_RESAMPLING_H_

#include <complex>
#include <cstddef>
#include <vector>

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

bool ResampleSlowTimeLinear(const std::vector<double>& explicit_times_s,
                            const std::vector<std::complex<double>>& input_samples,
                            std::vector<std::complex<double>>* output_samples,
                            SlowTimeResamplingDiagnostics* diagnostics);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_SLOW_TIME_RESAMPLING_H_
