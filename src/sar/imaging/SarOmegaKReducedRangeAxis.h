/**
 * @file SarOmegaKReducedRangeAxis.h
 * @brief Omega-K 收缩距离频率网格与相对延迟轴诊断。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_REDUCED_RANGE_AXIS_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_REDUCED_RANGE_AXIS_H_

#include <cstddef>
#include <vector>

namespace sar {
namespace imaging {

struct OmegaKReducedRangeAxisDiagnostics {
  bool valid{false};
  std::size_t sample_count{0U};
  double frequency_spacing_hz{0.0};
  double maximum_abs_spacing_deviation_hz{0.0};
  double effective_bandwidth_hz{0.0};
  double unambiguous_delay_window_s{0.0};
  double relative_delay_spacing_s{0.0};
  double relative_range_spacing_m{0.0};
  std::vector<double> relative_delays_s;
};

bool DiagnoseOmegaKReducedRangeAxis(
    const std::vector<double>& reduced_range_frequencies_hz,
    OmegaKReducedRangeAxisDiagnostics* diagnostics);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_REDUCED_RANGE_AXIS_H_
