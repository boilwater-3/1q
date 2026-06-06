/**
 * @file SarMotionCompensation.h
 * @brief SAR 内部一阶运动补偿工具。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_MOTION_COMPENSATION_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_MOTION_COMPENSATION_H_

#include <cstddef>
#include <vector>

#include "sar/geometry/SarGeometry.h"
#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

struct FirstOrderMotionCompensationConfig {
  double sample_rate_hz{0.0};
  double carrier_frequency_hz{0.0};
  geometry::LocalPoint reference_point_m{};
};

struct MotionCompensationDiagnostics {
  double max_abs_range_error_m{0.0};
  double rms_range_error_m{0.0};
  double max_abs_envelope_shift_bins{0.0};
  std::size_t compensated_pulses{0U};
  std::size_t out_of_bounds_samples{0U};
};

bool ApplyFirstOrderMotionCompensation(
    const FirstOrderMotionCompensationConfig& config,
    const std::vector<geometry::PlatformPulseState>& ideal_trajectory,
    const std::vector<geometry::PlatformPulseState>& actual_trajectory,
    const signal::ComplexMatrix& actual_raw_pulse_history, signal::ComplexMatrix* compensated,
    MotionCompensationDiagnostics* diagnostics);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_MOTION_COMPENSATION_H_
