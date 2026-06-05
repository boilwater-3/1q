/**
 * @file SarGbp.h
 * @brief SAR 内部小场景全局后向投影参考聚焦工具。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_GBP_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_GBP_H_

#include <cstddef>
#include <string>
#include <vector>

#include "sar/geometry/SarGeometry.h"
#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

struct GbpGridConfig {
  std::size_t azimuth_pixel_count{0U};
  std::size_t range_pixel_count{0U};
  double azimuth_start_m{0.0};
  double range_start_m{0.0};
  double azimuth_spacing_m{0.0};
  double range_spacing_m{0.0};
  double image_plane_z_m{0.0};
};

struct GbpConfig {
  double sample_rate_hz{0.0};
  double carrier_frequency_hz{0.0};
  GbpGridConfig grid{};
};

struct GbpDiagnostics {
  std::size_t evaluated_pixels{0U};
  std::size_t accumulated_samples{0U};
  std::size_t out_of_bounds_samples{0U};
  std::size_t max_approved_dimension{128U};
  std::string range_interpolation{"linear"};
};

struct FocusedGbpImage {
  signal::ComplexMatrix image{};
  GbpDiagnostics diagnostics{};
};

bool FocusSmallSceneGbp(const GbpConfig& config,
                        const std::vector<geometry::PlatformPulseState>& pulses,
                        const signal::ComplexMatrix& raw_pulse_history,
                        const signal::ComplexVector& matched_filter, FocusedGbpImage* output);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_GBP_H_
