/**
 * @file SarRda.h
 * @brief SAR 内部 L1 条带 RDA 聚焦工具。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_RDA_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_RDA_H_

#include <cstddef>
#include <string>
#include <vector>

#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

enum class RcmcInterpolation {
  kNone = 0,
  kLinear = 1,
  kSinc = 2,
};

struct RdaConfig {
  double sample_rate_hz{0.0};
  double carrier_frequency_hz{0.0};
  double prf_hz{0.0};
  double platform_velocity_mps{0.0};
  double reference_range_m{0.0};
  RcmcInterpolation rcmc_interpolation{RcmcInterpolation::kLinear};
  std::size_t sinc_half_width{4U};
};

struct RdaDiagnostics {
  double reference_range_m{0.0};
  double doppler_rate_hz_per_s{0.0};
  double range_bin_spacing_m{0.0};
  double azimuth_width_3db_bins{0.0};
  double image_entropy_nats{0.0};
  std::size_t out_of_bounds_samples{0U};
  std::string rcmc_interpolation{"none"};
  bool range_compression_applied{false};
  bool azimuth_fft_applied{false};
  bool azimuth_matched_filter_applied{false};
  bool azimuth_ifft_applied{false};
};

struct FocusedSarImage {
  signal::ComplexMatrix image{};
  RdaDiagnostics diagnostics{};
};

bool FocusStripmapRda(const RdaConfig& config, const signal::ComplexMatrix& raw_pulse_history,
                      const signal::ComplexVector& matched_filter, FocusedSarImage* output);

bool ApplyRangeMigrationCorrection(const signal::ComplexMatrix& input,
                                   const std::vector<double>& delta_bins_by_row,
                                   RcmcInterpolation interpolation, std::size_t sinc_half_width,
                                   signal::ComplexMatrix* output,
                                   std::size_t* out_of_bounds_samples);

std::size_t FindPeakIndex(const signal::ComplexMatrix& image);
double EstimateAzimuthWidth3dbBins(const signal::ComplexMatrix& image);
double EstimateImageEntropyNats(const signal::ComplexMatrix& image);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_RDA_H_
