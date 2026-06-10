/**
 * @file SarCsaGeometry.h
 * @brief CSA 频率几何基础与有效域诊断。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_CSA_GEOMETRY_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_CSA_GEOMETRY_H_

#include <cstddef>
#include <vector>

namespace sar {
namespace imaging {

struct CsaGeometryConfig {
  std::size_t range_sample_count{0U};
  std::size_t azimuth_pulse_count{0U};
  double sample_rate_hz{0.0};
  double prf_hz{0.0};
  double carrier_frequency_hz{0.0};
  double platform_velocity_mps{0.0};
  double reference_range_m{0.0};
};

struct CsaGeometryDiagnostics {
  bool valid{false};
  std::size_t range_frequency_bin_count{0U};
  std::size_t azimuth_frequency_bin_count{0U};
  double range_frequency_spacing_hz{0.0};
  double azimuth_frequency_spacing_hz{0.0};
  double wavelength_m{0.0};
  double minimum_doppler_factor{0.0};
  double maximum_abs_chirp_scaling_factor{0.0};
  std::size_t invalid_doppler_bin_count{0U};
  double valid_doppler_limit_hz{0.0};
  double maximum_abs_azimuth_frequency_hz{0.0};
  double doppler_validity_margin_hz{0.0};
  std::vector<double> range_frequencies_hz;
  std::vector<double> azimuth_frequencies_hz;
  std::vector<double> doppler_factors;
  std::vector<double> chirp_scaling_factors;
};

// Returns false for structurally invalid input. A true return with
// diagnostics.valid=false reports Doppler bins outside the approved CSA domain.
bool EvaluateCsaFrequencyGeometry(const CsaGeometryConfig& config,
                                  CsaGeometryDiagnostics* diagnostics);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_CSA_GEOMETRY_H_
