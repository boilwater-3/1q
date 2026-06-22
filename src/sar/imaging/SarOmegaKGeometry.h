// ============================================================================
// 【未进行设计需求，不再扩展 — DEPRECATED】
// 本文件不参与构建（见 src/sar/CMakeLists.txt 的 SAR_ENGINE_SOURCES 注释），
// 仅作为探索性参考保留。请勿新增依赖或据此实施。
// ============================================================================

/**
 * @file SarOmegaKGeometry.h
 * @brief Omega-K 波数与 Stolt 查询几何诊断。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_GEOMETRY_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_GEOMETRY_H_

#include <cstddef>
#include <vector>

namespace sar {
namespace imaging {

struct OmegaKGeometryConfig {
  std::size_t range_sample_count{0U};
  std::size_t azimuth_pulse_count{0U};
  double sample_rate_hz{0.0};
  double prf_hz{0.0};
  double carrier_frequency_hz{0.0};
  double platform_velocity_mps{0.0};
  double reference_range_m{0.0};
};

struct OmegaKGeometryDiagnostics {
  bool valid{false};
  std::size_t range_frequency_bin_count{0U};
  std::size_t azimuth_frequency_bin_count{0U};
  double range_frequency_spacing_hz{0.0};
  double azimuth_frequency_spacing_hz{0.0};
  double wavelength_m{0.0};
  double minimum_range_wavenumber_rad_per_m{0.0};
  double maximum_range_wavenumber_rad_per_m{0.0};
  double maximum_abs_azimuth_wavenumber_rad_per_m{0.0};
  double minimum_valid_propagation_wavenumber_rad_per_m{0.0};
  double maximum_valid_propagation_wavenumber_rad_per_m{0.0};
  std::size_t invalid_dispersion_point_count{0U};
  std::size_t out_of_support_stolt_query_count{0U};
  double minimum_source_range_frequency_hz{0.0};
  double maximum_source_range_frequency_hz{0.0};
  double maximum_abs_stolt_shift_hz{0.0};
  std::vector<double> range_frequencies_hz;
  std::vector<double> azimuth_frequencies_hz;
  std::vector<double> range_wavenumbers_rad_per_m;
  std::vector<double> azimuth_wavenumbers_rad_per_m;
  std::vector<double> propagation_wavenumbers_rad_per_m;
  std::vector<double> source_range_frequency_queries_hz;
  std::vector<double> stolt_shifts_hz;
};

bool EvaluateOmegaKStoltGeometry(const OmegaKGeometryConfig& config,
                                 OmegaKGeometryDiagnostics* diagnostics);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_GEOMETRY_H_
