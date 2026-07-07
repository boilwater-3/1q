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

/**
 * @brief Omega-K 几何配置（条带/聚束通用物理参数）。
 */
struct OmegaKGeometryConfig {
  std::size_t range_sample_count{0U};   /**< 距离向采样点数 */
  std::size_t azimuth_pulse_count{0U};  /**< 方位向脉冲数 */
  double sample_rate_hz{0.0};           /**< 距离向采样率（Hz） */
  double prf_hz{0.0};                  /**< 脉冲重复频率 PRF（Hz） */
  double carrier_frequency_hz{0.0};     /**< 载频（Hz） */
  double platform_velocity_mps{0.0};    /**< 平台速度（m/s） */
  double reference_range_m{0.0};        /**< 参考斜距（m） */
};

/**
 * @brief Omega-K 波数与 Stolt 查询几何诊断。
 */
struct OmegaKGeometryDiagnostics {
  bool valid{false};                                            /**< 几何是否有效 */
  std::size_t range_frequency_bin_count{0U};                    /**< 距离频率 bin 数 */
  std::size_t azimuth_frequency_bin_count{0U};                  /**< 方位频率 bin 数 */
  double range_frequency_spacing_hz{0.0};                       /**< 距离频率间隔（Hz） */
  double azimuth_frequency_spacing_hz{0.0};                     /**< 方位频率间隔（Hz） */
  double wavelength_m{0.0};                                     /**< 波长（m） */
  double minimum_range_wavenumber_rad_per_m{0.0};               /**< 最小距离波数（rad/m） */
  double maximum_range_wavenumber_rad_per_m{0.0};               /**< 最大距离波数（rad/m） */
  double maximum_abs_azimuth_wavenumber_rad_per_m{0.0};         /**< 最大方位波数绝对值（rad/m） */
  double minimum_valid_propagation_wavenumber_rad_per_m{0.0};   /**< 最小有效传播波数（rad/m） */
  double maximum_valid_propagation_wavenumber_rad_per_m{0.0};   /**< 最大有效传播波数（rad/m） */
  std::size_t invalid_dispersion_point_count{0U};               /**< 无效色散点数 */
  std::size_t out_of_support_stolt_query_count{0U};             /**< 越界 Stolt 查询数 */
  double minimum_source_range_frequency_hz{0.0};                /**< 源距离频率下限（Hz） */
  double maximum_source_range_frequency_hz{0.0};                /**< 源距离频率上限（Hz） */
  double maximum_abs_stolt_shift_hz{0.0};                       /**< 最大 Stolt 偏移绝对值（Hz） */
  std::vector<double> range_frequencies_hz;                     /**< 距离频率轴（Hz） */
  std::vector<double> azimuth_frequencies_hz;                   /**< 方位频率轴（Hz） */
  std::vector<double> range_wavenumbers_rad_per_m;              /**< 距离波数（rad/m） */
  std::vector<double> azimuth_wavenumbers_rad_per_m;            /**< 方位波数（rad/m） */
  std::vector<double> propagation_wavenumbers_rad_per_m;        /**< 传播波数（rad/m） */
  std::vector<double> source_range_frequency_queries_hz;        /**< 源距离频率查询（Hz） */
  std::vector<double> stolt_shifts_hz;                          /**< Stolt 偏移（Hz） */
};

/**
 * @brief 评估 Omega-K 波数与 Stolt 查询几何。
 * @param[in] config 几何配置。
 * @param[out] diagnostics 几何诊断。
 * @return 成功返回 true，失败返回 false。
 */
bool EvaluateOmegaKStoltGeometry(const OmegaKGeometryConfig& config,
                                 OmegaKGeometryDiagnostics* diagnostics);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_GEOMETRY_H_
