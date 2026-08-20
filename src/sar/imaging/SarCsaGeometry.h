// ============================================================================
// 【未进行设计需求，不再扩展 — DEPRECATED】
// 本文件不参与构建（见 src/sar/CMakeLists.txt 的 SAR_ENGINE_SOURCES 注释），
// 仅作为探索性参考保留。请勿新增依赖或据此实施。
// ============================================================================

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
  std::size_t range_sample_count{0U};        /**< 距离向采样点数 */
  std::size_t azimuth_pulse_count{0U};       /**< 方位向脉冲数 */
  double sample_rate_hz{0.0};                /**< 距离向采样率（Hz） */
  double prf_hz{0.0};                        /**< 脉冲重复频率 PRF（Hz） */
  double carrier_frequency_hz{0.0};          /**< 载频（Hz） */
  double platform_velocity_mps{0.0};         /**< 平台速度（m/s） */
  double reference_range_m{0.0};             /**< 参考斜距（m） */
};

/**
 * @brief CSA 频率几何诊断结果。
 *
 * 包含距离/方位频率轴、波长、Doppler 因子、chirp scaling 因子及有效域判定统计量。
 */
struct CsaGeometryDiagnostics {
  bool valid{false};                                  /**< 几何是否有效 */
  std::size_t range_frequency_bin_count{0U};          /**< 距离频率 bin 数 */
  std::size_t azimuth_frequency_bin_count{0U};        /**< 方位频率 bin 数 */
  double range_frequency_spacing_hz{0.0};             /**< 距离频率间隔（Hz） */
  double azimuth_frequency_spacing_hz{0.0};           /**< 方位频率间隔（Hz） */
  double wavelength_m{0.0};                           /**< 波长（m） */
  double minimum_doppler_factor{0.0};                 /**< 最小 Doppler 因子 */
  double maximum_abs_chirp_scaling_factor{0.0};       /**< 最大 chirp scaling 因子绝对值 */
  std::size_t invalid_doppler_bin_count{0U};          /**< 无效 Doppler bin 数 */
  double valid_doppler_limit_hz{0.0};                 /**< 有效 Doppler 限（Hz） */
  double maximum_abs_azimuth_frequency_hz{0.0};       /**< 最大方位频率绝对值（Hz） */
  double doppler_validity_margin_hz{0.0};             /**< Doppler 有效域余量（Hz） */
  std::vector<double> range_frequencies_hz;           /**< 距离频率轴（Hz） */
  std::vector<double> azimuth_frequencies_hz;         /**< 方位频率轴（Hz） */
  std::vector<double> doppler_factors;                /**< Doppler 因子序列 */
  std::vector<double> chirp_scaling_factors;          /**< chirp scaling 因子序列 */
};

/**
 * @brief 评估 CSA 频率几何有效域。
 * @param[in] config CSA 频率几何配置。
 * @param[out] diagnostics 诊断结果。
 * @return 结构非法返回 false；结构合法返回 true，此时 diagnostics.valid=false 表示存在
 *         超出 CSA 批准域的 Doppler 频率 bin。
 */
bool EvaluateCsaFrequencyGeometry(const CsaGeometryConfig& config,
                                  CsaGeometryDiagnostics* diagnostics);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_CSA_GEOMETRY_H_
