/**
 * @file SarOmegaKCommonSupport.h
 * @brief Omega-K 全方位共同有效 Stolt 支持窗口诊断。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_COMMON_SUPPORT_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_COMMON_SUPPORT_H_

#include <cstddef>
#include <vector>

#include "sar/imaging/SarOmegaKGeometry.h"

namespace sar {
namespace imaging {

/**
 * @brief Omega-K 全方位共同有效 Stolt 支持窗口诊断。
 */
struct OmegaKCommonSupportDiagnostics {
  bool valid{false};                                          /**< 诊断是否有效 */
  bool usable_for_interpolation{false};                       /**< 是否可用于插值收缩 */
  std::size_t azimuth_row_count{0U};                          /**< 方位行数 */
  std::size_t target_range_bin_count{0U};                     /**< 目标距离 bin 数 */
  std::size_t total_query_count{0U};                          /**< 总查询数 */
  std::vector<std::size_t> out_of_support_query_count_per_column; /**< 各列越界查询数 */
  std::vector<bool> common_valid_column_mask;                 /**< 共同有效列掩码 */
  std::size_t common_valid_column_count{0U};                  /**< 共同有效列数 */
  std::size_t discarded_column_count{0U};                     /**< 丢弃列数 */
  double common_valid_ratio{0.0};                             /**< 共同有效比例 */
  double maximum_abs_stolt_shift_hz{0.0};                     /**< 最大 Stolt 偏移绝对值（Hz） */
  std::vector<std::size_t> largest_contiguous_original_column_indices; /**< 最大连续列索引集 */
  std::size_t largest_contiguous_column_count{0U};            /**< 最大连续列数 */
  double largest_contiguous_minimum_frequency_hz{0.0};        /**< 最大连续块频率下限（Hz） */
  double largest_contiguous_maximum_frequency_hz{0.0};        /**< 最大连续块频率上限（Hz） */
};

/**
 * @brief 诊断 Omega-K 共同有效 Stolt 支持窗口。
 * @param[in] geometry 已评估的 Omega-K 几何诊断。
 * @param[out] diagnostics 共同支持诊断。
 * @return 成功返回 true，失败返回 false。
 */
bool DiagnoseOmegaKCommonStoltSupport(
    const OmegaKGeometryDiagnostics& geometry,
    OmegaKCommonSupportDiagnostics* diagnostics);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_COMMON_SUPPORT_H_
