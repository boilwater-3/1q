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

struct OmegaKCommonSupportDiagnostics {
  bool valid{false};
  bool usable_for_interpolation{false};
  std::size_t azimuth_row_count{0U};
  std::size_t target_range_bin_count{0U};
  std::size_t total_query_count{0U};
  std::vector<std::size_t> out_of_support_query_count_per_column;
  std::vector<bool> common_valid_column_mask;
  std::size_t common_valid_column_count{0U};
  std::size_t discarded_column_count{0U};
  double common_valid_ratio{0.0};
  double maximum_abs_stolt_shift_hz{0.0};
  std::vector<std::size_t> largest_contiguous_original_column_indices;
  std::size_t largest_contiguous_column_count{0U};
  double largest_contiguous_minimum_frequency_hz{0.0};
  double largest_contiguous_maximum_frequency_hz{0.0};
};

bool DiagnoseOmegaKCommonStoltSupport(
    const OmegaKGeometryDiagnostics& geometry,
    OmegaKCommonSupportDiagnostics* diagnostics);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_COMMON_SUPPORT_H_
