// ============================================================================
// 【未进行设计需求，不再扩展 — DEPRECATED】
// 本文件不参与构建（见 src/sar/CMakeLists.txt 的 SAR_ENGINE_SOURCES 注释），
// 仅作为探索性参考保留。请勿新增依赖或据此实施。
// ============================================================================

﻿/**
 * @file SarOmegaKStoltInterpolation.h
 * @brief Omega-K 内部复数 Stolt 线性插值。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_STOLT_INTERPOLATION_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_STOLT_INTERPOLATION_H_

#include <cstddef>
#include <vector>

#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

enum class StoltInterpolationStatus {
  kSucceeded = 0,
  kRejected = 1,
};

enum class StoltInterpolationRejectionReason {
  kNone = 0,
  kInvalidFrequencyAxis = 1,
  kInvalidSpectrum = 2,
  kInvalidQueries = 3,
  kOutOfSupportQuery = 4,
  kInterpolationFailure = 5,
};

struct StoltInterpolationRequest {
  std::vector<double> source_range_frequencies_hz;
  std::vector<double> target_range_frequencies_hz;
  signal::ComplexMatrix source_spectrum;
  signal::ComplexMatrix source_frequency_queries_hz;
};

struct StoltInterpolationDiagnostics {
  std::size_t row_count{0U};
  std::size_t range_bin_count{0U};
  std::size_t query_count{0U};
  std::size_t exact_hit_count{0U};
  std::size_t linear_interpolation_count{0U};
  std::size_t out_of_support_query_count{0U};
  double maximum_abs_shift_hz{0.0};
  double maximum_interpolation_interval_hz{0.0};
};

struct StoltInterpolationResult {
  StoltInterpolationStatus status{StoltInterpolationStatus::kRejected};
  StoltInterpolationRejectionReason reason{StoltInterpolationRejectionReason::kNone};
  StoltInterpolationDiagnostics diagnostics;
  signal::ComplexMatrix interpolated_spectrum;
};

StoltInterpolationResult InterpolateOmegaKStoltLinear(
    const StoltInterpolationRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_STOLT_INTERPOLATION_H_
