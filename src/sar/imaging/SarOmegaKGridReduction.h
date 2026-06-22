// ============================================================================
// 【未进行设计需求，不再扩展 — DEPRECATED】
// 本文件不参与构建（见 src/sar/CMakeLists.txt 的 SAR_ENGINE_SOURCES 注释），
// 仅作为探索性参考保留。请勿新增依赖或据此实施。
// ============================================================================

﻿/**
 * @file SarOmegaKGridReduction.h
 * @brief Omega-K 显式共同支持目标网格收缩请求执行器。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_GRID_REDUCTION_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_GRID_REDUCTION_H_

#include <cstdint>
#include <vector>

#include "sar/imaging/SarOmegaKCommonSupport.h"
#include "sar/imaging/SarOmegaKStoltInterpolation.h"

namespace sar {
namespace imaging {

enum class OmegaKGridReductionStatus { kSucceeded = 0, kRejected = 1 };
enum class OmegaKGridReductionReason {
  kNone = 0,
  kInvalidRequestId = 1,
  kInvalidGeometry = 2,
  kInvalidCommonSupport = 3,
  kInvalidSourceSpectrum = 4,
  kInterpolationFailure = 5,
};

struct OmegaKGridReductionRequest {
  std::uint64_t request_id{0U};
  OmegaKGeometryDiagnostics geometry;
  OmegaKCommonSupportDiagnostics common_support;
  signal::ComplexMatrix source_spectrum;
};

struct OmegaKGridReductionResult {
  std::uint64_t request_id{0U};
  OmegaKGridReductionStatus status{OmegaKGridReductionStatus::kRejected};
  OmegaKGridReductionReason reason{OmegaKGridReductionReason::kNone};
  std::vector<double> reduced_target_frequencies_hz;
  std::vector<std::size_t> original_column_indices;
  StoltInterpolationDiagnostics interpolation_diagnostics;
  signal::ComplexMatrix reduced_spectrum;
};

OmegaKGridReductionResult ExecuteOmegaKExplicitGridReduction(
    const OmegaKGridReductionRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_GRID_REDUCTION_H_
