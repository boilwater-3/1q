// ============================================================================
// 【未进行设计需求，不再扩展 — DEPRECATED】
// 本文件不参与构建（见 src/sar/CMakeLists.txt 的 SAR_ENGINE_SOURCES 注释），
// 仅作为探索性参考保留。请勿新增依赖或据此实施。
// ============================================================================

﻿/**
 * @file SarOmegaKReferencePhaseCompensation.h
 * @brief Explicit Omega-K reference phase compensation executor.
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_REFERENCE_PHASE_COMPENSATION_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_REFERENCE_PHASE_COMPENSATION_H_

#include <cstdint>
#include <vector>

#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

enum class OmegaKPhaseApplicationSign {
  kUnspecified = 0,
  kPositive = 1,
  kNegative = 2,
};

enum class OmegaKPhaseCompensationStatus { kSucceeded = 0, kRejected = 1 };
enum class OmegaKPhaseCompensationReason {
  kNone = 0,
  kInvalidRequestId = 1,
  kInvalidSign = 2,
  kInvalidAxis = 3,
  kInvalidPhase = 4,
  kInvalidMatrix = 5,
};

struct OmegaKPhaseCompensationRequest {
  std::uint64_t request_id{0U};
  OmegaKPhaseApplicationSign sign{OmegaKPhaseApplicationSign::kUnspecified};
  std::vector<double> absolute_slant_ranges_m;
  std::vector<double> azimuth_coordinates;
  std::vector<double> range_phase_radians;
  signal::ComplexMatrix referenced_intermediate;
};

struct OmegaKPhaseCompensationResult {
  std::uint64_t request_id{0U};
  OmegaKPhaseCompensationStatus status{OmegaKPhaseCompensationStatus::kRejected};
  OmegaKPhaseCompensationReason reason{OmegaKPhaseCompensationReason::kNone};
  std::vector<double> absolute_slant_ranges_m;
  std::vector<double> azimuth_coordinates;
  signal::ComplexMatrix compensated_intermediate;
};

OmegaKPhaseCompensationResult ExecuteOmegaKReferencePhaseCompensation(
    const OmegaKPhaseCompensationRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_REFERENCE_PHASE_COMPENSATION_H_
