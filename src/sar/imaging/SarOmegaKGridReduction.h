/**
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

/**
 * @brief 网格收缩执行状态。
 */
enum class OmegaKGridReductionStatus { kSucceeded = 0, kRejected = 1 };
/**
 * @brief 网格收缩拒绝原因。
 */
enum class OmegaKGridReductionReason {
  kNone = 0,                  /**< 无 */
  kInvalidRequestId = 1,      /**< 请求 ID 非法 */
  kInvalidGeometry = 2,       /**< 几何诊断非法 */
  kInvalidCommonSupport = 3,  /**< 共同支持诊断非法 */
  kInvalidSourceSpectrum = 4, /**< 源谱非法 */
  kInterpolationFailure = 5,   /**< Stolt 插值失败 */
};

/**
 * @brief 显式共同支持目标网格收缩请求。
 */
struct OmegaKGridReductionRequest {
  std::uint64_t request_id{0U};               /**< 请求 ID */
  OmegaKGeometryDiagnostics geometry;          /**< 几何诊断 */
  OmegaKCommonSupportDiagnostics common_support; /**< 共同支持诊断 */
  signal::ComplexMatrix source_spectrum;       /**< 源波数谱 */
};

/**
 * @brief 网格收缩结果。
 */
struct OmegaKGridReductionResult {
  std::uint64_t request_id{0U};               /**< 关联的请求 ID */
  OmegaKGridReductionStatus status{OmegaKGridReductionStatus::kRejected}; /**< 执行状态 */
  OmegaKGridReductionReason reason{OmegaKGridReductionReason::kNone}; /**< 拒绝原因 */
  std::vector<double> reduced_target_frequencies_hz; /**< 收缩后目标频率轴（Hz） */
  std::vector<std::size_t> original_column_indices;  /**< 保留的原列索引 */
  StoltInterpolationDiagnostics interpolation_diagnostics; /**< Stolt 插值诊断 */
  signal::ComplexMatrix reduced_spectrum;     /**< 收缩后频谱 */
};

/**
 * @brief 基于共同支持对源谱执行显式目标网格收缩。
 * @param[in] request 网格收缩请求。
 * @return 收缩结果（含收缩频谱与保留列索引）。
 */
OmegaKGridReductionResult ExecuteOmegaKExplicitGridReduction(
    const OmegaKGridReductionRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_GRID_REDUCTION_H_
