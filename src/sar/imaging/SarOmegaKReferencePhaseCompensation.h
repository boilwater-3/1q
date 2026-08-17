/**
 * @file SarOmegaKReferencePhaseCompensation.h
 * @brief Omega-K 显式参考相位补偿执行器。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_REFERENCE_PHASE_COMPENSATION_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_REFERENCE_PHASE_COMPENSATION_H_

#include <cstdint>
#include <vector>

#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

/**
 * @brief 相位施加符号约定。
 */
enum class OmegaKPhaseApplicationSign {
  kUnspecified = 0, /**< 未指定 */
  kPositive = 1,    /**< 施加正相位 */
  kNegative = 2,     /**< 施加负相位 */
};

/**
 * @brief 相位补偿执行状态。
 */
enum class OmegaKPhaseCompensationStatus { kSucceeded = 0, kRejected = 1 };
/**
 * @brief 相位补偿拒绝原因。
 */
enum class OmegaKPhaseCompensationReason {
  kNone = 0,             /**< 无 */
  kInvalidRequestId = 1, /**< 请求 ID 非法 */
  kInvalidSign = 2,      /**< 符号约定非法 */
  kInvalidAxis = 3,      /**< 坐标轴非法 */
  kInvalidPhase = 4,     /**< 相位向量非法 */
  kInvalidMatrix = 5,     /**< 矩阵非法 */
};

/**
 * @brief 参考相位补偿请求。
 */
struct OmegaKPhaseCompensationRequest {
  std::uint64_t request_id{0U};               /**< 请求 ID */
  OmegaKPhaseApplicationSign sign{OmegaKPhaseApplicationSign::kUnspecified}; /**< 相位施加符号 */
  std::vector<double> absolute_slant_ranges_m; /**< 绝对斜距轴（m） */
  std::vector<double> azimuth_coordinates;    /**< 方位坐标 */
  std::vector<double> range_phase_radians;    /**< 距离参考相位（rad） */
  signal::ComplexMatrix referenced_intermediate; /**< 参考后的中间矩阵 */
};

/**
 * @brief 参考相位补偿结果。
 */
struct OmegaKPhaseCompensationResult {
  std::uint64_t request_id{0U};               /**< 关联的请求 ID */
  OmegaKPhaseCompensationStatus status{OmegaKPhaseCompensationStatus::kRejected}; /**< 执行状态 */
  OmegaKPhaseCompensationReason reason{OmegaKPhaseCompensationReason::kNone}; /**< 拒绝原因 */
  std::vector<double> absolute_slant_ranges_m; /**< 绝对斜距轴（m） */
  std::vector<double> azimuth_coordinates;    /**< 方位坐标 */
  signal::ComplexMatrix compensated_intermediate; /**< 补偿后矩阵 */
};

/**
 * @brief 执行参考相位补偿，按斜距施加距离参考相位。
 * @param[in] request 相位补偿请求。
 * @return 补偿结果（含补偿后矩阵）。
 */
OmegaKPhaseCompensationResult ExecuteOmegaKReferencePhaseCompensation(
    const OmegaKPhaseCompensationRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_REFERENCE_PHASE_COMPENSATION_H_
