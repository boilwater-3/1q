/**
 * @file SarOmegaKReferenceMapping.h
 * @brief Omega-K 参考元数据与绝对斜距映射执行器。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_REFERENCE_MAPPING_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_REFERENCE_MAPPING_H_

#include <cstdint>
#include <vector>

#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

/**
 * @brief 相对延迟符号约定。
 */
enum class OmegaKDelaySign {
  kUnspecified = 0,            /**< 未指定 */
  kPositiveIncreasesRange = 1, /**< 正延迟对应斜距增大 */
  kPositiveDecreasesRange = 2,  /**< 正延迟对应斜距减小 */
};

/**
 * @brief 参考相位符号约定。
 */
enum class OmegaKReferencePhaseSign {
  kUnspecified = 0, /**< 未指定 */
  kPositive = 1,    /**< 正相位 */
  kNegative = 2,    /**< 负相位 */
  kIdentity = 3,     /**< 恒等（无相位） */
};

/**
 * @brief 参考映射执行状态。
 */
enum class OmegaKReferenceMappingStatus { kSucceeded = 0, kRejected = 1 };
/**
 * @brief 参考映射拒绝原因。
 */
enum class OmegaKReferenceMappingReason {
  kNone = 0,                    /**< 无 */
  kInvalidRequestId = 1,        /**< 请求 ID 非法 */
  kInvalidPhysicalMetadata = 2, /**< 物理元数据非法 */
  kInvalidConvention = 3,       /**< 符号约定非法 */
  kInvalidAxis = 4,             /**< 坐标轴非法 */
  kInvalidMatrix = 5,           /**< 矩阵非法 */
  kInvalidAbsoluteRange = 6,     /**< 绝对斜距非法 */
};

/**
 * @brief 参考映射请求。
 */
struct OmegaKReferenceMappingRequest {
  std::uint64_t request_id{0U};               /**< 请求 ID */
  double propagation_speed_mps{0.0};          /**< 传播速度（m/s） */
  double reference_slant_range_m{0.0};        /**< 参考斜距（m） */
  OmegaKDelaySign delay_sign{OmegaKDelaySign::kUnspecified}; /**< 延迟符号约定 */
  OmegaKReferencePhaseSign reference_phase_sign{OmegaKReferencePhaseSign::kUnspecified}; /**< 相位符号约定 */
  double transform_normalization{0.0};        /**< 变换归一化系数 */
  std::vector<double> relative_delays_s;      /**< 相对延迟轴（s） */
  std::vector<double> azimuth_coordinates;    /**< 方位坐标 */
  signal::ComplexMatrix relative_delay_domain; /**< 相对延迟域矩阵 */
};

/**
 * @brief 参考映射结果。
 */
struct OmegaKReferenceMappingResult {
  std::uint64_t request_id{0U};               /**< 关联的请求 ID */
  OmegaKReferenceMappingStatus status{OmegaKReferenceMappingStatus::kRejected}; /**< 执行状态 */
  OmegaKReferenceMappingReason reason{OmegaKReferenceMappingReason::kNone}; /**< 拒绝原因 */
  std::vector<double> absolute_slant_ranges_m; /**< 绝对斜距轴（m） */
  std::vector<double> azimuth_coordinates;    /**< 方位坐标 */
  signal::ComplexMatrix referenced_intermediate; /**< 参考后的中间矩阵 */
};

/**
 * @brief 执行参考映射，由相对延迟推导绝对斜距并施加参考。
 * @param[in] request 参考映射请求。
 * @return 映射结果（含绝对斜距轴与参考后矩阵）。
 */
OmegaKReferenceMappingResult ExecuteOmegaKReferenceMapping(
    const OmegaKReferenceMappingRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_REFERENCE_MAPPING_H_
