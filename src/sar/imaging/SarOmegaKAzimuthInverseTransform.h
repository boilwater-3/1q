/**
 * @file SarOmegaKAzimuthInverseTransform.h
 * @brief Omega-K 数值方位逆变换执行器。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_AZIMUTH_INVERSE_TRANSFORM_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_AZIMUTH_INVERSE_TRANSFORM_H_

#include <cstdint>
#include <vector>

#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

/**
 * @brief 方位逆变换执行状态。
 */
enum class OmegaKAzimuthInverseStatus { kSucceeded = 0, kRejected = 1 };
/**
 * @brief 方位逆变换拒绝原因。
 */
enum class OmegaKAzimuthInverseReason {
  kNone = 0,                /**< 无 */
  kInvalidRequestId = 1,    /**< 请求 ID 非法 */
  kInvalidAxis = 2,         /**< 坐标轴非法 */
  kInvalidNormalization = 3,/**< 归一化系数非法 */
  kInvalidMatrix = 4,       /**< 矩阵非法 */
  kTransformFailure = 5,     /**< 变换失败 */
};

/**
 * @brief 方位逆变换请求。
 */
struct OmegaKAzimuthInverseRequest {
  std::uint64_t request_id{0U};               /**< 请求 ID */
  std::vector<double> absolute_slant_ranges_m; /**< 绝对斜距轴（m） */
  std::vector<double> output_azimuth_coordinates; /**< 输出方位坐标 */
  double additional_normalization{0.0};       /**< 附加归一化系数 */
  signal::ComplexMatrix compensated_intermediate; /**< 补偿后矩阵 */
};

/**
 * @brief 方位逆变换结果。
 */
struct OmegaKAzimuthInverseResult {
  std::uint64_t request_id{0U};               /**< 关联的请求 ID */
  OmegaKAzimuthInverseStatus status{OmegaKAzimuthInverseStatus::kRejected}; /**< 执行状态 */
  OmegaKAzimuthInverseReason reason{OmegaKAzimuthInverseReason::kNone}; /**< 拒绝原因 */
  std::vector<double> absolute_slant_ranges_m; /**< 绝对斜距轴（m） */
  std::vector<double> output_azimuth_coordinates; /**< 输出方位坐标 */
  double additional_normalization{0.0};       /**< 附加归一化系数 */
  signal::ComplexMatrix numerical_image_candidate; /**< 数值图像候选矩阵 */
};

/**
 * @brief 执行数值方位逆变换（方位向 IFFT）生成图像候选。
 * @param[in] request 方位逆变换请求。
 * @return 变换结果（含数值图像候选矩阵）。
 */
OmegaKAzimuthInverseResult ExecuteOmegaKAzimuthInverseTransform(
    const OmegaKAzimuthInverseRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_AZIMUTH_INVERSE_TRANSFORM_H_
