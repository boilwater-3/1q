/**
 * @file SarOmegaKPointTargetAcceptance.h
 * @brief Omega-K 点目标候选的独立验收评估器。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_POINT_TARGET_ACCEPTANCE_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_POINT_TARGET_ACCEPTANCE_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

/**
 * @brief 点目标验收状态。
 */
enum class OmegaKPointTargetAcceptanceStatus { kPassed = 0, kFailed = 1, kRejected = 2 };
/**
 * @brief 点目标验收拒绝原因。
 */
enum class OmegaKPointTargetAcceptanceReason {
  kNone = 0,                 /**< 无 */
  kInvalidRequest = 1,       /**< 请求非法 */
  kTruthNotIndependent = 2,  /**< 真值非独立生成 */
  kOutsideCommonSupport = 3, /**< 超出共同支持域 */
  kInvalidCandidate = 4,      /**< 候选图像非法 */
};

/**
 * @brief 独立生成的点目标真值。
 */
struct OmegaKPointTargetTruth {
  bool independently_generated{false};    /**< 是否独立生成 */
  bool inside_common_support{false};      /**< 是否在共同支持域内 */
  double absolute_slant_range_m{0.0};     /**< 绝对斜距（m） */
  double azimuth_coordinate{0.0};         /**< 方位坐标 */
  double peak_phase_rad{0.0};             /**< 峰值相位（rad） */
  double peak_magnitude{0.0};             /**< 峰值幅度 */
  std::size_t range_mainlobe_half_width{0U};   /**< 距离主瓣半宽 */
  std::size_t azimuth_mainlobe_half_width{0U}; /**< 方位主瓣半宽 */
};

/**
 * @brief 点目标验收容差。
 */
struct OmegaKPointTargetTolerances {
  double maximum_range_error_m{0.0};            /**< 最大距离误差（m） */
  double maximum_azimuth_error{0.0};            /**< 最大方位误差 */
  double maximum_abs_phase_error_rad{0.0};      /**< 最大相位误差绝对值（rad） */
  double maximum_relative_magnitude_error{0.0}; /**< 最大相对幅度误差 */
  double maximum_range_pslr_db{0.0};            /**< 最大距离 PSLR（dB） */
  double maximum_azimuth_pslr_db{0.0};          /**< 最大方位 PSLR（dB） */
  double maximum_range_islr_db{0.0};            /**< 最大距离 ISLR（dB） */
  double maximum_azimuth_islr_db{0.0};          /**< 最大方位 ISLR（dB） */
};

/**
 * @brief 点目标验收请求。
 */
struct OmegaKPointTargetAcceptanceRequest {
  std::uint64_t request_id{0U};               /**< 请求 ID */
  std::vector<double> absolute_slant_ranges_m; /**< 绝对斜距轴（m） */
  std::vector<double> azimuth_coordinates;    /**< 方位坐标 */
  signal::ComplexMatrix numerical_image_candidate; /**< 数值图像候选 */
  OmegaKPointTargetTruth truth;               /**< 点目标真值 */
  OmegaKPointTargetTolerances tolerances;     /**< 验收容差 */
};

/**
 * @brief 点目标验收结果。
 */
struct OmegaKPointTargetAcceptanceResult {
  std::uint64_t request_id{0U};               /**< 关联的请求 ID */
  OmegaKPointTargetAcceptanceStatus status{OmegaKPointTargetAcceptanceStatus::kRejected}; /**< 验收状态 */
  OmegaKPointTargetAcceptanceReason reason{OmegaKPointTargetAcceptanceReason::kNone}; /**< 拒绝原因 */
  std::size_t peak_row{0U};                   /**< 峰值行索引 */
  std::size_t peak_col{0U};                   /**< 峰值列索引 */
  double range_error_m{0.0};                  /**< 距离误差（m） */
  double azimuth_error{0.0};                  /**< 方位误差 */
  double wrapped_phase_error_rad{0.0};        /**< 缠绕相位误差（rad） */
  double relative_magnitude_error{0.0};       /**< 相对幅度误差 */
  double range_pslr_db{0.0};                  /**< 距离 PSLR（dB） */
  double azimuth_pslr_db{0.0};                /**< 方位 PSLR（dB） */
  double range_islr_db{0.0};                  /**< 距离 ISLR（dB） */
  double azimuth_islr_db{0.0};                /**< 方位 ISLR（dB） */
};

/**
 * @brief 评估 Omega-K 点目标候选是否通过独立验收。
 * @param[in] request 验收请求。
 * @return 验收结果（含各项误差与通过判定）。
 */
OmegaKPointTargetAcceptanceResult EvaluateOmegaKPointTargetCandidate(
    const OmegaKPointTargetAcceptanceRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_POINT_TARGET_ACCEPTANCE_H_
