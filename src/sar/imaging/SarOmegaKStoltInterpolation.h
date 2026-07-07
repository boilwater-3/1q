/**
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

/**
 * @brief Stolt 插值执行状态。
 */
enum class StoltInterpolationStatus {
  kSucceeded = 0, /**< 成功 */
  kRejected = 1,   /**< 被拒绝 */
};

/**
 * @brief Stolt 插值拒绝原因。
 */
enum class StoltInterpolationRejectionReason {
  kNone = 0,                 /**< 无 */
  kInvalidFrequencyAxis = 1, /**< 频率轴非法 */
  kInvalidSpectrum = 2,      /**< 频谱矩阵非法 */
  kInvalidQueries = 3,       /**< 查询非法 */
  kOutOfSupportQuery = 4,    /**< 查询越界 */
  kInterpolationFailure = 5,  /**< 插值失败 */
};

/**
 * @brief Stolt 线性插值请求。
 */
struct StoltInterpolationRequest {
  std::vector<double> source_range_frequencies_hz; /**< 源距离频率轴（Hz） */
  std::vector<double> target_range_frequencies_hz; /**< 目标距离频率轴（Hz） */
  signal::ComplexMatrix source_spectrum;           /**< 源复数频谱 */
  signal::ComplexMatrix source_frequency_queries_hz; /**< 各行源频率查询（Hz） */
};

/**
 * @brief Stolt 插值诊断。
 */
struct StoltInterpolationDiagnostics {
  std::size_t row_count{0U};                    /**< 行数 */
  std::size_t range_bin_count{0U};              /**< 距离 bin 数 */
  std::size_t query_count{0U};                  /**< 查询总数 */
  std::size_t exact_hit_count{0U};              /**< 精确命中数 */
  std::size_t linear_interpolation_count{0U};   /**< 线性插值数 */
  std::size_t out_of_support_query_count{0U};   /**< 越界查询数 */
  double maximum_abs_shift_hz{0.0};             /**< 最大偏移绝对值（Hz） */
  double maximum_interpolation_interval_hz{0.0};/**< 最大插值间隔（Hz） */
};

/**
 * @brief Stolt 插值结果。
 */
struct StoltInterpolationResult {
  StoltInterpolationStatus status{StoltInterpolationStatus::kRejected}; /**< 执行状态 */
  StoltInterpolationRejectionReason reason{StoltInterpolationRejectionReason::kNone}; /**< 拒绝原因 */
  StoltInterpolationDiagnostics diagnostics;     /**< 插值诊断 */
  signal::ComplexMatrix interpolated_spectrum;   /**< 插值后频谱 */
};

/**
 * @brief 执行 Omega-K 复数 Stolt 线性插值。
 * @param[in] request 插值请求。
 * @return 插值结果（含诊断与插值后频谱）。
 */
StoltInterpolationResult InterpolateOmegaKStoltLinear(
    const StoltInterpolationRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_STOLT_INTERPOLATION_H_
