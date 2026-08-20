/**
 * @file SarOmegaKSpectrumFrontEnd.h
 * @brief Omega-K 前端执行器:从 raw baseband history 产生 2D 波数谱(source_spectrum)。
 *
 * front-end = raw history 的 2D FFT(距离正向 + 方位正向) + bulk 参考函数 H_bulk。
 * 距离压缩被吸收进 H_bulk 的 K_r 依赖,不单独调用 matched filter(契约 §3.2)。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_SPECTRUM_FRONT_END_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_SPECTRUM_FRONT_END_H_

#include <cstdint>

#include "sar/imaging/SarOmegaKGeometry.h"
#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

/**
 * @brief front-end 执行状态。
 */
enum class OmegaKSpectrumFrontEndStatus { kSucceeded = 0, kRejected = 1 };
/**
 * @brief front-end 执行拒绝/失败原因。
 */
enum class OmegaKSpectrumFrontEndReason {
  kNone = 0,              /**< 无 */
  kInvalidRequestId = 1,  /**< 请求 ID 非法 */
  kInvalidConfig = 2,     /**< 配置非法 */
  kInvalidRawHistory = 3, /**< raw history 非法 */
  kGeometryFailure = 4,   /**< 几何评估失败 */
  kTransformFailure = 5,   /**< FFT/H_bulk 变换失败 */
};

/**
 * @brief front-end 执行请求。
 */
struct OmegaKSpectrumFrontEndRequest {
  std::uint64_t request_id{0U};   /**< 请求 ID */
  OmegaKGeometryConfig config;    /**< Omega-K 几何配置 */
  signal::ComplexMatrix raw_pulse_history; /**< 原始相位历史矩阵 */
};

/**
 * @brief front-end 执行结果。
 */
struct OmegaKSpectrumFrontEndResult {
  std::uint64_t request_id{0U};   /**< 关联的请求 ID */
  OmegaKSpectrumFrontEndStatus status{OmegaKSpectrumFrontEndStatus::kRejected}; /**< 执行状态 */
  OmegaKSpectrumFrontEndReason reason{OmegaKSpectrumFrontEndReason::kNone}; /**< 拒绝原因 */
  OmegaKGeometryDiagnostics geometry;             /**< 几何诊断 */
  signal::ComplexMatrix source_spectrum;          /**< 2D 波数源谱 */
};

/**
 * @brief 执行 Omega-K 前端：2D FFT + bulk 参考函数 H_bulk，产生波数源谱。
 * @param[in] request front-end 执行请求。
 * @return 执行结果（含源谱与几何诊断）。
 */
OmegaKSpectrumFrontEndResult ExecuteOmegaKSpectrumFrontEnd(
    const OmegaKSpectrumFrontEndRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_SPECTRUM_FRONT_END_H_
