/**
 * @file SarPhaseReference.h
 * @brief SAR 内部相位重参考与全局常数相位估计工具。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_PHASE_REFERENCE_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_PHASE_REFERENCE_H_

#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

/**
 * @brief 相位参考模式。
 */
enum class PhaseReferenceMode {
  kNative = 0,          /**< 保持原生相位参考 */
  kCenterBroadside = 1,  /**< 重参考至中心正侧视 */
};

/**
 * @brief 相位重参考配置。
 */
struct PhaseReferenceConfig {
  PhaseReferenceMode mode{PhaseReferenceMode::kCenterBroadside}; /**< 参考模式 */
  double carrier_frequency_hz{0.0};  /**< 载频（Hz） */
  double prf_hz{0.0};               /**< PRF（Hz） */
  double platform_velocity_mps{0.0}; /**< 平台速度（m/s） */
  double range_bin_spacing_m{0.0};   /**< 距离 bin 间距（m） */
};

/**
 * @brief 相位重参考诊断。
 */
struct PhaseReferenceDiagnostics {
  bool applied{false};                            /**< 是否施加了重参考 */
  PhaseReferenceMode mode{PhaseReferenceMode::kNative}; /**< 使用的参考模式 */
  double min_phase_rad{0.0};                      /**< 最小相位（rad） */
  double max_phase_rad{0.0};                      /**< 最大相位（rad） */
};

/**
 * @brief 判断给定模式是否需要相位重参考。
 * @param[in] mode 相位参考模式。
 * @param[out] needs_reference 是否需要重参考。
 * @return 成功返回 true，失败返回 false。
 */
bool NeedsPhaseReference(PhaseReferenceMode mode, bool* needs_reference);

/**
 * @brief 对复图像施加中心正侧视相位重参考。
 * @param[in] config 相位重参考配置。
 * @param[out] image 待重参考的复图像。
 * @param[out] diagnostics 重参考诊断。
 * @return 成功返回 true，失败返回 false。
 */
bool ApplyBroadsideCenterPhaseReference(const PhaseReferenceConfig& config,
                                        signal::ComplexMatrix* image,
                                        PhaseReferenceDiagnostics* diagnostics);

/**
 * @brief 估计候选图像相对参考图像的全局相位偏移。
 * @param[in] reference 参考复图像。
 * @param[in] candidate 候选复图像。
 * @param[out] phase_offset_rad 全局相位偏移（rad）。
 * @return 成功返回 true，失败返回 false。
 */
bool EstimateGlobalPhaseOffset(const signal::ComplexMatrix& reference,
                               const signal::ComplexMatrix& candidate,
                               double* phase_offset_rad);

/**
 * @brief 对复图像施加全局相位偏移。
 * @param[in] phase_offset_rad 全局相位偏移（rad）。
 * @param[out] image 待调整的复图像。
 * @return 成功返回 true，失败返回 false。
 */
bool ApplyGlobalPhaseOffset(double phase_offset_rad, signal::ComplexMatrix* image);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_PHASE_REFERENCE_H_
