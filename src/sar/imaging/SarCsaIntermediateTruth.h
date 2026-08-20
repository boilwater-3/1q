// ============================================================================
// 【未进行设计需求，不再扩展 — DEPRECATED】
// 本文件不参与构建（见 src/sar/CMakeLists.txt 的 SAR_ENGINE_SOURCES 注释），
// 仅作为探索性参考保留。请勿新增依赖或据此实施。
// ============================================================================

/**
 * @file SarCsaIntermediateTruth.h
 * @brief CSA 显式操作列表与逐阶段中间域真值执行器。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_CSA_INTERMEDIATE_TRUTH_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_CSA_INTERMEDIATE_TRUTH_H_

#include <cstddef>
#include <string>
#include <vector>

#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

/**
 * @brief CSA 真值流水线单步操作类型。
 */
enum class CsaTruthOperation {
  kForwardRangeFft = 0,    /**< 距离向正向 FFT */
  kInverseRangeFft = 1,    /**< 距离向逆向 FFT */
  kForwardAzimuthFft = 2,  /**< 方位向正向 FFT */
  kInverseAzimuthFft = 3,  /**< 方位向逆向 FFT */
  kMultiplyPhaseKernel = 4, /**< 乘以相位核 */
};

/**
 * @brief CSA 真值执行状态。
 */
enum class CsaTruthExecutionStatus {
  kSucceeded = 0, /**< 执行成功 */
  kRejected = 1,   /**< 请求被拒绝 */
};

/**
 * @brief CSA 真值执行拒绝原因。
 */
enum class CsaTruthRejectionReason {
  kNone = 0,                 /**< 无 */
  kInvalidReferenceId = 1,   /**< 参考 ID 非法 */
  kInvalidInput = 2,         /**< 输入非法 */
  kEmptyStages = 3,          /**< 阶段列表为空 */
  kInvalidStage = 4,         /**< 阶段非法 */
  kInvalidPhaseKernel = 5,   /**< 相位核非法 */
  kOperationFailure = 6,     /**< 操作执行失败 */
  kTruthMismatch = 7,         /**< 与期望输出不符 */
};

/**
 * @brief CSA 真值流水线单阶段定义。
 */
struct CsaTruthStage {
  std::string stage_id;        /**< 阶段标识 */
  CsaTruthOperation operation{CsaTruthOperation::kForwardRangeFft}; /**< 阶段操作 */
  std::string phase_kernel_id; /**< 相位核标识 */
  signal::ComplexMatrix phase_kernel;   /**< 相位核矩阵 */
  signal::ComplexMatrix expected_output; /**< 期望输出 */
  double maximum_abs_error_tolerance{0.0}; /**< 最大绝对误差容差 */
  double unit_energy_nrms_tolerance{0.0}; /**< 单位能量 NRMS 容差 */
};

/**
 * @brief CSA 中间域真值参考（输入 + 阶段序列）。
 */
struct CsaIntermediateTruthReference {
  std::string reference_id;    /**< 参考 ID */
  signal::ComplexMatrix input; /**< 输入矩阵 */
  std::vector<CsaTruthStage> stages; /**< 阶段序列 */
};

/**
 * @brief CSA 真值单阶段诊断。
 */
struct CsaTruthStageDiagnostics {
  std::string stage_id;        /**< 阶段标识 */
  CsaTruthOperation operation{CsaTruthOperation::kForwardRangeFft}; /**< 阶段操作 */
  double maximum_abs_error{0.0}; /**< 最大绝对误差 */
  double unit_energy_nrms{0.0}; /**< 单位能量 NRMS */
  bool passed{false};          /**< 是否通过容差 */
};

/**
 * @brief CSA 中间域真值执行结果。
 */
struct CsaIntermediateTruthResult {
  CsaTruthExecutionStatus status{CsaTruthExecutionStatus::kRejected}; /**< 执行状态 */
  CsaTruthRejectionReason reason{CsaTruthRejectionReason::kNone}; /**< 拒绝原因 */
  std::size_t first_failed_stage_index{static_cast<std::size_t>(-1)}; /**< 首个失败阶段索引 */
  std::vector<CsaTruthStageDiagnostics> stage_diagnostics; /**< 各阶段诊断 */
  signal::ComplexMatrix final_output; /**< 最终输出 */
};

/**
 * @brief 逐阶段执行 CSA 中间域真值参考并与期望输出比对。
 * @param[in] reference 真值参考（输入 + 阶段）。
 * @return 执行结果（含各阶段诊断与最终输出）。
 */
CsaIntermediateTruthResult ExecuteCsaIntermediateTruthReference(
    const CsaIntermediateTruthReference& reference);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_CSA_INTERMEDIATE_TRUTH_H_
