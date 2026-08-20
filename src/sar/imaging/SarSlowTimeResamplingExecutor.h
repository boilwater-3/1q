/**
 * @file SarSlowTimeResamplingExecutor.h
 * @brief SAR 内部慢时间重采样显式请求执行器。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_SLOW_TIME_RESAMPLING_EXECUTOR_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_SLOW_TIME_RESAMPLING_EXECUTOR_H_

#include <cstdint>
#include <vector>

#include "sar/imaging/SarSlowTimeResampling.h"

namespace sar {
namespace imaging {

/**
 * @brief 慢时间重采样执行状态。
 */
enum class SlowTimeResamplingExecutionStatus {
  kSucceeded = 0, /**< 重采样成功 */
  kRejected = 1,   /**< 请求被拒绝 */
};

/**
 * @brief 慢时间重采样拒绝原因。
 */
enum class SlowTimeResamplingRejectionReason {
  kNone = 0,                   /**< 无拒绝 */
  kInvalidRequestId = 1,       /**< 请求 ID 非法 */
  kInvalidExpectedInterval = 2,/**< 期望间隔非法 */
  kInvalidTimeAxis = 3,        /**< 时间轴非法 */
  kInvalidRawHistory = 4,      /**< raw history 非法 */
  kMissingPulseGap = 5,        /**< 存在缺脉冲间隙 */
  kResamplingFailure = 6,       /**< 重采样失败 */
};

/**
 * @brief 慢时间重采样请求。
 */
struct SlowTimeResamplingRequest {
  std::uint64_t request_id{0U};           /**< 请求 ID */
  std::vector<double> explicit_times_s;   /**< 显式慢时间轴（s） */
  double expected_interval_s{0.0};        /**< 期望均匀间隔（s） */
  signal::ComplexMatrix raw_history;      /**< 待重采样的 raw history */
};

/**
 * @brief 慢时间重采样执行结果。
 */
struct SlowTimeResamplingExecutionResult {
  std::uint64_t request_id{0U};           /**< 关联的请求 ID */
  SlowTimeResamplingExecutionStatus status{SlowTimeResamplingExecutionStatus::kRejected}; /**< 执行状态 */
  SlowTimeResamplingRejectionReason reason{SlowTimeResamplingRejectionReason::kNone}; /**< 拒绝原因 */
  SlowTimeGapDiagnostics gap_diagnostics; /**< 间隙诊断 */
  SlowTimeResamplingDiagnostics resampling_diagnostics; /**< 重采样诊断 */
  signal::ComplexMatrix resampled_raw_history; /**< 重采样后的 raw history */
};

/**
 * @brief 执行单条慢时间重采样请求并返回结果。
 * @param[in] request 重采样请求。
 * @return 执行结果（含状态、拒绝原因与重采样诊断）。
 */
SlowTimeResamplingExecutionResult ExecuteSlowTimeResamplingRequest(
    const SlowTimeResamplingRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_SLOW_TIME_RESAMPLING_EXECUTOR_H_
