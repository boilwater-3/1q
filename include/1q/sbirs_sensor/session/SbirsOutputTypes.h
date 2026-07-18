/**
 * @file SbirsOutputTypes.h
 * @brief 定义 SBIRS-inspired 原生观测输出与归属类型。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_OUTPUT_TYPES_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_OUTPUT_TYPES_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"

namespace sbirs_sensor {
namespace output {

/** @brief 原生观测阶段：对应状态机输出的 WFOV 搜索、NFOV 首次捕获或 NFOV 真值辅助跟踪。 */
enum class ONEQ_API SbirsObservationStage {
  kWideFieldSearch = 0,    /**< WFOV 宽视场搜索 */
  kNarrowFieldAcquisition, /**< NFOV 首次捕获 */
  kNarrowFieldTrack        /**< NFOV 真值辅助跟踪 */
};

/**
 * @brief 单条 SBIRS-inspired 原生观测检测记录，属于 1q 仿真传感器主输出层。
 * @note 该结构不含目标真值或仿真归属；归属信息进入 `SbirsDetectionAttributionRecord`。
 */
struct ONEQ_API SbirsDetectionRecord {
  std::uint64_t detection_id{0U};  /**< 本输出帧内的检测记录标识 */
  float azimuth_deg{0.0f};         /**< 方位角，单位 deg */
  float elevation_deg{0.0f};       /**< 仰角，单位 deg */
  float infrared_snr_linear{0.0f}; /**< 红外通道线性 IR SNR */
  SbirsObservationStage observation_stage{SbirsObservationStage::kWideFieldSearch}; /**< 观测阶段 */
  bool detected{false}; /**< 是否通过探测门限判决 */
};

/** @brief 检测记录列表。 */
using SbirsDetectionRecordList = std::vector<SbirsDetectionRecord>;

}  // namespace output

namespace attribution {

/**
 * @brief NFOV 交接与跟踪失败原因，仅供归属/调试层消费，不进 `SbirsOutputFrame` raw output。
 * @note 标识首次捕获、调度、估计门、pointing timeout 或闭环跟踪门导致的失败。
 */
enum class ONEQ_API SbirsCaptureFailureReason {
  kNone = 0,              /**< 无失败（成功捕获或仅 WFOV 观测） */
  kNfovAcquisitionFailed, /**< 进入 NFOV 待捕获但捕获失败（视场外或 SNR 不足） */
  kSchedulerSkipped,      /**< WFOV 候选未被调度器选中（资源被占用或排序靠后） */
  kEstimationNisGateLost, /**< EKF NIS 连续超限导致释放 NFOV 锁定 */
  kNfovPointingTimeout,   /**< ATP 光轴在派生等待上限内未稳定 */
  kNfovTrackingGateLost   /**< NFOV 跟踪几何/SNR 门连续失败导致丢锁 */
};

/**
 * @brief 检测记录到输入目标的仿真归属记录，仅供结构化结果/调试层消费。
 * @note 归属信息不得混入 `SbirsOutputFrame` raw output；`estimated_range_m` 为内部
 *       cue/诊断层估计距离，不代表真实被动红外测距能力。
 */
struct ONEQ_API SbirsDetectionAttributionRecord {
  std::uint64_t detection_id{0U}; /**< 对应的检测记录标识 */
  std::uint64_t target_id{0U};    /**< 输入场景目标 ID */
  std::string target_name{};      /**< 输入场景目标名称 */
  float estimated_range_m{0.0f};  /**< 估计距离，单位 m（仅归属/诊断层） */
  bool used_truth_assist{false};  /**< 是否使用真值辅助跟踪 */
  SbirsCaptureFailureReason capture_failure_reason{
      SbirsCaptureFailureReason::kNone};    /**< NFOV 交接/跟踪失败原因（仅归属/诊断层） */
  bool has_estimation_nis{false};           /**< 是否包含 EKF 估计跟踪 NIS 诊断 */
  float estimation_nis{0.0f};               /**< EKF 归一化新息平方，仅归属/诊断层 */
  bool estimation_nis_gate_exceeded{false}; /**< EKF NIS 是否超过 2 维 95% 门限 */
  int nfov_channel_id{-1}; /**< NFOV 通道编号；-1 表示 WFOV/未占用 NFOV 资源（仅归属/诊断层） */
  bool has_nfov_tracking_diagnostics{false}; /**< 是否包含闭环 NFOV 跟踪门诊断 */
  float nfov_pointing_error_deg{0.0f};       /**< 有效光轴中心与目标真值 LOS 角距 */
  bool nfov_geometry_gate_passed{false};     /**< 目标是否位于实际 NFOV 矩形窗口内 */
  bool nfov_snr_gate_passed{false};          /**< 目标是否通过既有 NFOV SNR 门 */
  unsigned int nfov_tracking_gate_failure_count{0U}; /**< 连续 NFOV 跟踪门失败计数 */
  bool nfov_tracking_coasting{false}; /**< 本周期无有效 NFOV 量测但尚未正式丢锁 */
};

/** @brief 归属记录列表。 */
using SbirsDetectionAttributionRecordList = std::vector<SbirsDetectionAttributionRecord>;

}  // namespace attribution

namespace session {

/** @brief 单周期执行的中止原因：无或输入校验拒绝。 */
enum class ONEQ_API SbirsPipelineAbortReason {
  kNone = 0,           /**< 正常执行，无中止 */
  kValidationRejected /**< 输入校验拒绝 */
};

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_OUTPUT_TYPES_H_
