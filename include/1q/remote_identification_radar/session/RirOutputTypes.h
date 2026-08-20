/**
 * @file RirOutputTypes.h
 * @brief 远程识别雷达输出与诊断基础类型。
 *
 * 统一问题列表（session_contract.md 规则 14）与周期中止原因的单一事实来源。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_OUTPUT_TYPES_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_OUTPUT_TYPES_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/remote_identification_radar/session/RirFeatureMeasurementTypes.h"
#include "1q/remote_identification_radar/session/RirRecognitionResult.h"

namespace remote_identification_radar {
namespace session {

/** @brief 问题严重级别。 */
enum class ONEQ_API RirIssueSeverity : std::uint8_t {
  kInfo = 0,    /**< 信息级（不改变周期语义）。 */
  kWarning = 1, /**< 警告级。 */
  kError = 2    /**< 错误级（校验拒绝/执行失败）。 */
};

/** @brief 问题来源阶段标签。 */
enum class ONEQ_API RirIssuePhase : std::uint8_t {
  kInputValidation = 0, /**< 输入/配置校验。 */
  kExecution = 1,       /**< 周期执行。 */
  kOutputContract = 2   /**< 输出契约。 */
};

/**
 * @brief RirIssue 描述单周期问题条目（统一问题列表模型，session_contract.md 规则 14）。
 */
struct ONEQ_API RirIssue {
  RirIssueSeverity severity{RirIssueSeverity::kInfo};
  RirIssuePhase phase{RirIssuePhase::kExecution};
  std::string code{};   /**< 细粒度代码，形如 "rir.validation.<snake_case>"。 */
  std::string field{};  /**< 定位字段路径（可选）。 */
  std::string message{}; /**< 人读描述（英文）。 */
};

/** @brief RirIssueList 单周期统一问题列表。 */
using RirIssueList = std::vector<RirIssue>;

/** @brief 周期中止原因（粗粒度结构化信号）。 */
enum class ONEQ_API RirCycleAbortReason {
  kNone = 0,              /**< 未中止。 */
  kValidationRejected,    /**< 输入/配置校验拒绝。 */
  kPoweredOff,            /**< 传感器关机（非执行周期）。 */
  kExecutionRejected      /**< 执行阶段失败。 */
};

/**
 * @brief RirTrackRecognitionOutput 单条航迹的识别结论输出。
 */
struct ONEQ_API RirTrackRecognitionOutput {
  std::uint64_t association_key{0}; /**< 结论所属航迹关联键。 */
  RirRecognitionResult result{};    /**< 识别结论。 */
};

/**
 * @brief RirOutputFrame 识别雷达系统输出帧（双产品：识别结论 + 特征量测）。
 */
struct ONEQ_API RirOutputFrame {
  std::uint32_t input_cycle_index{0U}; /**< 本次调用输入周期号。 */
  std::uint64_t batch_id{0U};          /**< 输入批号。 */
  std::vector<RirTrackRecognitionOutput> recognition_outputs{}; /**< 逐航迹识别结论（出口②）。 */
  std::vector<RirFeatureMeasurementRecord> feature_measurements{}; /**< 逐航迹特征量测（出口①，
      加性字段，默认空；全维无效记录不产生，识别链未构建观测的周期为空）。 */
};

/**
 * @brief RirTrackAttributionRecord 航迹归属记录（库内键 ↔ 场景真值目标对照）。
 * @note 仅供结构化结果/调试层消费，不得混入 RirOutputFrame 产品层（三层纪律，
 *       与 SBIRS detection_attributions 同层同纪律）；识别结论本体不在此重复
 *       出口（出口②零变更）。非执行周期（校验失败/关机/中止）返回空列表。
 */
struct ONEQ_API RirTrackAttributionRecord {
  std::uint64_t association_key{0U}; /**< RIR 内部航迹关联键。 */
  std::uint64_t external_target_id{0U}; /**< 场景真值目标 ID（0 = 未提供）。 */
  std::string target_name{};             /**< 场景真值目标名。 */
  std::uint32_t hit_count{0U};           /**< 航迹累计命中数。 */
  double position_enu_x_m{0.0};          /**< 滤波位置 ENU x（m）。 */
  double position_enu_y_m{0.0};          /**< 滤波位置 ENU y（m）。 */
  double position_enu_z_m{0.0};          /**< 滤波位置 ENU z（m）。 */
  double speed_m_per_s{0.0};             /**< 滤波速度模长（m/s）。 */
};

}  // namespace session
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_OUTPUT_TYPES_H_
