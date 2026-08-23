/**
 * @file ArOutputTypes.h
 * @brief 机载雷达输出辅助类型集合。
 *
 * 输出辅助类型（问题条目、关联质量指标等）的主头文件。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_OUTPUT_TYPES_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_OUTPUT_TYPES_H_

#include <cstddef>
#include <cstdint>
#include <string>

#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "1q/api.hpp"
#include "1q/foundation/validation_types.h"

namespace airborne_radar {
namespace session {

/**
 * @brief AR 问题条目严重等级。
 */
enum class ONEQ_API ArIssueSeverity : std::uint8_t {
  kInfo = 0,    /**< 信息 */
  kWarning = 1, /**< 警告 */
  kError = 2    /**< 错误 */
};

/**
 * @brief AR 问题条目来源阶段（统一问题列表模型，session_contract.md 规则 14）。
 * @note 结构化来源判别字段；状态判断仍以 `status`/`abort_reason` 为准，phase 不改变状态语义。
 */
enum class ONEQ_API ArIssuePhase : std::uint8_t {
  kInputValidation = 0, /**< 输入校验阶段（调用方输入问题） */
  kExecution = 1,       /**< 信号/决策管线执行阶段（含关机等运行态条件） */
  kOutputContract = 2   /**< 输出违反内部契约 */
};

/**
 * @brief AR 问题条目门内归因（规则 13b 门内归因条款，session_contract.md）。
 * @note 仅排除诊断（如 "ar.target_snr_below_threshold"）使用：SNR 检测门折入距离/波束/
 *       噪声底/RCS 多种物理因素，cause 标识门失败的主因物理链路；主因 = 对该项取理想值后
 *       门余量增益最大者。不替代 code（机器键仍只认 code），不用于状态判断。
 */
enum class ONEQ_API ArIssueCause : std::uint8_t {
  kNone = 0,          /**< 无归因（具体门排除或非排除诊断） */
  kDistanceLimited,   /**< 距离衰减（4·r_db）主导门失败 */
  kBeamLimited,       /**< 波束偏轴/方向图衰减主导门失败 */
  kNoiseLimited,      /**< 噪声底（热噪声/杂波/干扰/大气）主导门失败 */
  kRcsLimited,        /**< 目标 RCS 主导门失败 */
  kUnknown            /**< 无法判定主因 */
};

/**
 * @brief ArIssue 描述单周期问题条目（统一问题列表模型，session_contract.md 规则 14）。
 * @note 承载输入校验问题（phase=kInputValidation）与执行诊断（phase=kExecution/kOutputContract）；
 *       code 带模块前缀（如 "ar.lifecycle_unavailable"、"ar.validation.invalid_cycle_delta_time"），
 *       机器消费只认 code；不用于调用方状态判断。
 */
struct ONEQ_API ArIssue {
  ArIssueSeverity severity{ArIssueSeverity::kInfo};
  ArIssuePhase phase{ArIssuePhase::kExecution};
  std::string code{};
  std::string message{};
  oneq::foundation::ValidationLocation location{}; /**< 可选定位；kind==kGlobal 表示无定位 */
  std::string field{}; /**< 可选定位；为空表示无关联字段（跨字段或域级问题） */
  ArIssueCause cause{ArIssueCause::kNone}; /**< 可选归因；仅排除诊断使用（规则 13b） */
};

/** @brief AR 问题条目列表。 */
using ArIssueList = std::vector<ArIssue>;

/**
 * @brief SignalCycleAbortReason 描述信号流水线单周期终止原因。
 */
enum class ONEQ_API SignalCycleAbortReason {
  kNone = 0,
  kLifecycleUnavailable = 1,
  kInvalidEnvironmentCycle = 2,
  kRuntimePreparationFailed = 3,
  kValidationRejected = 4,
  kSensorPoweredOff = 5, /**< 设备关机，未执行主链路但配置边界已被接受 */
};

/**
 * @brief AssociationQualityMetrics 表示关联质量观测指标（对外公开版本）。
 * @note 字段按量纲分为 counts、rates、costs 与 normalized summaries。聚合统计时，
 *       counts 适合求和，rates/costs/summaries 通常应按有效周期求均值或分位数。
 */
struct ONEQ_API AssociationQualityMetrics {
  std::size_t prior_track_count{0};  /**< count: 进入关联阶段的历史先验轨迹数 */
  std::size_t detection_count{0};    /**< count: 本周期探测成功并参与关联的量测数 */
  std::size_t matched_count{0};      /**< count: 命中已有轨迹的关联数 */
  std::size_t new_track_count{0};    /**< count: 触发新建轨迹键的量测数 */
  std::size_t missed_track_count{0}; /**< count: 未命中任何量测的历史轨迹数 */
  float match_rate{0.0f};            /**< rate [0,1]: matched_count / detection_count */
  float new_track_rate{0.0f};        /**< rate [0,1]: new_track_count / detection_count */
  float missed_track_rate{0.0f};     /**< rate [0,1]: missed_track_count / prior_track_count */
  float mean_match_cost{0.0f};       /**< cost: 命中关联代价均值（仅统计 matches） */
  float p95_match_cost{0.0f};        /**< cost: 命中关联代价 P95（仅统计 matches） */
  float association_stress{0.0f};      /**< summary [0,1]: 当前周期的归一化关联压力 */
};

/**
 * @brief ArTrackAttributionRecord 航迹归属记录（库内键 ↔ 场景真值目标对照，信封通道）。
 * @note 仅供结构化结果/调试层消费，是 AR 仿真真值归属的权威路径（session_contract.md
 *       Attribution 挂载表）；`TrackStateSnapshot.external_target_id/target_name` 为注册
 *       deprecated 产品遗留（sim-only），新代码不得以其为关联依据。覆盖范围为产品导出
 *       航迹（output_frame.tracks，去重后），与产品帧逐条对应；非执行周期（校验失败/
 *       关机/中止）返回空列表，不复用上一周期。
 */
struct ONEQ_API ArTrackAttributionRecord {
  std::uint64_t association_key{0};    /**< AR 内部航迹关联键。 */
  std::uint64_t external_target_id{0}; /**< 场景真值目标 ID（0 = 未提供）。 */
  std::string target_name{};           /**< 场景真值目标名。 */
};

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_OUTPUT_TYPES_H_
