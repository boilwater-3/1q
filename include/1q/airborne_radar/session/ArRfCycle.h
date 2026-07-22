/**
 * @file ArRfCycle.h
 * @brief 定义 AR 工程 RF 两阶段周期协议的公共值类型。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_RF_CYCLE_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_RF_CYCLE_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/config/ArEnvironmentConfig.h"
#include "1q/airborne_radar/session/ArCycleResult.h"
#include "1q/airborne_radar/session/ArInterferenceObservation.h"
#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "1q/api.hpp"
#include "1q/coordinate/types.h"
#include "1q/electromagnetics/RfScene.h"

namespace airborne_radar {
namespace session {

/** @brief Prepare 阶段的不透明周期令牌。 */
struct ONEQ_API ArPreparedCycleToken {
  std::uint64_t value{0U};             /**< 会话内单调令牌值；0 表示无效。 */
  std::uint64_t world_cycle_index{0U}; /**< 令牌所属世界周期。 */
};

/** @brief Prepare 请求；world time 由外部 orchestrator 唯一拥有。 */
struct ONEQ_API ArPrepareCycleInput {
  std::uint64_t world_cycle_index{0U};                        /**< 世界周期号。 */
  double window_start_time_s{0.0};                            /**< 接收窗口绝对起始时间（s）。 */
  double window_duration_s{0.0};                              /**< 接收窗口持续时间（s）。 */
  std::uint64_t platform_id{0U};                              /**< RF platform 身份；必须非零。 */
  oneq::coordinate::EcefPositionM platform_position_ecef_m{}; /**< 平台 ECEF 位置。 */
  oneq::coordinate::EcefVelocityMps platform_velocity_ecef_mps{};    /**< 平台 ECEF 速度。 */
  oneq::electromagnetics::RfSceneDirection antenna_boresight_ecef{}; /**< 本周期实际波束轴。 */
};

/** @brief Prepare 阶段状态。 */
enum class ArPrepareCycleStatus : std::uint8_t {
  kPrepared = 0, /**< 已发布实际发射并冻结接收状态。 */
  kPoweredOff,   /**< 世界时间已推进，但未消费发射身份或随机状态。 */
  kBusy,         /**< 已存在一个待完成令牌。 */
  kRejected      /**< 输入或运行期配置非法，未改变周期状态。 */
};

/** @brief Prepare 阶段结果。 */
struct ONEQ_API ArPrepareCycleResult {
  ArPrepareCycleStatus status{ArPrepareCycleStatus::kRejected}; /**< Prepare 状态。 */
  ArPreparedCycleToken token{};                                 /**< 成功时的待完成令牌。 */
  bool has_emission{false};                                     /**< 是否发布了实际发射。 */
  oneq::electromagnetics::RfSceneEmission emission{};           /**< 实际 AR 发射事实。 */
  oneq::electromagnetics::RfSceneReceiverState
      receiver_state{}; /**< Complete 使用的冻结接收状态。 */
};

/** @brief Complete 阶段场景与目标输入。 */
struct ONEQ_API ArCompleteCycleInput {
  oneq::electromagnetics::RfSceneFrame rf_scene{}; /**< orchestrator 冻结的 RF v2 场景。 */
  ArSceneTargetList targets{};                     /**< 本周期目标事实。 */
  config::AtmosphericPhysicsConfig atmospheric_observation{};   /**< 大气传播输入。 */
  config::AtmosphericDerivedContext atmospheric_context{};      /**< 时间与空间天气输入。 */
  config::VegetationScatterPhysicsConfig surface_observation{}; /**< 地表杂波输入。 */
};

/** @brief 接收机结构化损伤状态。 */
enum class ArReceiverImpairment : std::uint8_t {
  kNone = 0, /**< 接收机在线性区工作。 */
  kSaturated /**< 宽带前端总输入超过最大线性输入功率。 */
};

/** @brief Complete 阶段状态。 */
enum class ArCompleteCycleStatus : std::uint8_t {
  kCompleted = 0, /**< 本周期已完成物理执行并产生新输出。 */
  kRejected,      /**< 输入或执行被拒绝；令牌保留，可重试。 */
  kTokenMismatch  /**< 令牌不属于当前待完成周期。 */
};

/** @brief Complete 阶段结果；仅 kCompleted 携带本周期输出。 */
struct ONEQ_API ArCompleteCycleResult {
  ArCompleteCycleStatus status{ArCompleteCycleStatus::kRejected}; /**< Complete 状态。 */
  std::uint64_t world_cycle_index{0U};                            /**< 结果所属世界周期。 */
  TrackOutputFrame track_output_frame{};                          /**< 本周期新轨迹帧。 */
  ArInterferenceObservationList interference_observations{}; /**< 仅通过 J/N 门的本机 RF 观测。 */
  ArReceiverImpairment receiver_impairment{ArReceiverImpairment::kNone}; /**< 结构化接收机损伤。 */
};

/** @brief Abandon 阶段状态。 */
enum class ArAbandonCycleStatus : std::uint8_t {
  kAbandoned = 0, /**< 已释放待完成接收阶段；发射状态不回滚。 */
  kTokenMismatch  /**< 无待完成周期或令牌不匹配。 */
};

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_RF_CYCLE_H_
