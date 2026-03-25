/**
 * @file RadarEvents.h
 * @brief 定义雷达核心流程对外发布的事件类型。
 */

#ifndef AIRBORNE_RADAR_CORE_EVENT_RADAR_EVENTS_H_
#define AIRBORNE_RADAR_CORE_EVENT_RADAR_EVENTS_H_

#include <cstddef>
#include <cstdint>

#include "1q/airborne_radar/common/ControlDirective.h"
#include "1q/airborne_radar/common/TargetFeature.h"
#include "1q/airborne_radar/common/TrackOutputFrame.h"

namespace airborne_radar {
namespace core {
namespace event {

/**
 * @brief 表示本周期的航迹状态已更新。
 */
struct TracksUpdatedEvent {
  common::TargetFeatureList state{}; /**< 信号处理后的最新雷达状态。 */
};

/**
 * @brief 表示中性轨迹输出帧已发布。
 */
struct TrackOutputPublishedEvent {
  common::TrackOutputFrame frame{}; /**< 当前周期发布的中性轨迹输出帧。 */
};

/**
 * @brief 表示检测到电子干扰告警。
 */
struct JammingAlertEvent {
  bool detected{false}; /**< 干扰是否被检测到。 */
};

/**
 * @brief 表示命令已提交到硬件层。
 */
struct CommandsSubmittedEvent {
  std::size_t command_count{0}; /**< 本周期提交的命令数量。 */
};

/**
 * @brief 表示单次雷达处理循环完成。
 */
struct RadarCycleCompletedEvent {
  std::size_t command_count{0}; /**< 本周期提交到硬件层的命令数量。 */
  bool jamming_detected{false}; /**< 本周期是否检测到干扰。 */
};

/**
 * @brief 表示控制真值已经更新。
 */
struct ControlProfileUpdatedEvent {
  std::uint32_t cycle_index{0};             /**< 触发更新的周期号。 */
  std::uint64_t profile_version{0};         /**< 新控制 profile 的版本号。 */
  std::size_t applied_directive_count{0};   /**< 已采纳的控制意图数量。 */
  std::size_t rejected_directive_count{0};  /**< 已拒绝的控制意图数量。 */
  bool lpi_power_control_enabled{false};    /**< LPI 功率控制是否启用。 */
  bool agility_frequency_enabled{false};    /**< 频率捷变是否启用。 */
  bool sidelobe_canceller_enabled{false};   /**< 旁瓣对消是否启用。 */
  bool adaptive_beamforming_enabled{false}; /**< 自适应波束形成是否启用。 */
};

/**
 * @brief 表示单条控制意图被采纳。
 */
struct DirectiveAppliedEvent {
  std::uint32_t cycle_index{0};         /**< 触发事件的周期号。 */
  std::uint64_t profile_version{0};     /**< 生效 profile 的版本号。 */
  common::ControlDirective directive{}; /**< 被采纳的控制意图。 */
};

/**
 * @brief 表示单条控制意图被拒绝。
 */
struct DirectiveRejectedEvent {
  std::uint32_t cycle_index{0};         /**< 触发事件的周期号。 */
  std::uint64_t profile_version{0};     /**< 生效 profile 的版本号。 */
  common::ControlDirective directive{}; /**< 被拒绝的控制意图。 */
};

}  // namespace event
}  // namespace core
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_EVENT_RADAR_EVENTS_H_
