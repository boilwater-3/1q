// Copyright 2026. All Rights Reserved.
//
// Description: 定义雷达核心流程对外发布的事件类型。

#ifndef AIRBORNE_RADAR_CORE_EVENT_RADAR_EVENTS_H_
#define AIRBORNE_RADAR_CORE_EVENT_RADAR_EVENTS_H_

#include <cstddef>
#include <cstdint>

#include "1q/airborne_radar/common/TrackOutputFrame.h"
#include "1q/airborne_radar/common/ControlDirective.h"
#include "1q/airborne_radar/common/TargetFeature.h"

namespace airborne_radar {
namespace core {
namespace event {

/// @brief TracksUpdatedEvent 表示本周期的航迹状态已更新。
struct TracksUpdatedEvent {
  /// @brief 信号处理后的最新雷达状态。
  common::TargetFeatureList state{};
};

/// @brief TrackOutputPublishedEvent 表示中性轨迹输出帧已发布。
struct TrackOutputPublishedEvent {
  /// @brief 当前周期发布的中性轨迹输出帧。
  common::TrackOutputFrame frame{};
};

/// @brief JammingAlertEvent 表示检测到电子干扰告警。
struct JammingAlertEvent {
  /// @brief 干扰是否被检测到。
  bool detected{false};
};

/// @brief CommandsSubmittedEvent 表示命令已提交到硬件层。
struct CommandsSubmittedEvent {
  /// @brief 本周期提交命令数量。
  std::size_t command_count{0};
};

/// @brief RadarCycleCompletedEvent 表示单次雷达处理循环完成事件。
struct RadarCycleCompletedEvent {
  /// @brief 本周期提交到硬件层的命令数量。
  std::size_t command_count{0};

  /// @brief 本周期是否检测到干扰。
  bool jamming_detected{false};
};

/// @brief ControlProfileUpdatedEvent 表示控制真值已经更新。
struct ControlProfileUpdatedEvent {
  /// @brief 触发更新的周期号。
  std::uint32_t cycle_index{0};

  /// @brief 新 profile 版本号。
  std::uint64_t profile_version{0};

  /// @brief 采纳的控制意图数量。
  std::size_t applied_directive_count{0};

  /// @brief 拒绝的控制意图数量。
  std::size_t rejected_directive_count{0};

  /// @brief LPI 功率控制是否启用。
  bool lpi_power_control_enabled{false};

  /// @brief 频率捷变是否启用。
  bool agility_frequency_enabled{false};

  /// @brief 旁瓣对消是否启用。
  bool sidelobe_canceller_enabled{false};

  /// @brief 自适应波束形成是否启用。
  bool adaptive_beamforming_enabled{false};
};

/// @brief DirectiveAppliedEvent 表示单条控制意图被采纳。
struct DirectiveAppliedEvent {
  /// @brief 触发事件的周期号。
  std::uint32_t cycle_index{0};

  /// @brief 生效 profile 版本号。
  std::uint64_t profile_version{0};

  /// @brief 被采纳的控制意图。
  common::ControlDirective directive{};
};

/// @brief DirectiveRejectedEvent 表示单条控制意图被拒绝。
struct DirectiveRejectedEvent {
  /// @brief 触发事件的周期号。
  std::uint32_t cycle_index{0};

  /// @brief 生效 profile 版本号。
  std::uint64_t profile_version{0};

  /// @brief 被拒绝的控制意图。
  common::ControlDirective directive{};
};

} // namespace event
} // namespace core
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_CORE_EVENT_RADAR_EVENTS_H_
