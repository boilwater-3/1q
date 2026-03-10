// Copyright 2026. All Rights Reserved.
//
// Description: 定义雷达核心流程对外发布的事件类型。

#ifndef AIRBORNE_RADAR_CORE_EVENT_RADAR_EVENTS_H_
#define AIRBORNE_RADAR_CORE_EVENT_RADAR_EVENTS_H_

#include <cstddef>

#include "1q/airborne_radar/common/TargetFeature.h"

namespace airborne_radar {
namespace core {
namespace event {

/// @brief TracksUpdatedEvent 表示本周期的航迹状态已更新。
struct TracksUpdatedEvent {
  /// @brief 信号处理后的最新雷达状态。
  common::TargetFeatureList state{};
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

} // namespace event
} // namespace core
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_CORE_EVENT_RADAR_EVENTS_H_
