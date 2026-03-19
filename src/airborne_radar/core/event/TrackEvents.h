// Copyright 2026. All Rights Reserved.
//
// @file TrackEvents.h
// @brief 定义轨迹生命周期相关事件。

#ifndef AIRBORNE_RADAR_CORE_EVENT_TRACK_EVENTS_H_
#define AIRBORNE_RADAR_CORE_EVENT_TRACK_EVENTS_H_

#include <cstdint>

namespace airborne_radar {
namespace core {
namespace event {

/// @brief TrackCreatedEvent 表示新轨迹已创建。
struct TrackCreatedEvent {
  /// @brief 轨迹编号。
  std::uint64_t track_id{0};

  /// @brief 轨迹所属批号。
  std::uint64_t batch_id{0};
};

/// @brief TrackConfirmedEvent 表示轨迹已由候选转为确认。
struct TrackConfirmedEvent {
  /// @brief 轨迹编号。
  std::uint64_t track_id{0};
};

/// @brief TrackLostEvent 表示轨迹进入丢失状态。
struct TrackLostEvent {
  /// @brief 轨迹编号。
  std::uint64_t track_id{0};

  /// @brief 当前连续失配计数。
  std::uint32_t miss_count{0};
};

/// @brief TrackRecycledEvent 表示轨迹对象已回收到对象池。
struct TrackRecycledEvent {
  /// @brief 轨迹编号。
  std::uint64_t track_id{0};

  /// @brief 回收前对象代次。
  std::uint32_t generation{0};
};

} // namespace event
} // namespace core
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_CORE_EVENT_TRACK_EVENTS_H_
