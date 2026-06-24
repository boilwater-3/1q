/**
 * @file RadarTrackLifecycleRecorder.h
 * @brief 定义机载雷达轨迹生命周期记录器。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_RADAR_TRACK_LIFECYCLE_RECORDER_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_RADAR_TRACK_LIFECYCLE_RECORDER_H_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/airborne_radar/session/RadarCycleResult.h"

namespace airborne_radar {
namespace session {

// 前向声明：Update 参数为 const 引用，header 无需完整类型，避免拉入 RadarCycleInput 重依赖。
struct RadarCycleInput;

enum class RadarTrackLifecycleEventKind {
  kFirstConfirmed = 0,
  kUpdated = 1,
  kLost = 2,
  kNotTracked = 3
};

enum class RadarTrackLifecycleReason {
  kNone = 0,
  kNoTrack = 1,
  kValidationRejected = 2,
  kCycleNotExecuted = 3,
  kUnknown = 4
};

struct ONEQ_API RadarTrackLifecycleEvent {
  std::uint32_t cycle_index{0U};
  std::uint64_t external_target_id{0U};
  std::string target_name{};
  RadarTrackLifecycleEventKind kind{RadarTrackLifecycleEventKind::kUpdated};
  RadarTrackLifecycleReason reason{RadarTrackLifecycleReason::kNone};
  std::uint64_t association_key{0U};
  model::TrackStatus track_status{model::TrackStatus::kTentative};
  float speed{0.0f};
};

struct ONEQ_API RadarTrackLifecycleRecorderConfig {
  bool emit_not_tracked_events{false};
};

/**
 * @brief 记录轨迹首次确认/更新/丢失/未跟踪事件；未跟踪原因需显式开启。
 *
 * 生命周期贴合 AR 的 TrackStatus(kTentative/kConfirmed/kLost)：
 * 首次进入 kConfirmed 产生 kFirstConfirmed；已确认周期更新产生 kUpdated；
 * 进入 kLost 产生 kLost；输入目标无对应 track 且开启诊断时产生 kNotTracked。
 * 私有状态(含 unordered_map)与判定逻辑见 .cpp，避免在 public header 暴露实现细节。
 */
class ONEQ_API RadarTrackLifecycleRecorder {
 public:
  explicit RadarTrackLifecycleRecorder(
      RadarTrackLifecycleRecorderConfig config = RadarTrackLifecycleRecorderConfig{});
  ~RadarTrackLifecycleRecorder();

  RadarTrackLifecycleRecorder(const RadarTrackLifecycleRecorder&) = delete;
  RadarTrackLifecycleRecorder& operator=(const RadarTrackLifecycleRecorder&) = delete;
  // 移动操作声明在 header、定义在 .cpp：unique_ptr<Impl> 析构需要完整类型，
  // 不能内联定义否则破坏 PImpl 不透明性。
  RadarTrackLifecycleRecorder(RadarTrackLifecycleRecorder&&) noexcept;
  RadarTrackLifecycleRecorder& operator=(RadarTrackLifecycleRecorder&&) noexcept;

  std::vector<RadarTrackLifecycleEvent> Update(const RadarCycleInput& input, const RadarCycleResult& result);

  void Reset();

 private:
  // 不透明私有状态，定义在 .cpp 中，避免在 header 暴露 <unordered_map> 依赖。
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_RADAR_TRACK_LIFECYCLE_RECORDER_H_
