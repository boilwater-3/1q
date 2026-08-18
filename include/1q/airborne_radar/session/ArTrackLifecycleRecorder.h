/**
 * @file ArTrackLifecycleRecorder.h
 * @brief 机载雷达轨迹生命周期记录类型集合。
 *
 * 轨迹首次确认/更新/丢失/未跟踪事件记录的主头文件。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_TRACK_LIFECYCLE_RECORDER_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_TRACK_LIFECYCLE_RECORDER_H_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "1q/airborne_radar/session/ArCycleResult.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace session {

/**
 * @brief 轨迹生命周期事件类型。
 */
enum class ONEQ_API ArTrackLifecycleEventKind {
  kFirstConfirmed = 0, /**< 目标首次进入已确认状态。 */
  kUpdated = 1,        /**< 已确认轨迹本周期再次更新。 */
  kLost = 2,           /**< 轨迹进入丢失状态。 */
  kNotTracked = 3,     /**< 输入目标无对应 track（需显式开启诊断）。 */
  kDesignationDropped = 4 /**< 指定跟踪目标被自动放弃（STT 航迹跟随回退 TWS）。 */
};

/**
 * @brief 未跟踪/丢失事件的成因归类（仅在 kNotTracked 诊断时填充）。
 */
enum class ONEQ_API ArTrackLifecycleReason {
  kNone = 0,              /**< 无特殊成因（已确认/更新/丢失事件均用此值）。 */
  kNoTrack = 1,           /**< 本周期未为该输入目标建立任何 track。 */
  kUnknown = 2            /**< 其他无法归类的情形。 */
};

/**
 * @brief 单条轨迹生命周期事件记录。
 */
struct ONEQ_API ArTrackLifecycleEvent {
  std::uint64_t world_cycle_index{0U};                                /**< 触发该事件的世界周期号 */
  std::uint64_t external_target_id{0U};                               /**< 输入目标外部标识符 */
  std::string target_name{};                                          /**< 目标名称（人读标签） */
  ArTrackLifecycleEventKind kind{ArTrackLifecycleEventKind::kUpdated}; /**< 事件类型 */
  ArTrackLifecycleReason reason{ArTrackLifecycleReason::kNone};       /**< 事件成因（诊断用） */
  std::uint64_t association_key{0U};                                  /**< 关联轨迹的关联键（无 track 时为 0） */
  session::TrackStatus track_status{session::TrackStatus::kTentative}; /**< 关联轨迹的生命周期状态 */
  float speed{0.0f};                                                  /**< 关联轨迹的速度（无 track 时为 0） */
  ArDesignationRevertReason designation_revert_reason{
      ArDesignationRevertReason::kNone}; /**< 指定跟踪回退成因（仅 kDesignationDropped 填充） */
};

/**
 * @brief 轨迹生命周期记录器配置。
 */
struct ONEQ_API ArTrackLifecycleRecorderConfig {
  bool emit_not_tracked_events{false}; /**< 是否为无对应 track 的输入目标产生 kNotTracked 事件 */
};

/**
 * @brief 记录轨迹首次确认/更新/丢失/未跟踪事件；未跟踪原因需显式开启。
 *
 * 生命周期贴合 AR 的 TrackStatus(kTentative/kConfirmed/kLost)：
 * 首次进入 kConfirmed 产生 kFirstConfirmed；已确认周期更新产生 kUpdated；
 * 进入 kLost 产生 kLost；输入目标无对应 track 且开启诊断时产生 kNotTracked。
 * 非 completed 周期不产生事件，也不推进记录器状态。
 * 私有状态(含 unordered_map)与判定逻辑见 .cpp，避免在 public header 暴露实现细节。
 */
class ONEQ_API ArTrackLifecycleRecorder {
 public:
  explicit ArTrackLifecycleRecorder(
      ArTrackLifecycleRecorderConfig config = ArTrackLifecycleRecorderConfig{});
  ~ArTrackLifecycleRecorder();

  ArTrackLifecycleRecorder(const ArTrackLifecycleRecorder&) = delete;
  ArTrackLifecycleRecorder& operator=(const ArTrackLifecycleRecorder&) = delete;
  // 移动操作声明在 header、定义在 .cpp：unique_ptr<Impl> 析构需要完整类型，
  // 不能内联定义否则破坏 PImpl 不透明性。
  ArTrackLifecycleRecorder(ArTrackLifecycleRecorder&&) noexcept;
  ArTrackLifecycleRecorder& operator=(ArTrackLifecycleRecorder&&) noexcept;

  /**
   * @brief 基于目标事实与单周期结果产出生命周期事件。
   *
   * 仅处理 `external_target_id != 0` 的输入目标；按确认/更新/丢失/未跟踪规则
   * 生成事件，未跟踪事件仅在配置开启时产生。
   *
   * @param[in] targets 当前周期目标事实（用于遍历输入目标表）。
   * @param[in] result 当前周期结果（用于查询关联轨迹状态）。
   * @return 本周期产生的生命周期事件列表（可能为空）。
   */
  std::vector<ArTrackLifecycleEvent> Update(const ArTargetInputList& targets,
                                            const ArCycleResult& result);

  /**
   * @brief 清空内部轨迹状态，回到初始状态。
   */
  void Reset();

  /**
   * @brief 获取最近一次执行周期 `Update()` 返回的生命周期事件列表。
   *
   * 事件在每次执行周期的 `Update()` 调用时缓存；非执行周期不刷新缓存，
   * 保留上一次执行周期的事件。供注册到 Session 后由调用方事后读取。
   * @return 最近一次执行周期 `Update()` 产生的事件列表的 const 引用。
   */
  const std::vector<ArTrackLifecycleEvent>& GetLastEvents() const noexcept;

 private:
  // 不透明私有状态，定义在 .cpp 中，避免在 header 暴露 <unordered_map> 依赖。
  struct Impl;
  std::unique_ptr<Impl> impl_;
};


}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_TRACK_LIFECYCLE_RECORDER_H_
