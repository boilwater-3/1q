// Copyright 2026. All Rights Reserved.
//
// Description: 定义轨迹生命周期管理器接口。

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_TRACK_LIFECYCLE_MANAGER_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_TRACK_LIFECYCLE_MANAGER_H_

#include <cstdint>
#include <map>
#include <vector>

#include "1q/airborne_radar/common/TargetFeature.h"
#include "1q/airborne_radar/signal/tracking/ITrackLifecycleManager.h"
#include "1q/airborne_radar/signal/tracking/ITrackPool.h"
#include "1q/airborne_radar/signal/tracking/TrackLifecycleTypes.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

class IKalmanPredictor;
class IKalmanUpdater;

/// @brief LifecycleConfig 定义轨迹状态机阈值配置。
struct LifecycleConfig {
  /// @brief 候选轨迹转已确认所需最小命中次数。
  std::uint32_t confirm_hits{3};

  /// @brief 已确认轨迹转丢失前允许的最大连续失配次数。
  std::uint32_t max_miss_before_lost{2};

  /// @brief 丢失轨迹可保留的最大周期数，超出则回收。
  std::uint32_t max_lost_cycles{5};
};

/// @brief TrackLifecycleManager 负责轨迹生命周期状态推进。
/// 该类依赖 ITrackPool 管理对象复用，并提供快照导出能力。
class TrackLifecycleManager : public ITrackLifecycleManager {
public:
  /// @brief 构造函数。
  /// @param pool 轨迹对象池抽象。
  /// @param config 生命周期阈值配置。
  /// @param predictor 可选的 Kalman 预测器（为 nullptr 时不执行状态外推）。
  /// @param updater 可选的 Kalman 更新器（为 nullptr 时不执行状态更新）。
  TrackLifecycleManager(ITrackPool &pool, const LifecycleConfig &config,
                        const IKalmanPredictor *predictor = nullptr,
                        const IKalmanUpdater *updater = nullptr);

  /// @brief 用本周期量测更新轨迹生命周期。
  /// @param cycle 当前周期上下文。
  /// @param measurements 本周期关联后的量测集合。
  void Update(const CycleContext &cycle,
              const std::vector<TrackMeasurement> &measurements) override;

  /// @brief 导出当前活跃轨迹（只读指针）。
  /// @return 活跃轨迹列表。
  std::vector<const common::TrackState *> GetActiveTracks() const;

  /// @brief 导出兼容现有决策链路的轻量目标特征快照。
  /// @return 可直接用于 DecisionContext 的目标特征列表。
  common::TargetFeatureList BuildFeatureSnapshot() const override;

private:
  /// @brief 根据命中结果推进单条轨迹状态。
  /// @param track 轨迹对象。
  /// @param cycle_index 当前周期号。
  /// @param hit_this_cycle 本周期是否命中。
  void PromoteState(common::TrackState &track,
                    std::uint32_t cycle_index,
                    bool hit_this_cycle);

  /// @brief 将对象重置为可复用状态，避免脏数据泄露。
  /// @param track 待重置轨迹对象。
  void ResetForReuse(common::TrackState &track) const;

private:
	/// @brief 轨迹对象池抽象，用于对象申请与归还。
  ITrackPool &pool_;

	/// @brief 生命周期阈值配置。
  LifecycleConfig config_;

	/// @brief 下一个待分配轨迹 ID。
  std::uint64_t next_track_id_{1};

  /// @brief 活跃轨迹表，key 为关联键。
  std::map<std::uint64_t, common::TrackState *> tracks_by_key_;

  /// @brief 可选 Kalman 预测器（非拥有）。
  const IKalmanPredictor *kalman_predictor_{nullptr};

  /// @brief 可选 Kalman 更新器（非拥有）。
  const IKalmanUpdater *kalman_updater_{nullptr};

  /// @brief 上一周期编号，用于计算 dt。
  std::uint32_t last_cycle_index_{0};

  /// @brief 每周期时间步长（秒）。
  float cycle_dt_{1.0f};
};

} // namespace tracking
} // namespace signal
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_SIGNAL_TRACKING_TRACK_LIFECYCLE_MANAGER_H_
