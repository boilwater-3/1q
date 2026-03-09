// Copyright 2026. All Rights Reserved.
//
// Description: 定义轨迹生命周期管理抽象接口。

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_I_TRACK_LIFECYCLE_MANAGER_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_I_TRACK_LIFECYCLE_MANAGER_H_

#include <vector>

#include "1q/airborne_radar/common/TargetFeature.h"
#include "1q/airborne_radar/signal/tracking/TrackLifecycleTypes.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

/// @brief ITrackLifecycleManager 抽象轨迹生命周期推进与快照导出能力。
class ITrackLifecycleManager {
public:
  virtual ~ITrackLifecycleManager() {}

  /// @brief 用本周期量测更新轨迹状态机。
  /// @param cycle 周期上下文。
  /// @param measurements 关联后量测列表。
  virtual void Update(const CycleContext &cycle,
                      const std::vector<TrackMeasurement> &measurements) = 0;

  /// @brief 导出兼容决策链路的目标特征快照。
  virtual common::TargetFeatureList BuildFeatureSnapshot() const = 0;
};

} // namespace tracking
} // namespace signal
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_SIGNAL_TRACKING_I_TRACK_LIFECYCLE_MANAGER_H_
