// Copyright 2026. All Rights Reserved.
//
// @file RadarCycleResult.h
// @brief 定义 RadarSession 单周期聚合结果类型。

#ifndef AIRBORNE_RADAR_CORE_SESSION_RADAR_CYCLE_RESULT_H_
#define AIRBORNE_RADAR_CORE_SESSION_RADAR_CYCLE_RESULT_H_

#include <vector>

#include "1q/airborne_radar/common/RadarCommand.h"
#include "1q/airborne_radar/common/RadarControlProfile.h"
#include "1q/airborne_radar/common/TrackOutputFrame.h"
#include "1q/airborne_radar/signal/pipeline/ISignalPipeline.h"
#include "1q/airborne_radar/signal/tracking/TrackLifecycleTypes.h"

namespace airborne_radar {
namespace core {
namespace session {

/// @brief RadarCycleResult 描述单周期执行后的聚合观测结果。
struct RadarCycleResult {
  /// @brief 当前周期轨迹输出帧。
  common::TrackOutputFrame track_output_frame{};

  /// @brief 当前周期已提交的控制指令。
  std::vector<common::RadarCommand> submitted_commands{};

  /// @brief 是否已持有最近一次控制真值。
  bool has_control_profile{false};

  /// @brief 最近一次控制真值。
  common::RadarControlProfile control_profile{};

  /// @brief 最近一次关联质量观测指标。
  signal::pipeline::AssociationQualityMetrics association_quality_metrics{};

  /// @brief 最近一次跟踪量测列表。
  std::vector<signal::tracking::TrackMeasurement> track_measurements{};
};

} // namespace session
} // namespace core
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_CORE_SESSION_RADAR_CYCLE_RESULT_H_
