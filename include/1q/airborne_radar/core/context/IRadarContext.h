// Copyright 2026. All Rights Reserved.
//
// @file IRadarContext.h
// @brief 定义雷达系统对外暴露的上下文抽象接口。
// 它解耦了中介者模式与环境状态之间的依赖。

#ifndef AIRBORNE_RADAR_CORE_CONTEXT_I_RADAR_CONTEXT_H_
#define AIRBORNE_RADAR_CORE_CONTEXT_I_RADAR_CONTEXT_H_

#include "1q/airborne_radar/common/RadarOrientationConfig.h"
#include "1q/airborne_radar/common/RadarCommand.h"
#include "1q/airborne_radar/common/RadarControlProfile.h"
#include "1q/airborne_radar/common/TargetFeature.h"

namespace airborne_radar {
namespace core {
namespace context {

/// @brief IRadarContext 抽象了系统的当前战术态势操作。
/// 通过该接口，控制器不再依赖具体的处理层类，实现依赖倒置。
class IRadarContext {
public:
  virtual ~IRadarContext() = default;

  /// @brief 获取当前周期的目标特征列表。
  virtual common::TargetFeatureList GetTargetFeatures() const = 0;

  /// @brief 获取当前搭载平台姿态角。
  /// @return 当前平台姿态角（单位：度）。
  virtual common::PlatformAttitudeDeg GetPlatformAttitude() const = 0;

  /// @brief 获取当前周期时间步长。
  /// @return 当前周期时间步长（单位：秒）；<= 0 表示外部输入无效。
  virtual float GetCycleDeltaTimeSec() const = 0;

  /// @brief 提交（发射）一组战术动作命令给底座硬件或协调总线执行。
  /// @param cmd 单个被执行的控制器指令。
  virtual void SubmitControlCommand(common::RadarCommand cmd) = 0;

  /// @brief 通知最新控制真值已生成。
  /// @param profile 下一周期控制真值。
  virtual void UpdateRadarControlProfile(
      const common::RadarControlProfile& profile) {
    (void)profile;
  }
};
} // namespace context
} // namespace core
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_CORE_CONTEXT_I_RADAR_CONTEXT_H_
