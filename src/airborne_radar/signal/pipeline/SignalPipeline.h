// Copyright 2026. All Rights Reserved.
//
// @file SignalPipeline.h
// @brief 定义信号处理流水线的默认内部实现。

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_PIPELINE_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_PIPELINE_H_

#include <memory>
#include <vector>

#include "1q/airborne_radar/signal/pipeline/ISignalPipeline.h"
#include "airborne_radar/signal/tracking/ITrackLifecycleManager.h"
#include "airborne_radar/signal/tracking/TrackLifecycleTypes.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

/// @brief SignalPipeline 提供可配置的信号处理默认实现。
class SignalPipeline final : public ISignalPipeline {
 public:
  explicit SignalPipeline(SignalPipelineConfig config = {});
  ~SignalPipeline() override;

  SignalCycleResult RunCycle(
      const common::TargetFeatureList& input_state,
      const environment::IEnvironmentService& environment) override;

  std::vector<tracking::TrackMeasurement> GetLastTrackMeasurements() const;

  AssociationQualityMetrics GetLastAssociationQualityMetrics() const;

  void SetAssociationSeeds(
      const std::vector<tracking::AssociationTrackSeed>& seeds);

  void ResetAssociationSeedModeToStateless();

  std::unique_ptr<tracking::ITrackLifecycleManager>
  CreateAutoLifecycleManager() const;

  void UpdatePlatformAttitude(
      const common::PlatformAttitudeDeg& platform_attitude_deg) override;

  common::PlatformAttitudeDeg GetPlatformAttitude() const override;

  void SetControlProfile(
      const common::RadarControlProfile& control_profile) override;

  common::RadarControlProfile GetControlProfile() const override;

  void UpdateConfig(SignalPipelineConfig config);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_PIPELINE_H_
