/**
 * @file RadarSessionConfigPresets.cpp
 * @brief 实现机载雷达公开预设配置工厂。
 */

#include "1q/airborne_radar/config/RadarSessionConfigPresets.h"

#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"

namespace airborne_radar {
namespace config {
namespace presets {

session::RadarSessionConfig MakeDefaultRadarSessionConfig() {
  return config::RadarSessionConfigBuilder().Build();
}

session::RadarSessionConfig MakeDetectionMissionRadarSessionConfig() {
  return config::RadarSessionConfigBuilder()
      .Detection()
      .WithDetectionIntentProfile(semantic::DetectionIntentProfile::kDetectionPriority)
      .End()
      .Tracking()
      .WithTrackingPolicyProfile(semantic::TrackingPolicyProfile::kFastAssociation)
      .End()
      .Lifecycle()
      .WithLifecyclePolicyProfile(semantic::LifecyclePolicyProfile::kFastConfirm)
      .End()
      .Build();
}

session::RadarSessionConfig MakeTrackingMissionRadarSessionConfig() {
  return config::RadarSessionConfigBuilder()
      .Detection()
      .WithDetectionIntentProfile(semantic::DetectionIntentProfile::kTrackStabilityPriority)
      .End()
      .Build();
}

session::RadarSessionConfig MakeHighRobustnessRadarSessionConfig() {
  return config::RadarSessionConfigBuilder()
      .Detection()
      .WithDetectionIntentProfile(semantic::DetectionIntentProfile::kTrackStabilityPriority)
      .End()
      .Tracking()
      .WithTrackingPolicyProfile(semantic::TrackingPolicyProfile::kRobustAntiJamming)
      .End()
      .Lifecycle()
      .WithLifecyclePolicyProfile(semantic::LifecyclePolicyProfile::kHighPersistence)
      .End()
      .Build();
}

}  // namespace presets
}  // namespace config
}  // namespace airborne_radar
