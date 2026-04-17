/**
 * @file RadarSessionConfigBuilder.h
 * @brief 提供链式构造 RadarSessionConfig 的 Builder（用于会话初始化基线配置）。
 */

#ifndef AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_BUILDER_H_
#define AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_BUILDER_H_

#include "1q/airborne_radar/config/RadarSessionConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {

using model::AzimuthElevationDeg;
using model::CommandedBeamwidthDeg;
using model::RadarWorkSubMode;

/**
 * @brief RadarSession 配置链式构造器。
 *
 * Builder 同时提供两类能力：
 * - 粗粒度覆盖：整块替换 detection/beam_control/tracking/lifecycle/environment
 * - 叶子字段覆盖：直接修改高频参数，Build() 后无需再手改
 *
 * @code
 * auto config = RadarSessionConfigBuilder(MakeDetectionMissionRadarSessionConfig())
 *                   .Detection()
 *                   .EnablePhysicsDetection(true)
 *                   .WithPeakPowerW(5e6f)
 *                   .WithFrequencyHz(9.3e9f)
 *                   .End()
 *                   .Environment()
 *                   .WithJammingDetectionThresholdDb(5.0f)
 *                   .End()
 *                   .Build();
 * @endcode
 */
class ONEQ_API RadarSessionConfigBuilder {
 public:
  class DetectionEditor;
  class BeamEditor;
  class TrackingEditor;
  class LifecycleEditor;
  class EnvironmentEditor;

  explicit RadarSessionConfigBuilder(const session::RadarSessionConfig& config = {})
      : config_(config) {}

  DetectionEditor Detection();
  BeamEditor Beam();
  TrackingEditor Tracking();
  LifecycleEditor Lifecycle();
  EnvironmentEditor Environment();

  session::RadarSessionConfig Build() const;

 private:
  friend class DetectionEditor;
  friend class BeamEditor;
  friend class TrackingEditor;
  friend class LifecycleEditor;
  friend class EnvironmentEditor;

  session::RadarSessionConfig config_;
};

class RadarSessionConfigBuilder::DetectionEditor {
 public:
  explicit DetectionEditor(RadarSessionConfigBuilder* builder) : builder_(builder) {}

  DetectionEditor& WithDetection(const SignalDetectionConfig& detection) {
    builder_->config_.detection = detection;
    return *this;
  }
  DetectionEditor& EnablePhysicsDetection(bool enable = true) {
    builder_->config_.detection.enable_physics_detection = enable;
    return *this;
  }
  DetectionEditor& WithHardwareProfile(RadarHardwareProfile profile) {
    builder_->config_.detection.hardware_profile = profile;
    return *this;
  }
  DetectionEditor& WithDetectionIntentProfile(DetectionIntentProfile profile) {
    builder_->config_.detection.intent_profile = profile;
    return *this;
  }
  DetectionEditor& WithAntennaPatternProfile(AntennaPatternProfile profile) {
    builder_->config_.detection.antenna_pattern.profile = profile;
    return *this;
  }
  DetectionEditor& WithRcsFusionProfile(RcsFusionProfile profile) {
    builder_->config_.detection.rcs_fusion_profile = profile;
    return *this;
  }
  DetectionEditor& WithMinDetectionMarginDb(float margin_db) {
    builder_->config_.detection.min_detection_margin_db = margin_db;
    return *this;
  }

  RadarSessionConfigBuilder& End() { return *builder_; }

 private:
  RadarSessionConfigBuilder* builder_;
};

class RadarSessionConfigBuilder::BeamEditor {
 public:
  explicit BeamEditor(RadarSessionConfigBuilder* builder) : builder_(builder) {}

  BeamEditor& WithBeamControl(const SignalBeamControlConfig& beam_control) {
    builder_->config_.beam_control = beam_control;
    return *this;
  }
  BeamEditor& WithRadarWorkSubMode(RadarWorkSubMode work_sub_mode) {
    builder_->config_.beam_control.radar_orientation.work_sub_mode = work_sub_mode;
    return *this;
  }
  BeamEditor& WithScanCenterDeg(const AzimuthElevationDeg& scan_center_deg) {
    builder_->config_.beam_control.radar_orientation.scan_center_deg = scan_center_deg;
    return *this;
  }
  BeamEditor& WithDwellCenterDeg(const AzimuthElevationDeg& dwell_center_deg) {
    builder_->config_.beam_control.radar_orientation.dwell_center_deg = dwell_center_deg;
    return *this;
  }
  BeamEditor& EnableCommandedBeamwidth(bool enable = true) {
    builder_->config_.beam_control.radar_orientation.commanded_beamwidth_enabled = enable;
    return *this;
  }
  BeamEditor& WithCommandedBeamwidthDeg(const CommandedBeamwidthDeg& beamwidth_deg) {
    builder_->config_.beam_control.radar_orientation.commanded_beamwidth_deg = beamwidth_deg;
    return *this;
  }

  RadarSessionConfigBuilder& End() { return *builder_; }

 private:
  RadarSessionConfigBuilder* builder_;
};

class RadarSessionConfigBuilder::TrackingEditor {
 public:
  explicit TrackingEditor(RadarSessionConfigBuilder* builder) : builder_(builder) {}

  TrackingEditor& WithTracking(const SignalTrackingConfig& tracking) {
    builder_->config_.tracking = tracking;
    return *this;
  }
  TrackingEditor& EnableTrackingFilter(bool enable = true) {
    builder_->config_.tracking.enable_tracking_filter = enable;
    return *this;
  }
  TrackingEditor& WithTrackingPolicyProfile(TrackingPolicyProfile profile) {
    builder_->config_.tracking.policy_profile = profile;
    return *this;
  }

  RadarSessionConfigBuilder& End() { return *builder_; }

 private:
  RadarSessionConfigBuilder* builder_;
};

class RadarSessionConfigBuilder::LifecycleEditor {
 public:
  explicit LifecycleEditor(RadarSessionConfigBuilder* builder) : builder_(builder) {}

  LifecycleEditor& WithLifecycle(const SignalLifecycleConfig& lifecycle) {
    builder_->config_.lifecycle = lifecycle;
    return *this;
  }
  LifecycleEditor& EnableImmFusion(bool enable = true) {
    builder_->config_.lifecycle.enable_imm_fusion = enable;
    return *this;
  }
  LifecycleEditor& WithLifecyclePolicyProfile(LifecyclePolicyProfile profile) {
    builder_->config_.lifecycle.policy_profile = profile;
    return *this;
  }

  RadarSessionConfigBuilder& End() { return *builder_; }

 private:
  RadarSessionConfigBuilder* builder_;
};

class RadarSessionConfigBuilder::EnvironmentEditor {
 public:
  explicit EnvironmentEditor(RadarSessionConfigBuilder* builder) : builder_(builder) {}

  EnvironmentEditor& WithEnvironmentDefault(const environment::EnvironmentDefaultConfig& env) {
    builder_->config_.environment_default_config = env;
    return *this;
  }
  EnvironmentEditor& WithJammingDetectionThresholdDb(float threshold_db) {
    builder_->config_.environment_default_config.jamming_detection_threshold_db = threshold_db;
    return *this;
  }

  RadarSessionConfigBuilder& End() { return *builder_; }

 private:
  RadarSessionConfigBuilder* builder_;
};

inline RadarSessionConfigBuilder::DetectionEditor RadarSessionConfigBuilder::Detection() {
  return DetectionEditor(this);
}

inline RadarSessionConfigBuilder::BeamEditor RadarSessionConfigBuilder::Beam() {
  return BeamEditor(this);
}

inline RadarSessionConfigBuilder::TrackingEditor RadarSessionConfigBuilder::Tracking() {
  return TrackingEditor(this);
}

inline RadarSessionConfigBuilder::LifecycleEditor RadarSessionConfigBuilder::Lifecycle() {
  return LifecycleEditor(this);
}

inline RadarSessionConfigBuilder::EnvironmentEditor RadarSessionConfigBuilder::Environment() {
  return EnvironmentEditor(this);
}

}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_BUILDER_H_
