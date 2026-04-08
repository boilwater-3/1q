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
  DetectionEditor& WithMinDetectionMarginDb(float margin_db) {
    builder_->config_.detection.min_detection_margin_db = margin_db;
    return *this;
  }
  DetectionEditor& WithPulseCount(int pulse_count) {
    builder_->config_.detection.pulse_count = pulse_count;
    return *this;
  }
  DetectionEditor& WithTransmitterConfig(const TransmitterConfig& transmitter) {
    builder_->config_.detection.transmitter = transmitter;
    return *this;
  }
  DetectionEditor& WithAntennaConfig(const AntennaConfig& antenna) {
    builder_->config_.detection.antenna = antenna;
    return *this;
  }
  DetectionEditor& WithReceiverConfig(const ReceiverConfig& receiver) {
    builder_->config_.detection.receiver = receiver;
    return *this;
  }
  DetectionEditor& WithDetectionPolicy(const DetectionPolicy& detection_policy) {
    builder_->config_.detection.detection_policy = detection_policy;
    return *this;
  }
  DetectionEditor& WithPeakPowerW(float peak_power_w) {
    builder_->config_.detection.transmitter.peak_power_w = peak_power_w;
    return *this;
  }
  DetectionEditor& WithFrequencyHz(float frequency_hz) {
    builder_->config_.detection.transmitter.frequency_hz = frequency_hz;
    return *this;
  }
  DetectionEditor& WithBandwidthHz(float bandwidth_hz) {
    builder_->config_.detection.transmitter.bandwidth_hz = bandwidth_hz;
    return *this;
  }
  DetectionEditor& WithPulseWidthS(float pulse_width_s) {
    builder_->config_.detection.transmitter.pulse_width_s = pulse_width_s;
    return *this;
  }
  DetectionEditor& WithPrfHz(float prf_hz) {
    builder_->config_.detection.transmitter.prf_hz = prf_hz;
    return *this;
  }
  DetectionEditor& WithMainBeamGainDb(float main_beam_gain_db) {
    builder_->config_.detection.antenna.main_beam_gain_db = main_beam_gain_db;
    return *this;
  }
  DetectionEditor& WithNoiseFigureDb(float noise_figure_db) {
    builder_->config_.detection.receiver.noise_figure_db = noise_figure_db;
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
  TrackingEditor& EnableKalmanFilter(bool enable = true) {
    builder_->config_.tracking.enable_kalman_filter = enable;
    return *this;
  }
  TrackingEditor& WithKalmanMeasurementNoiseStd(float stddev) {
    builder_->config_.tracking.kalman_measurement_noise_std = stddev;
    return *this;
  }
  TrackingEditor& WithKalmanUpdateBackend(KalmanUpdateBackend backend) {
    builder_->config_.tracking.kalman_update_backend = backend;
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
  LifecycleEditor& WithLifecycleConfirmHits(std::uint32_t confirm_hits) {
    builder_->config_.lifecycle.lifecycle_config.confirm_hits = confirm_hits;
    return *this;
  }
  LifecycleEditor& WithLifecycleMaxMissBeforeLost(std::uint32_t max_miss_before_lost) {
    builder_->config_.lifecycle.lifecycle_config.max_miss_before_lost = max_miss_before_lost;
    return *this;
  }
  LifecycleEditor& WithLifecycleMaxLostCycles(std::uint32_t max_lost_cycles) {
    builder_->config_.lifecycle.lifecycle_config.max_lost_cycles = max_lost_cycles;
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
