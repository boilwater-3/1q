/**
 * @file RadarDetailedSessionConfigBuilder.h
 * @brief 提供链式构造 RadarSessionConfig 的详细 Builder。
 */

#ifndef AIRBORNE_RADAR_CONFIG_RADAR_DETAILED_SESSION_CONFIG_BUILDER_H_
#define AIRBORNE_RADAR_CONFIG_RADAR_DETAILED_SESSION_CONFIG_BUILDER_H_

#include <cstdint>

#include "1q/airborne_radar/config/RadarSessionConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {

using model::AzimuthElevationDeg;
using model::CommandedBeamwidthDeg;
using model::RadarWorkSubMode;

/**
 * @brief RadarSession 详细配置链式构造器。
 *
 * 直接编辑会话四域中的细粒度参数：
 * - `hardware`：探测链路固有能力
 * - `mission`：任务态波束与扫描运行态
 * - `policy`：调度、关联、跟踪与生命周期策略
 * - `environment`：环境默认配置
 */
class ONEQ_API RadarDetailedSessionConfigBuilder {
 public:
  /** @brief 探测域配置编辑器。 */
  class DetectionEditor;
  /** @brief 任务/策略波束配置编辑器。 */
  class BeamEditor;
  /** @brief 跟踪策略配置编辑器。 */
  class TrackingEditor;
  /** @brief 生命周期策略配置编辑器。 */
  class LifecycleEditor;
  /** @brief 环境默认配置编辑器。 */
  class EnvironmentEditor;

  /**
   * @brief 使用现有会话配置初始化详细 Builder。
   * @param config 作为编辑基线的会话配置。
   */
  explicit RadarDetailedSessionConfigBuilder(const session::RadarSessionConfig& config = {})
      : config_(config) {}

  /** @brief 进入探测域配置编辑器。 */
  DetectionEditor Detection();
  /** @brief 进入波束相关配置编辑器。 */
  BeamEditor Beam();
  /** @brief 进入跟踪策略配置编辑器。 */
  TrackingEditor Tracking();
  /** @brief 进入生命周期策略配置编辑器。 */
  LifecycleEditor Lifecycle();
  /** @brief 进入环境默认配置编辑器。 */
  EnvironmentEditor Environment();

  /**
   * @brief 生成最终会话配置。
   * @return 构建完成的 `RadarSessionConfig`。
   */
  session::RadarSessionConfig Build() const { return config_; }

 private:
  friend class DetectionEditor;
  friend class BeamEditor;
  friend class TrackingEditor;
  friend class LifecycleEditor;
  friend class EnvironmentEditor;

  session::RadarSessionConfig config_{};
};

/**
 * @brief 探测域配置编辑器。
 */
class RadarDetailedSessionConfigBuilder::DetectionEditor {
 public:
  explicit DetectionEditor(RadarDetailedSessionConfigBuilder* builder) : builder_(builder) {}

  /** @brief 整块替换探测配置。 */
  DetectionEditor& WithDetection(const DetectionConfig& detection) {
    builder_->config_.hardware.detection = detection;
    return *this;
  }
  /** @brief 开启或关闭物理探测链路。 */
  DetectionEditor& EnablePhysicsDetection(bool enable = true) {
    builder_->config_.hardware.detection.enable_physics_detection = enable;
    return *this;
  }
  /** @brief 设置峰值发射功率。 */
  DetectionEditor& WithPeakPowerW(float peak_power_w) {
    builder_->config_.hardware.detection.transmitter.peak_power_w = peak_power_w;
    return *this;
  }
  /** @brief 设置工作频率。 */
  DetectionEditor& WithFrequencyHz(float frequency_hz) {
    builder_->config_.hardware.detection.transmitter.frequency_hz = frequency_hz;
    return *this;
  }
  /** @brief 设置发射带宽。 */
  DetectionEditor& WithBandwidthHz(float bandwidth_hz) {
    builder_->config_.hardware.detection.transmitter.bandwidth_hz = bandwidth_hz;
    return *this;
  }
  /** @brief 设置脉宽。 */
  DetectionEditor& WithPulseWidthS(float pulse_width_s) {
    builder_->config_.hardware.detection.transmitter.pulse_width_s = pulse_width_s;
    return *this;
  }
  /** @brief 设置脉冲重复频率。 */
  DetectionEditor& WithPrfHz(float prf_hz) {
    builder_->config_.hardware.detection.transmitter.prf_hz = prf_hz;
    return *this;
  }
  /** @brief 设置主瓣峰值增益。 */
  DetectionEditor& WithMainBeamGainDb(float gain_db) {
    builder_->config_.hardware.detection.antenna.main_beam_gain_db = gain_db;
    return *this;
  }
  /** @brief 设置接收机噪声系数。 */
  DetectionEditor& WithNoiseFigureDb(float noise_figure_db) {
    builder_->config_.hardware.detection.receiver.noise_figure_db = noise_figure_db;
    return *this;
  }
  /** @brief 设置最小探测裕量。 */
  DetectionEditor& WithMinDetectionMarginDb(float margin_db) {
    builder_->config_.hardware.detection.min_detection_margin_db = margin_db;
    return *this;
  }
  /** @brief 设置脉冲积累数。 */
  DetectionEditor& WithPulseCount(int pulse_count) {
    builder_->config_.hardware.detection.pulse_count = pulse_count;
    return *this;
  }
  /** @brief 设置目标起伏模型。 */
  DetectionEditor& WithSwerlingModel(profiles::SwerlingModel swerling_model) {
    builder_->config_.hardware.detection.swerling_model = swerling_model;
    return *this;
  }

  RadarDetailedSessionConfigBuilder& End() { return *builder_; }

 private:
  RadarDetailedSessionConfigBuilder* builder_;
};

/**
 * @brief 任务/策略波束配置编辑器。
 *
 * 本编辑器同时操作两条路径：
 * - `mission.orientation`：任务态方向与扫描控制量。
 * - `policy.beam_control`：策略域调度与指向参数。
 */
class RadarDetailedSessionConfigBuilder::BeamEditor {
 public:
  explicit BeamEditor(RadarDetailedSessionConfigBuilder* builder) : builder_(builder) {}

  /** @brief 整块替换波束控制策略。 */
  BeamEditor& WithBeamControl(const BeamControlConfig& beam_control) {
    builder_->config_.policy.beam_control = beam_control;
    builder_->config_.mission.orientation.scan_center_deg =
        beam_control.pointing.default_scan_center_deg;
    builder_->config_.mission.orientation.commanded_beamwidth_deg =
        beam_control.pointing.nominal_beamwidth_deg;
    return *this;
  }
  /** @brief 设置雷达工作子模式。 */
  BeamEditor& WithRadarWorkSubMode(RadarWorkSubMode work_sub_mode) {
    builder_->config_.mission.orientation.work_sub_mode = work_sub_mode;
    return *this;
  }
  /** @brief 设置扫描中心。 */
  BeamEditor& WithScanCenterDeg(const AzimuthElevationDeg& scan_center_deg) {
    builder_->config_.mission.orientation.scan_center_deg = scan_center_deg;
    return *this;
  }
  /** @brief 更新指令态波束使能。 */
  BeamEditor& EnableCommandedBeamwidth(bool enable = true) {
    builder_->config_.mission.orientation.commanded_beamwidth_enabled = enable;
    return *this;
  }
  /** @brief 设置指令态波束宽度。 */
  BeamEditor& WithCommandedBeamwidthDeg(const CommandedBeamwidthDeg& beamwidth_deg) {
    builder_->config_.mission.orientation.commanded_beamwidth_deg = beamwidth_deg;
    return *this;
  }
  /** @brief 设置方位步进数提示（策略调度路径）。 */
  BeamEditor& WithAzimuthStepCountHint(std::uint32_t count) {
    builder_->config_.policy.beam_control.scheduler.azimuth_step_count_hint = count;
    return *this;
  }
  /** @brief 设置俯仰步进数提示（策略调度路径）。 */
  BeamEditor& WithElevationStepCountHint(std::uint32_t count) {
    builder_->config_.policy.beam_control.scheduler.elevation_step_count_hint = count;
    return *this;
  }
  /** @brief 设置是否偏好更密的 TAS 采样（策略调度路径）。 */
  BeamEditor& PreferDenseTasSampling(bool prefer = true) {
    builder_->config_.policy.beam_control.scheduler.prefer_dense_tas_sampling = prefer;
    return *this;
  }
  /** @brief 设置默认扫描中心（策略指向路径）。 */
  BeamEditor& WithDefaultScanCenterDeg(const AzimuthElevationDeg& scan_center_deg) {
    builder_->config_.policy.beam_control.pointing.default_scan_center_deg = scan_center_deg;
    builder_->config_.mission.orientation.scan_center_deg = scan_center_deg;
    return *this;
  }
  /** @brief 设置名义指令态波束宽度（策略指向路径）。 */
  BeamEditor& WithNominalBeamwidthDeg(const CommandedBeamwidthDeg& beamwidth_deg) {
    builder_->config_.policy.beam_control.pointing.nominal_beamwidth_deg = beamwidth_deg;
    builder_->config_.mission.orientation.commanded_beamwidth_deg = beamwidth_deg;
    return *this;
  }

  RadarDetailedSessionConfigBuilder& End() { return *builder_; }

 private:
  RadarDetailedSessionConfigBuilder* builder_;
};

/**
 * @brief 跟踪策略配置编辑器。
 */
class RadarDetailedSessionConfigBuilder::TrackingEditor {
 public:
  explicit TrackingEditor(RadarDetailedSessionConfigBuilder* builder) : builder_(builder) {}

  /** @brief 整块替换跟踪策略配置。 */
  TrackingEditor& WithTracking(const TrackingConfig& tracking) {
    builder_->config_.policy.tracking = tracking;
    return *this;
  }
  /** @brief 开启或关闭 Kalman 滤波。 */
  TrackingEditor& EnableKalmanFilter(bool enable = true) {
    builder_->config_.policy.tracking.enable_kalman_filter = enable;
    return *this;
  }
  /** @brief 设置 Kalman 量测噪声标准差。 */
  TrackingEditor& WithKalmanMeasurementNoiseStd(float stddev) {
    builder_->config_.policy.tracking.kalman_measurement_noise_std = stddev;
    return *this;
  }
  /** @brief 设置 Kalman 更新后端。 */
  TrackingEditor& WithKalmanUpdateBackend(KalmanUpdateBackend backend) {
    builder_->config_.policy.tracking.kalman_update_backend = backend;
    return *this;
  }
  /** @brief 设置丢失周期速度衰减系数。 */
  TrackingEditor& WithSpeedDecayRatioOnLoss(float ratio) {
    builder_->config_.policy.tracking.speed_decay_ratio_on_loss = ratio;
    return *this;
  }
  /** @brief 设置丢失周期 RCS 衰减系数。 */
  TrackingEditor& WithRcsDecayRatioOnLoss(float ratio) {
    builder_->config_.policy.tracking.rcs_decay_ratio_on_loss = ratio;
    return *this;
  }

  RadarDetailedSessionConfigBuilder& End() { return *builder_; }

 private:
  RadarDetailedSessionConfigBuilder* builder_;
};

/**
 * @brief 生命周期策略配置编辑器。
 */
class RadarDetailedSessionConfigBuilder::LifecycleEditor {
 public:
  explicit LifecycleEditor(RadarDetailedSessionConfigBuilder* builder) : builder_(builder) {}

  /** @brief 整块替换生命周期策略配置。 */
  LifecycleEditor& WithLifecycle(const LifecycleConfig& lifecycle) {
    builder_->config_.policy.lifecycle = lifecycle;
    return *this;
  }
  /** @brief 开启或关闭 IMM 生命周期路径。 */
  LifecycleEditor& EnableImmFusion(bool enable = true) {
    builder_->config_.policy.lifecycle.enable_imm_lifecycle = enable;
    return *this;
  }
  /** @brief 设置确认命中数。 */
  LifecycleEditor& WithLifecycleConfirmHits(std::uint32_t hits) {
    builder_->config_.policy.lifecycle.confirm_hits = hits;
    return *this;
  }
  /** @brief 设置 lost 前允许的连续失配数。 */
  LifecycleEditor& WithLifecycleMaxMissBeforeLost(std::uint32_t misses) {
    builder_->config_.policy.lifecycle.max_miss_before_lost = misses;
    return *this;
  }
  /** @brief 设置最大 lost 保留周期数。 */
  LifecycleEditor& WithLifecycleMaxLostCycles(std::uint32_t cycles) {
    builder_->config_.policy.lifecycle.max_lost_cycles = cycles;
    return *this;
  }

  RadarDetailedSessionConfigBuilder& End() { return *builder_; }

 private:
  RadarDetailedSessionConfigBuilder* builder_;
};

/**
 * @brief 环境默认配置编辑器。
 */
class RadarDetailedSessionConfigBuilder::EnvironmentEditor {
 public:
  explicit EnvironmentEditor(RadarDetailedSessionConfigBuilder* builder) : builder_(builder) {}

  /** @brief 整块替换环境默认配置。 */
  EnvironmentEditor& WithEnvironmentDefault(const environment::EnvironmentDefaultConfig& env) {
    builder_->config_.environment = env;
    return *this;
  }
  /** @brief 设置干扰判定灵敏度语义档位。 */
  EnvironmentEditor& WithJammingSensitivityProfile(environment::JammingSensitivityProfile profile) {
    builder_->config_.environment.jamming_sensitivity_profile = profile;
    return *this;
  }

  RadarDetailedSessionConfigBuilder& End() { return *builder_; }

 private:
  RadarDetailedSessionConfigBuilder* builder_;
};

inline RadarDetailedSessionConfigBuilder::DetectionEditor
RadarDetailedSessionConfigBuilder::Detection() {
  return DetectionEditor(this);
}

inline RadarDetailedSessionConfigBuilder::BeamEditor RadarDetailedSessionConfigBuilder::Beam() {
  return BeamEditor(this);
}

inline RadarDetailedSessionConfigBuilder::TrackingEditor
RadarDetailedSessionConfigBuilder::Tracking() {
  return TrackingEditor(this);
}

inline RadarDetailedSessionConfigBuilder::LifecycleEditor
RadarDetailedSessionConfigBuilder::Lifecycle() {
  return LifecycleEditor(this);
}

inline RadarDetailedSessionConfigBuilder::EnvironmentEditor
RadarDetailedSessionConfigBuilder::Environment() {
  return EnvironmentEditor(this);
}

}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_RADAR_DETAILED_SESSION_CONFIG_BUILDER_H_
