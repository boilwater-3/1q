/**
 * @file RadarExpertSessionConfigBuilder.h
 * @brief 提供链式构造 RadarSessionConfig 的专家级 Builder。
 */

#ifndef AIRBORNE_RADAR_CONFIG_RADAR_EXPERT_SESSION_CONFIG_BUILDER_H_
#define AIRBORNE_RADAR_CONFIG_RADAR_EXPERT_SESSION_CONFIG_BUILDER_H_

#include "1q/airborne_radar/config/RadarSessionConfig.h"
#include "1q/airborne_radar/config/semantic/DetectionProfiles.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {

using model::AzimuthElevationDeg;
using model::CommandedBeamwidthDeg;
using model::RadarWorkSubMode;

/**
 * @brief RadarSession 专家级配置链式构造器。
 *
 * 直接操作 `expert::ExpertPipelineConfig` 物理参数，
 * `Build()` 返回值中 `pipeline_config` 仅含 `expert` + `orientation`。
 */
class ONEQ_API RadarExpertSessionConfigBuilder {
 public:
  /** @brief expert 探测配置编辑器。 */
  class DetectionEditor;
  /** @brief 波束控制配置编辑器。 */
  class BeamEditor;
  /** @brief expert 跟踪配置编辑器。 */
  class TrackingEditor;
  /** @brief expert 生命周期配置编辑器。 */
  class LifecycleEditor;
  /** @brief 环境默认配置编辑器。 */
  class EnvironmentEditor;

  /**
   * @brief 使用现有会话配置初始化专家 Builder。
   * @param config 作为编辑基线的会话配置。
   */
  explicit RadarExpertSessionConfigBuilder(const session::RadarSessionConfig& config = {})
      : config_(config) {}

  /** @brief 进入 expert 探测配置编辑域。 */
  DetectionEditor Detection();
  /** @brief 进入波束控制配置编辑域。 */
  BeamEditor Beam();
  /** @brief 进入 expert 跟踪配置编辑域。 */
  TrackingEditor Tracking();
  /** @brief 进入 expert 生命周期配置编辑域。 */
  LifecycleEditor Lifecycle();
  /** @brief 进入环境默认配置编辑域。 */
  EnvironmentEditor Environment();

  /**
   * @brief 生成最终专家级会话配置。
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
 * @brief expert 探测配置编辑器。
 */
class RadarExpertSessionConfigBuilder::DetectionEditor {
 public:
  explicit DetectionEditor(RadarExpertSessionConfigBuilder* builder) : builder_(builder) {}

  /** @brief 整块替换 expert 探测配置。 */
  DetectionEditor& WithDetection(const expert::DetectionConfig& detection) {
    builder_->config_.pipeline_config.expert.detection = detection;
    return *this;
  }
  /** @brief 开启或关闭物理探测链路。 */
  DetectionEditor& EnablePhysicsDetection(bool enable = true) {
    builder_->config_.pipeline_config.expert.detection.enable_physics_detection = enable;
    return *this;
  }
  /** @brief 设置峰值发射功率。 */
  DetectionEditor& WithPeakPowerW(float peak_power_w) {
    builder_->config_.pipeline_config.expert.detection.transmitter.peak_power_w = peak_power_w;
    return *this;
  }
  /** @brief 设置工作频率。 */
  DetectionEditor& WithFrequencyHz(float frequency_hz) {
    builder_->config_.pipeline_config.expert.detection.transmitter.frequency_hz = frequency_hz;
    return *this;
  }
  /** @brief 设置发射带宽。 */
  DetectionEditor& WithBandwidthHz(float bandwidth_hz) {
    builder_->config_.pipeline_config.expert.detection.transmitter.bandwidth_hz = bandwidth_hz;
    return *this;
  }
  /** @brief 设置脉宽。 */
  DetectionEditor& WithPulseWidthS(float pulse_width_s) {
    builder_->config_.pipeline_config.expert.detection.transmitter.pulse_width_s = pulse_width_s;
    return *this;
  }
  /** @brief 设置脉冲重复频率。 */
  DetectionEditor& WithPrfHz(float prf_hz) {
    builder_->config_.pipeline_config.expert.detection.transmitter.prf_hz = prf_hz;
    return *this;
  }
  /** @brief 设置主瓣峰值增益。 */
  DetectionEditor& WithMainBeamGainDb(float gain_db) {
    builder_->config_.pipeline_config.expert.detection.antenna.main_beam_gain_db = gain_db;
    return *this;
  }
  /** @brief 设置接收机噪声系数。 */
  DetectionEditor& WithNoiseFigureDb(float noise_figure_db) {
    builder_->config_.pipeline_config.expert.detection.receiver.noise_figure_db = noise_figure_db;
    return *this;
  }
  /** @brief 设置最小探测裕量。 */
  DetectionEditor& WithMinDetectionMarginDb(float margin_db) {
    builder_->config_.pipeline_config.expert.detection.min_detection_margin_db = margin_db;
    return *this;
  }
  /** @brief 设置脉冲积累数。 */
  DetectionEditor& WithPulseCount(int pulse_count) {
    builder_->config_.pipeline_config.expert.detection.pulse_count = pulse_count;
    return *this;
  }
  /** @brief 设置目标起伏模型。 */
  DetectionEditor& WithSwerlingModel(semantic::SwerlingModel swerling_model) {
    builder_->config_.pipeline_config.expert.detection.swerling_model = swerling_model;
    return *this;
  }

  RadarExpertSessionConfigBuilder& End() { return *builder_; }

 private:
  RadarExpertSessionConfigBuilder* builder_;
};

/**
 * @brief 波束控制配置编辑器。
 *
 * 专家 Builder 的波束编辑器操作两条路径：
 * - `pipeline_config.orientation`（方向与扫描运行态，支持运行时热更新）。
 * - `pipeline_config.expert.beam_control`（expert 调度与指向参数）。
 */
class RadarExpertSessionConfigBuilder::BeamEditor {
 public:
  explicit BeamEditor(RadarExpertSessionConfigBuilder* builder) : builder_(builder) {}

  /** @brief 整块替换 expert 波束控制配置（调度与指向入口）。 */
  BeamEditor& WithExpertBeamControl(const expert::BeamControlConfig& beam_control) {
    builder_->config_.pipeline_config.expert.beam_control = beam_control;
    builder_->config_.pipeline_config.orientation.scan_center_deg =
        beam_control.pointing.default_scan_center_deg;
    builder_->config_.pipeline_config.orientation.commanded_beamwidth_deg =
        beam_control.pointing.nominal_beamwidth_deg;
    return *this;
  }
  /** @brief 设置雷达工作子模式。 */
  BeamEditor& WithRadarWorkSubMode(RadarWorkSubMode work_sub_mode) {
    builder_->config_.pipeline_config.orientation.work_sub_mode = work_sub_mode;
    return *this;
  }
  /** @brief 设置扫描中心。 */
  BeamEditor& WithScanCenterDeg(const AzimuthElevationDeg& scan_center_deg) {
    builder_->config_.pipeline_config.orientation.scan_center_deg = scan_center_deg;
    return *this;
  }
  /** @brief 更新指令态波束使能。 */
  BeamEditor& EnableCommandedBeamwidth(bool enable = true) {
    builder_->config_.pipeline_config.orientation.commanded_beamwidth_enabled = enable;
    return *this;
  }
  /** @brief 设置指令态波束宽度。 */
  BeamEditor& WithCommandedBeamwidthDeg(const CommandedBeamwidthDeg& beamwidth_deg) {
    builder_->config_.pipeline_config.orientation.commanded_beamwidth_deg = beamwidth_deg;
    return *this;
  }
  /** @brief 设置方位步进数提示（expert 调度路径）。 */
  BeamEditor& WithAzimuthStepCountHint(std::uint32_t count) {
    builder_->config_.pipeline_config.expert.beam_control.scheduler.azimuth_step_count_hint =
        count;
    return *this;
  }
  /** @brief 设置俯仰步进数提示（expert 调度路径）。 */
  BeamEditor& WithElevationStepCountHint(std::uint32_t count) {
    builder_->config_.pipeline_config.expert.beam_control.scheduler.elevation_step_count_hint =
        count;
    return *this;
  }
  /** @brief 设置是否偏好更密的 TAS 采样（expert 调度路径）。 */
  BeamEditor& PreferDenseTasSampling(bool prefer = true) {
    builder_->config_.pipeline_config.expert.beam_control.scheduler.prefer_dense_tas_sampling =
        prefer;
    return *this;
  }
  /** @brief 设置默认扫描中心（expert 指向路径）。 */
  BeamEditor& WithDefaultScanCenterDeg(const AzimuthElevationDeg& scan_center_deg) {
    builder_->config_.pipeline_config.expert.beam_control.pointing.default_scan_center_deg =
        scan_center_deg;
    builder_->config_.pipeline_config.orientation.scan_center_deg = scan_center_deg;
    return *this;
  }
  /** @brief 设置名义指令态波束宽度（expert 指向路径）。 */
  BeamEditor& WithNominalBeamwidthDeg(const CommandedBeamwidthDeg& beamwidth_deg) {
    builder_->config_.pipeline_config.expert.beam_control.pointing.nominal_beamwidth_deg =
        beamwidth_deg;
    builder_->config_.pipeline_config.orientation.commanded_beamwidth_deg = beamwidth_deg;
    return *this;
  }

  RadarExpertSessionConfigBuilder& End() { return *builder_; }

 private:
  RadarExpertSessionConfigBuilder* builder_;
};

/**
 * @brief expert 跟踪配置编辑器。
 */
class RadarExpertSessionConfigBuilder::TrackingEditor {
 public:
  explicit TrackingEditor(RadarExpertSessionConfigBuilder* builder) : builder_(builder) {}

  /** @brief 整块替换 expert 跟踪配置。 */
  TrackingEditor& WithTracking(const expert::TrackingConfig& tracking) {
    builder_->config_.pipeline_config.expert.tracking = tracking;
    return *this;
  }
  /** @brief 开启或关闭 Kalman 滤波。 */
  TrackingEditor& EnableKalmanFilter(bool enable = true) {
    builder_->config_.pipeline_config.expert.tracking.enable_kalman_filter = enable;
    return *this;
  }
  /** @brief 设置 Kalman 量测噪声标准差。 */
  TrackingEditor& WithKalmanMeasurementNoiseStd(float stddev) {
    builder_->config_.pipeline_config.expert.tracking.kalman_measurement_noise_std = stddev;
    return *this;
  }
  /** @brief 设置 Kalman 更新后端。 */
  TrackingEditor& WithKalmanUpdateBackend(expert::KalmanUpdateBackend backend) {
    builder_->config_.pipeline_config.expert.tracking.kalman_update_backend = backend;
    return *this;
  }
  /** @brief 设置丢失周期速度衰减系数。 */
  TrackingEditor& WithSpeedDecayRatioOnLoss(float ratio) {
    builder_->config_.pipeline_config.expert.tracking.speed_decay_ratio_on_loss = ratio;
    return *this;
  }
  /** @brief 设置丢失周期 RCS 衰减系数。 */
  TrackingEditor& WithRcsDecayRatioOnLoss(float ratio) {
    builder_->config_.pipeline_config.expert.tracking.rcs_decay_ratio_on_loss = ratio;
    return *this;
  }

  RadarExpertSessionConfigBuilder& End() { return *builder_; }

 private:
  RadarExpertSessionConfigBuilder* builder_;
};

/**
 * @brief expert 生命周期配置编辑器。
 */
class RadarExpertSessionConfigBuilder::LifecycleEditor {
 public:
  explicit LifecycleEditor(RadarExpertSessionConfigBuilder* builder) : builder_(builder) {}

  /** @brief 整块替换 expert 生命周期配置。 */
  LifecycleEditor& WithLifecycle(const expert::LifecycleConfig& lifecycle) {
    builder_->config_.pipeline_config.expert.lifecycle = lifecycle;
    return *this;
  }
  /** @brief 开启或关闭 IMM 生命周期路径。 */
  LifecycleEditor& EnableImmFusion(bool enable = true) {
    builder_->config_.pipeline_config.expert.lifecycle.enable_imm_lifecycle = enable;
    return *this;
  }
  /** @brief 设置确认命中数。 */
  LifecycleEditor& WithLifecycleConfirmHits(std::uint32_t hits) {
    builder_->config_.pipeline_config.expert.lifecycle.confirm_hits = hits;
    return *this;
  }
  /** @brief 设置 lost 前允许的连续失配数。 */
  LifecycleEditor& WithLifecycleMaxMissBeforeLost(std::uint32_t misses) {
    builder_->config_.pipeline_config.expert.lifecycle.max_miss_before_lost = misses;
    return *this;
  }
  /** @brief 设置最大 lost 保留周期数。 */
  LifecycleEditor& WithLifecycleMaxLostCycles(std::uint32_t cycles) {
    builder_->config_.pipeline_config.expert.lifecycle.max_lost_cycles = cycles;
    return *this;
  }

  RadarExpertSessionConfigBuilder& End() { return *builder_; }

 private:
  RadarExpertSessionConfigBuilder* builder_;
};

/**
 * @brief 环境默认配置编辑器。
 */
class RadarExpertSessionConfigBuilder::EnvironmentEditor {
 public:
  explicit EnvironmentEditor(RadarExpertSessionConfigBuilder* builder) : builder_(builder) {}

  /** @brief 整块替换环境默认配置。 */
  EnvironmentEditor& WithEnvironmentDefault(const environment::EnvironmentDefaultConfig& env) {
    builder_->config_.environment_default_config = env;
    return *this;
  }
  /** @brief 设置干扰判定灵敏度语义档位。 */
  EnvironmentEditor& WithJammingSensitivityProfile(
      environment::JammingSensitivityProfile profile) {
    builder_->config_.environment_default_config.jamming_sensitivity_profile = profile;
    return *this;
  }

  RadarExpertSessionConfigBuilder& End() { return *builder_; }

 private:
  RadarExpertSessionConfigBuilder* builder_;
};

inline RadarExpertSessionConfigBuilder::DetectionEditor
RadarExpertSessionConfigBuilder::Detection() {
  return DetectionEditor(this);
}

inline RadarExpertSessionConfigBuilder::BeamEditor RadarExpertSessionConfigBuilder::Beam() {
  return BeamEditor(this);
}

inline RadarExpertSessionConfigBuilder::TrackingEditor RadarExpertSessionConfigBuilder::Tracking() {
  return TrackingEditor(this);
}

inline RadarExpertSessionConfigBuilder::LifecycleEditor
RadarExpertSessionConfigBuilder::Lifecycle() {
  return LifecycleEditor(this);
}

inline RadarExpertSessionConfigBuilder::EnvironmentEditor
RadarExpertSessionConfigBuilder::Environment() {
  return EnvironmentEditor(this);
}

}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_RADAR_EXPERT_SESSION_CONFIG_BUILDER_H_
