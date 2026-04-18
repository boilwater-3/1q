/**
 * @file RadarExpertSessionConfigBuilder.h
 * @brief 提供链式构造 RadarSessionConfig 的专家级 Builder。
 */

#ifndef AIRBORNE_RADAR_CONFIG_RADAR_EXPERT_SESSION_CONFIG_BUILDER_H_
#define AIRBORNE_RADAR_CONFIG_RADAR_EXPERT_SESSION_CONFIG_BUILDER_H_

#include "1q/airborne_radar/config/semantic/BeamControlConfig.h"
#include "1q/airborne_radar/config/RadarSessionConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {

using model::AzimuthElevationDeg;
using model::CommandedBeamwidthDeg;
using model::RadarWorkSubMode;

/**
 * @brief RadarSession 专家级配置链式构造器。
 * @note 该 Builder 构造出的会话配置会将 `pipeline_config_model` 设为 `kExpert`。
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
      : config_(config) {
    config_.pipeline_config_model = PipelineConfigModel::kExpert;
  }

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
  /**
   * @brief 绑定所属 Builder。
   * @param builder 目标 Builder。
   */
  explicit DetectionEditor(RadarExpertSessionConfigBuilder* builder) : builder_(builder) {}

  /** @brief 整块替换 expert 探测配置。 */
  DetectionEditor& WithDetection(const expert::DetectionConfig& detection) {
    builder_->config_.expert_pipeline_config.detection = detection;
    return *this;
  }
  /** @brief 开启或关闭物理探测链路。 */
  DetectionEditor& EnablePhysicsDetection(bool enable = true) {
    builder_->config_.expert_pipeline_config.detection.enable_physics_detection = enable;
    return *this;
  }
  /** @brief 设置峰值发射功率。 */
  DetectionEditor& WithPeakPowerW(float peak_power_w) {
    builder_->config_.expert_pipeline_config.detection.transmitter.peak_power_w = peak_power_w;
    return *this;
  }
  /** @brief 设置工作频率。 */
  DetectionEditor& WithFrequencyHz(float frequency_hz) {
    builder_->config_.expert_pipeline_config.detection.transmitter.frequency_hz = frequency_hz;
    return *this;
  }
  /** @brief 设置发射带宽。 */
  DetectionEditor& WithBandwidthHz(float bandwidth_hz) {
    builder_->config_.expert_pipeline_config.detection.transmitter.bandwidth_hz = bandwidth_hz;
    return *this;
  }
  /** @brief 设置脉宽。 */
  DetectionEditor& WithPulseWidthS(float pulse_width_s) {
    builder_->config_.expert_pipeline_config.detection.transmitter.pulse_width_s = pulse_width_s;
    return *this;
  }
  /** @brief 设置脉冲重复频率。 */
  DetectionEditor& WithPrfHz(float prf_hz) {
    builder_->config_.expert_pipeline_config.detection.transmitter.prf_hz = prf_hz;
    return *this;
  }
  /** @brief 设置主瓣峰值增益。 */
  DetectionEditor& WithMainBeamGainDb(float gain_db) {
    builder_->config_.expert_pipeline_config.detection.antenna.main_beam_gain_db = gain_db;
    return *this;
  }
  /** @brief 设置接收机噪声系数。 */
  DetectionEditor& WithNoiseFigureDb(float noise_figure_db) {
    builder_->config_.expert_pipeline_config.detection.receiver.noise_figure_db = noise_figure_db;
    return *this;
  }
  /** @brief 设置最小探测裕量。 */
  DetectionEditor& WithMinDetectionMarginDb(float margin_db) {
    builder_->config_.expert_pipeline_config.detection.min_detection_margin_db = margin_db;
    return *this;
  }
  /** @brief 设置脉冲积累数。 */
  DetectionEditor& WithPulseCount(int pulse_count) {
    builder_->config_.expert_pipeline_config.detection.pulse_count = pulse_count;
    return *this;
  }
  /** @brief 设置目标起伏模型。 */
  DetectionEditor& WithSwerlingModel(semantic::SwerlingModel swerling_model) {
    builder_->config_.expert_pipeline_config.detection.swerling_model = swerling_model;
    return *this;
  }

  RadarExpertSessionConfigBuilder& End() { return *builder_; }

 private:
  RadarExpertSessionConfigBuilder* builder_;
};

/**
 * @brief 波束控制配置编辑器。
 * @note 专家 Builder 目前仍复用公开的 `semantic::BeamControlConfig` 作为波束入口。
 */
class RadarExpertSessionConfigBuilder::BeamEditor {
 public:
  /**
   * @brief 绑定所属 Builder。
   * @param builder 目标 Builder。
   */
  explicit BeamEditor(RadarExpertSessionConfigBuilder* builder) : builder_(builder) {}

  /** @brief 整块替换波束控制配置。 */
  BeamEditor& WithBeamControl(const semantic::BeamControlConfig& beam_control) {
    builder_->config_.beam_control = beam_control;
    return *this;
  }
  /** @brief 设置雷达工作子模式。 */
  BeamEditor& WithRadarWorkSubMode(RadarWorkSubMode work_sub_mode) {
    builder_->config_.beam_control.radar_orientation.work_sub_mode = work_sub_mode;
    return *this;
  }
  /** @brief 设置扫描中心。 */
  BeamEditor& WithScanCenterDeg(const AzimuthElevationDeg& scan_center_deg) {
    builder_->config_.beam_control.radar_orientation.scan_center_deg = scan_center_deg;
    return *this;
  }
  /** @brief 更新指令态波束使能。 */
  BeamEditor& EnableCommandedBeamwidth(bool enable = true) {
    builder_->config_.beam_control.radar_orientation.commanded_beamwidth_enabled = enable;
    return *this;
  }
  /** @brief 设置指令态波束宽度。 */
  BeamEditor& WithCommandedBeamwidthDeg(const CommandedBeamwidthDeg& beamwidth_deg) {
    builder_->config_.beam_control.radar_orientation.commanded_beamwidth_deg = beamwidth_deg;
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
  /**
   * @brief 绑定所属 Builder。
   * @param builder 目标 Builder。
   */
  explicit TrackingEditor(RadarExpertSessionConfigBuilder* builder) : builder_(builder) {}

  /** @brief 整块替换 expert 跟踪配置。 */
  TrackingEditor& WithTracking(const expert::TrackingConfig& tracking) {
    builder_->config_.expert_pipeline_config.tracking = tracking;
    return *this;
  }
  /** @brief 开启或关闭 Kalman 滤波。 */
  TrackingEditor& EnableKalmanFilter(bool enable = true) {
    builder_->config_.expert_pipeline_config.tracking.enable_kalman_filter = enable;
    return *this;
  }
  /** @brief 设置 Kalman 量测噪声标准差。 */
  TrackingEditor& WithKalmanMeasurementNoiseStd(float stddev) {
    builder_->config_.expert_pipeline_config.tracking.kalman_measurement_noise_std = stddev;
    return *this;
  }
  /** @brief 设置 Kalman 更新后端。 */
  TrackingEditor& WithKalmanUpdateBackend(expert::KalmanUpdateBackend backend) {
    builder_->config_.expert_pipeline_config.tracking.kalman_update_backend = backend;
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
  /**
   * @brief 绑定所属 Builder。
   * @param builder 目标 Builder。
   */
  explicit LifecycleEditor(RadarExpertSessionConfigBuilder* builder) : builder_(builder) {}

  /** @brief 整块替换 expert 生命周期配置。 */
  LifecycleEditor& WithLifecycle(const expert::LifecycleConfig& lifecycle) {
    builder_->config_.expert_pipeline_config.lifecycle = lifecycle;
    return *this;
  }
  /** @brief 开启或关闭 IMM 生命周期路径。 */
  LifecycleEditor& EnableImmFusion(bool enable = true) {
    builder_->config_.expert_pipeline_config.lifecycle.enable_imm_lifecycle = enable;
    return *this;
  }
  /** @brief 设置确认命中数。 */
  LifecycleEditor& WithLifecycleConfirmHits(std::uint32_t hits) {
    builder_->config_.expert_pipeline_config.lifecycle.confirm_hits = hits;
    return *this;
  }
  /** @brief 设置 lost 前允许的连续失配数。 */
  LifecycleEditor& WithLifecycleMaxMissBeforeLost(std::uint32_t misses) {
    builder_->config_.expert_pipeline_config.lifecycle.max_miss_before_lost = misses;
    return *this;
  }
  /** @brief 设置最大 lost 保留周期数。 */
  LifecycleEditor& WithLifecycleMaxLostCycles(std::uint32_t cycles) {
    builder_->config_.expert_pipeline_config.lifecycle.max_lost_cycles = cycles;
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
  /**
   * @brief 绑定所属 Builder。
   * @param builder 目标 Builder。
   */
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
