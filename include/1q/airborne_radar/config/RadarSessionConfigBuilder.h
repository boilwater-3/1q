/**
 * @file RadarSessionConfigBuilder.h
 * @brief 提供链式构造 RadarSessionConfig 的 Builder（用于会话初始化基线配置）。
 */

#ifndef AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_BUILDER_H_
#define AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_BUILDER_H_

#include "1q/airborne_radar/config/RadarSessionConfig.h"
#include "1q/airborne_radar/config/semantic/BeamControlConfig.h"
#include "1q/airborne_radar/config/semantic/DetectionConfig.h"
#include "1q/airborne_radar/config/semantic/LifecycleConfig.h"
#include "1q/airborne_radar/config/semantic/TrackingConfig.h"
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
 *                   .WithHardwareProfile(
 *                       semantic::RadarHardwareProfile::kLongRangeHighPower)
 *                   .WithDetectionIntentProfile(
 *                       semantic::DetectionIntentProfile::kDetectionPriority)
 *                   .End()
 *                   .Environment()
 *                   .WithJammingSensitivityProfile(
 *                       environment::JammingSensitivityProfile::kStrict)
 *                   .End()
 *                   .Build();
 * @endcode
 */
class ONEQ_API RadarSessionConfigBuilder {
 public:
  /** @brief 语义探测配置编辑器。 */
  class DetectionEditor;
  /** @brief 语义波束控制配置编辑器。 */
  class BeamEditor;
  /** @brief 语义跟踪配置编辑器。 */
  class TrackingEditor;
  /** @brief 语义生命周期配置编辑器。 */
  class LifecycleEditor;
  /** @brief 环境默认配置编辑器。 */
  class EnvironmentEditor;

  /**
   * @brief 使用现有会话配置初始化 Builder。
   * @param config 作为编辑基线的会话配置。
   */
  explicit RadarSessionConfigBuilder(const session::RadarSessionConfig& config = {})
      : config_(config) {}

  /** @brief 进入探测配置编辑域。 */
  DetectionEditor Detection();
  /** @brief 进入波束控制配置编辑域。 */
  BeamEditor Beam();
  /** @brief 进入跟踪配置编辑域。 */
  TrackingEditor Tracking();
  /** @brief 进入生命周期配置编辑域。 */
  LifecycleEditor Lifecycle();
  /** @brief 进入环境默认配置编辑域。 */
  EnvironmentEditor Environment();

  /**
   * @brief 生成最终会话配置。
   * @return 构建完成的 `RadarSessionConfig`。
   */
  session::RadarSessionConfig Build() const;

 private:
  friend class DetectionEditor;
  friend class BeamEditor;
  friend class TrackingEditor;
  friend class LifecycleEditor;
  friend class EnvironmentEditor;

  session::RadarSessionConfig config_;
};

/**
 * @brief 语义探测配置编辑器。
 */
class RadarSessionConfigBuilder::DetectionEditor {
 public:
  /**
   * @brief 绑定所属 Builder。
   * @param builder 目标 Builder。
   */
  explicit DetectionEditor(RadarSessionConfigBuilder* builder) : builder_(builder) {}

  /** @brief 整块替换 semantic 探测配置。 */
  DetectionEditor& WithDetection(const semantic::DetectionConfig& detection) {
    builder_->config_.detection = detection;
    return *this;
  }
  /** @brief 开启或关闭物理探测链路。 */
  DetectionEditor& EnablePhysicsDetection(bool enable = true) {
    builder_->config_.detection.enable_physics_detection = enable;
    return *this;
  }
  /** @brief 设置硬件语义档位。 */
  DetectionEditor& WithHardwareProfile(semantic::RadarHardwareProfile profile) {
    builder_->config_.detection.hardware_profile = profile;
    return *this;
  }
  /** @brief 设置探测意图语义档位。 */
  DetectionEditor& WithDetectionIntentProfile(semantic::DetectionIntentProfile profile) {
    builder_->config_.detection.intent_profile = profile;
    return *this;
  }
  /** @brief 设置方向图语义档位。 */
  DetectionEditor& WithAntennaPatternProfile(semantic::AntennaPatternProfile profile) {
    builder_->config_.detection.antenna_pattern.profile = profile;
    return *this;
  }
  /** @brief 设置 RCS 融合语义档位。 */
  DetectionEditor& WithRcsFusionProfile(semantic::RcsFusionProfile profile) {
    builder_->config_.detection.rcs_fusion_profile = profile;
    return *this;
  }

  RadarSessionConfigBuilder& End() { return *builder_; }

 private:
  RadarSessionConfigBuilder* builder_;
};

/**
 * @brief 语义波束控制配置编辑器。
 */
class RadarSessionConfigBuilder::BeamEditor {
 public:
  /**
   * @brief 绑定所属 Builder。
   * @param builder 目标 Builder。
   */
  explicit BeamEditor(RadarSessionConfigBuilder* builder) : builder_(builder) {}

  /** @brief 整块替换 semantic 波束控制配置。 */
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

  RadarSessionConfigBuilder& End() { return *builder_; }

 private:
  RadarSessionConfigBuilder* builder_;
};

/**
 * @brief 语义跟踪配置编辑器。
 */
class RadarSessionConfigBuilder::TrackingEditor {
 public:
  /**
   * @brief 绑定所属 Builder。
   * @param builder 目标 Builder。
   */
  explicit TrackingEditor(RadarSessionConfigBuilder* builder) : builder_(builder) {}

  /** @brief 整块替换 semantic 跟踪配置。 */
  TrackingEditor& WithTracking(const semantic::TrackingConfig& tracking) {
    builder_->config_.tracking = tracking;
    return *this;
  }
  /** @brief 开启或关闭公开跟踪滤波链路。 */
  TrackingEditor& EnableTrackingFilter(bool enable = true) {
    builder_->config_.tracking.enable_tracking_filter = enable;
    return *this;
  }
  /** @brief 设置跟踪策略语义档位。 */
  TrackingEditor& WithTrackingPolicyProfile(semantic::TrackingPolicyProfile profile) {
    builder_->config_.tracking.policy_profile = profile;
    return *this;
  }

  RadarSessionConfigBuilder& End() { return *builder_; }

 private:
  RadarSessionConfigBuilder* builder_;
};

/**
 * @brief 语义生命周期配置编辑器。
 */
class RadarSessionConfigBuilder::LifecycleEditor {
 public:
  /**
   * @brief 绑定所属 Builder。
   * @param builder 目标 Builder。
   */
  explicit LifecycleEditor(RadarSessionConfigBuilder* builder) : builder_(builder) {}

  /** @brief 整块替换 semantic 生命周期配置。 */
  LifecycleEditor& WithLifecycle(const semantic::LifecycleConfig& lifecycle) {
    builder_->config_.lifecycle = lifecycle;
    return *this;
  }
  /** @brief 开启或关闭 IMM 融合。 */
  LifecycleEditor& EnableImmFusion(bool enable = true) {
    builder_->config_.lifecycle.enable_imm_fusion = enable;
    return *this;
  }
  /** @brief 设置生命周期策略语义档位。 */
  LifecycleEditor& WithLifecyclePolicyProfile(semantic::LifecyclePolicyProfile profile) {
    builder_->config_.lifecycle.policy_profile = profile;
    return *this;
  }

  RadarSessionConfigBuilder& End() { return *builder_; }

 private:
  RadarSessionConfigBuilder* builder_;
};

/**
 * @brief 环境默认配置编辑器。
 */
class RadarSessionConfigBuilder::EnvironmentEditor {
 public:
  /**
   * @brief 绑定所属 Builder。
   * @param builder 目标 Builder。
   */
  explicit EnvironmentEditor(RadarSessionConfigBuilder* builder) : builder_(builder) {}

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
