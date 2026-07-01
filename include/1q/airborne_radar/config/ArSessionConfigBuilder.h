/**
 * @file ArSessionConfigBuilder.h
 * @brief AR module primary session config builder type.
 *
 * Primary header for session config chain builder.
 * Include this for new code; RadarSessionConfigBuilder.h is the deprecated compat wrapper.
 */

#ifndef ONEQ_AIRBORNE_RADAR_CONFIG_AR_SESSION_CONFIG_BUILDER_H_
#define ONEQ_AIRBORNE_RADAR_CONFIG_AR_SESSION_CONFIG_BUILDER_H_

#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {

/**
 * @brief ArSessionConfig 配置链式构造器。
 *
 * Builder 通过语义档位（Profile enum）控制探测 / 跟踪 / 生命周期行为，
 * `Build()` 时将语义设定翻译为四域配置中的 hardware/policy 子域。
 * 波束方向与扫描状态（mission.orientation）及环境细项由
 * ArSessionConfig 直接编辑。
 *
 * @note 推荐路径：
 * - 会话初始化优先使用本构造器表达高层语义输入；
 * - 运行期热更新统一使用 `ArRuntimeConfigBuilder`；
 * - 仅在需要直接编辑四域细项时使用直接字段赋值。
 *
 * @code
 * auto config = ArSessionConfigBuilder()
 *                   .Detection()
 *                   .EnablePhysicsDetection(true)
 *                   .WithHardwareProfile(
 *                       profiles::ArHardwareProfile::kLongRangeHighPower)
 *                   .WithDetectionIntentProfile(
 *                       profiles::DetectionIntentProfile::kDetectionPriority)
 *                   .End()
 *                   .Environment()
 *                   .WithJammingSensitivityProfile(
 *                       config::JammingSensitivityProfile::kStrict)
 *                   .End()
 *                   .Build();
 * @endcode
 */
class ONEQ_API ArSessionConfigBuilder {
 public:
  /** @brief 语义探测配置编辑器。 */
  class DetectionEditor;
  /** @brief 语义跟踪配置编辑器。 */
  class TrackingEditor;
  /** @brief 语义生命周期配置编辑器。 */
  class LifecycleEditor;
  /** @brief 环境语义配置编辑器。 */
  class EnvironmentEditor;

  /** @brief 使用语义默认值初始化 Builder。 */
  ArSessionConfigBuilder();

  /**
   * @brief 使用现有会话配置初始化 Builder。
   *
   * 输入配置中的 `hardware/mission/policy` 会作为工程参数基线保留；
   * 仅当调用相应语义编辑器时，对应域才会被语义档位重新覆盖。
   *
   * @param config 作为编辑基线的会话配置。
   */
  explicit ArSessionConfigBuilder(const config::ArSessionConfig& config);

  /** @brief 进入探测配置编辑域。 */
  DetectionEditor Detection();
  /** @brief 进入跟踪配置编辑域。 */
  TrackingEditor Tracking();
  /** @brief 进入生命周期配置编辑域。 */
  LifecycleEditor Lifecycle();
  /** @brief 进入环境语义配置编辑域。 */
  EnvironmentEditor Environment();

  /**
   * @brief 将语义档位翻译为四域配置，生成最终会话配置。
   * @return 构建完成的 `config::ArSessionConfig`（hardware/mission/policy/environment）。
   */
  config::ArSessionConfig Build() const;

 private:
  friend class DetectionEditor;
  friend class TrackingEditor;
  friend class LifecycleEditor;
  friend class EnvironmentEditor;

  bool enable_physics_detection_{false};
  profiles::ArHardwareProfile hardware_profile_{
      profiles::ArHardwareProfile::kGenericAirborneXBand};
  profiles::DetectionIntentProfile intent_profile_{profiles::DetectionIntentProfile::kBalanced};
  profiles::AntennaPatternProfile antenna_profile_{profiles::AntennaPatternProfile::kStandard};
  config::AzimuthElevationDeg antenna_boresight_offset_deg_{};
  profiles::RcsFusionProfile rcs_fusion_profile_{profiles::RcsFusionProfile::kDisabled};

  bool enable_tracking_filter_{false};
  profiles::TrackingPolicyProfile tracking_profile_{profiles::TrackingPolicyProfile::kBalanced};

  bool enable_imm_fusion_{false};
  profiles::LifecyclePolicyProfile lifecycle_profile_{profiles::LifecyclePolicyProfile::kBalanced};

  config::ArSessionConfig base_config_{};
  bool detection_dirty_{false};
  bool tracking_dirty_{false};
  bool lifecycle_dirty_{false};
};

/**
 * @brief 语义探测配置编辑器。
 */
class ONEQ_API ArSessionConfigBuilder::DetectionEditor {
 public:
  explicit DetectionEditor(ArSessionConfigBuilder* builder) : builder_(builder) {}

  /** @brief 开启或关闭物理探测链路。 */
  DetectionEditor& EnablePhysicsDetection(bool enable = true) {
    builder_->enable_physics_detection_ = enable;
    builder_->detection_dirty_ = true;
    return *this;
  }
  /** @brief 设置硬件语义档位。 */
  DetectionEditor& WithHardwareProfile(profiles::ArHardwareProfile profile) {
    builder_->hardware_profile_ = profile;
    builder_->detection_dirty_ = true;
    return *this;
  }
  /** @brief 设置探测意图语义档位。 */
  DetectionEditor& WithDetectionIntentProfile(profiles::DetectionIntentProfile profile) {
    builder_->intent_profile_ = profile;
    builder_->detection_dirty_ = true;
    return *this;
  }
  /** @brief 设置方向图语义档位。 */
  DetectionEditor& WithAntennaPatternProfile(profiles::AntennaPatternProfile profile) {
    builder_->antenna_profile_ = profile;
    builder_->detection_dirty_ = true;
    return *this;
  }
  /** @brief 设置 RCS 融合语义档位。 */
  DetectionEditor& WithRcsFusionProfile(profiles::RcsFusionProfile profile) {
    builder_->rcs_fusion_profile_ = profile;
    builder_->detection_dirty_ = true;
    return *this;
  }

  ArSessionConfigBuilder& End() { return *builder_; }

 private:
  ArSessionConfigBuilder* builder_;
};

/**
 * @brief 语义跟踪配置编辑器。
 */
class ONEQ_API ArSessionConfigBuilder::TrackingEditor {
 public:
  explicit TrackingEditor(ArSessionConfigBuilder* builder) : builder_(builder) {}

  /** @brief 开启或关闭公开跟踪滤波链路。 */
  TrackingEditor& EnableTrackingFilter(bool enable = true) {
    builder_->enable_tracking_filter_ = enable;
    builder_->tracking_dirty_ = true;
    return *this;
  }
  /** @brief 设置跟踪策略语义档位。 */
  TrackingEditor& WithTrackingPolicyProfile(profiles::TrackingPolicyProfile profile) {
    builder_->tracking_profile_ = profile;
    builder_->tracking_dirty_ = true;
    return *this;
  }

  ArSessionConfigBuilder& End() { return *builder_; }

 private:
  ArSessionConfigBuilder* builder_;
};

/**
 * @brief 语义生命周期配置编辑器。
 */
class ONEQ_API ArSessionConfigBuilder::LifecycleEditor {
 public:
  explicit LifecycleEditor(ArSessionConfigBuilder* builder) : builder_(builder) {}

  /** @brief 开启或关闭 IMM 融合。 */
  LifecycleEditor& EnableImmFusion(bool enable = true) {
    builder_->enable_imm_fusion_ = enable;
    builder_->lifecycle_dirty_ = true;
    return *this;
  }
  /** @brief 设置生命周期策略语义档位。 */
  LifecycleEditor& WithLifecyclePolicyProfile(profiles::LifecyclePolicyProfile profile) {
    builder_->lifecycle_profile_ = profile;
    builder_->lifecycle_dirty_ = true;
    return *this;
  }

  ArSessionConfigBuilder& End() { return *builder_; }

 private:
  ArSessionConfigBuilder* builder_;
};

/** @brief 环境语义配置编辑器。 */
class ONEQ_API ArSessionConfigBuilder::EnvironmentEditor {
 public:
  explicit EnvironmentEditor(ArSessionConfigBuilder* builder) : builder_(builder) {}

  /** @brief 设置干扰判定灵敏度语义档位。 */
  EnvironmentEditor& WithJammingSensitivityProfile(config::JammingSensitivityProfile profile) {
    builder_->base_config_.environment.jamming_sensitivity_profile = profile;
    return *this;
  }

  ArSessionConfigBuilder& End() { return *builder_; }

 private:
  ArSessionConfigBuilder* builder_;
};

inline ArSessionConfigBuilder::DetectionEditor ArSessionConfigBuilder::Detection() {
  return DetectionEditor(this);
}

inline ArSessionConfigBuilder::TrackingEditor ArSessionConfigBuilder::Tracking() {
  return TrackingEditor(this);
}

inline ArSessionConfigBuilder::LifecycleEditor ArSessionConfigBuilder::Lifecycle() {
  return LifecycleEditor(this);
}

inline ArSessionConfigBuilder::EnvironmentEditor ArSessionConfigBuilder::Environment() {
  return EnvironmentEditor(this);
}

// 兼容别名：旧名称在 wrapper 阶段保留。
using RadarSessionConfigBuilder = ArSessionConfigBuilder;
using RadarSessionConfigBuilder_DetectionEditor = ArSessionConfigBuilder::DetectionEditor;

}  // namespace config
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_CONFIG_AR_SESSION_CONFIG_BUILDER_H_
