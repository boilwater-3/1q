/**
 * @file RadarSessionConfigBuilder.h
 * @brief 提供链式构造 config::RadarSessionConfig 的 Builder（用于会话初始化基线配置）。
 */

#ifndef AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_BUILDER_H_
#define AIRBORNE_RADAR_CONFIG_RADAR_SESSION_CONFIG_BUILDER_H_

#include <string>
#include <vector>

#include "1q/airborne_radar/config/RadarSessionConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {

using model::AzimuthElevationDeg;
using model::CommandedBeamwidthDeg;
using model::RadarWorkSubMode;

/**
 * @brief ConfigValidationCode 表示构造器校验问题编码。
 */
enum class ConfigValidationCode {
  kNone = 0,
  kCommandedBeamwidthAzNotPositive,    /**< 指令态方位波束宽度 <= 0 */
  kCommandedBeamwidthElNotPositive,    /**< 指令态俯仰波束宽度 <= 0 */
  kMechanicalScanLimitsSwappedAz,      /**< 机械方位限位 min > max */
  kMechanicalScanLimitsSwappedEl,      /**< 机械俯仰限位 min > max */
  kElectronicScanLimitsSwappedAz,      /**< 电子方位限位 min > max */
  kElectronicScanLimitsSwappedEl,      /**< 电子俯仰限位 min > max */
  kRobustTrackingWithoutImm            /**< 抗干扰跟踪开启但 IMM 未启用 */
};

/**
 * @brief ConfigValidationIssue 描述一条构造器校验结果。
 */
struct ConfigValidationIssue {
  ConfigValidationCode code{ConfigValidationCode::kNone}; /**< 问题编码 */
  std::string field{};   /**< 关联字段名 */
  std::string message{}; /**< 简短说明 */
};

/** @brief ValidationIssueList 表示构造器校验问题列表。 */
using ValidationIssueList = std::vector<ConfigValidationIssue>;

/**
 * @brief RadarSession 配置链式构造器。
 *
 * Builder 通过语义档位（Profile enum）控制探测 / 跟踪 / 生命周期行为，
 * `Build()` 时将语义设定翻译为四域配置中的 hardware/policy 子域。
 * 波束方向与扫描状态（mission.orientation）及环境默认配置直接透传至输出。
 *
 * @note 推荐路径：
 * - 会话初始化优先使用本构造器表达高层语义输入；
 * - 运行期热更新统一使用 `RadarRuntimeConfigBuilder`；
 * - 仅在需要直接编辑四域细项时使用直接字段赋值。
 *
 * @code
 * auto config = RadarSessionConfigBuilder()
 *                   .Detection()
 *                   .EnablePhysicsDetection(true)
 *                   .WithHardwareProfile(
 *                       profiles::RadarHardwareProfile::kLongRangeHighPower)
 *                   .WithDetectionIntentProfile(
 *                       profiles::DetectionIntentProfile::kDetectionPriority)
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
  /** @brief 任务态与波束控制配置编辑器。 */
  class MissionEditor;
  /** @brief 语义跟踪配置编辑器。 */
  class TrackingEditor;
  /** @brief 语义生命周期配置编辑器。 */
  class LifecycleEditor;
  /** @brief 环境默认配置编辑器。 */
  class EnvironmentEditor;

  /** @brief 使用语义默认值初始化 Builder。 */
  RadarSessionConfigBuilder();

  /**
   * @brief 使用现有会话配置初始化 Builder。
   *
   * 输入配置中的 `hardware/mission/policy` 会作为工程参数基线保留；
   * 仅当调用相应语义编辑器时，对应域才会被语义档位重新覆盖。
   *
   * @param config 作为编辑基线的会话配置。
   */
  explicit RadarSessionConfigBuilder(const config::RadarSessionConfig& config);

  /** @brief 进入探测配置编辑域。 */
  DetectionEditor Detection();
  /** @brief 进入任务态与波束控制配置编辑域。 */
  MissionEditor Mission();
  /** @brief 进入跟踪配置编辑域。 */
  TrackingEditor Tracking();
  /** @brief 进入生命周期配置编辑域。 */
  LifecycleEditor Lifecycle();
  /** @brief 进入环境默认配置编辑域。 */
  EnvironmentEditor Environment();

  /**
   * @brief 将语义档位翻译为四域配置，生成最终会话配置。
   * @return 构建完成的 `config::RadarSessionConfig`（hardware/mission/policy/environment）。
   */
  config::RadarSessionConfig Build() const;

  /**
   * @brief 校验当前 Builder 状态的合法性，用于构造完成前的早期反馈。
   *
   * 检查项包括：
   * - 指令态波束宽度在启用时为正；
   * - 机械 / 电子扫描限位一致（min <= max）；
   * - 策略组合合理性（如抗干扰跟踪但未启用 IMM）。
   *
   * @note 本校验仅为构造期早期反馈，完整的运行期校验仍由
   *       `RuntimeConfigResolver` 执行。`Build()` 的行为不受校验结果影响。
   * @return 按发现顺序返回的校验问题列表。
   */
  ValidationIssueList Validate() const;

 private:
  friend class DetectionEditor;
  friend class MissionEditor;
  friend class TrackingEditor;
  friend class LifecycleEditor;
  friend class EnvironmentEditor;

  bool enable_physics_detection_{false};
  profiles::RadarHardwareProfile hardware_profile_{
      profiles::RadarHardwareProfile::kGenericAirborneXBand};
  profiles::DetectionIntentProfile intent_profile_{profiles::DetectionIntentProfile::kBalanced};
  profiles::AntennaPatternProfile antenna_profile_{profiles::AntennaPatternProfile::kStandard};
  model::AzimuthElevationDeg antenna_boresight_offset_deg_{};
  profiles::RcsFusionProfile rcs_fusion_profile_{profiles::RcsFusionProfile::kDisabled};

  bool enable_tracking_filter_{false};
  profiles::TrackingPolicyProfile tracking_profile_{profiles::TrackingPolicyProfile::kBalanced};

  bool enable_imm_fusion_{false};
  profiles::LifecyclePolicyProfile lifecycle_profile_{profiles::LifecyclePolicyProfile::kBalanced};

  config::RadarSessionConfig base_config_{};
  model::RadarOrientationConfig orientation_{};
  environment::EnvironmentDefaultConfig env_{};
  environment::JammingSensitivityProfile jamming_sensitivity_profile_{
      environment::JammingSensitivityProfile::kBalanced};
  bool detection_dirty_{false};
  bool tracking_dirty_{false};
  bool lifecycle_dirty_{false};
};

/**
 * @brief 语义探测配置编辑器。
 */
class ONEQ_API RadarSessionConfigBuilder::DetectionEditor {
 public:
  explicit DetectionEditor(RadarSessionConfigBuilder* builder) : builder_(builder) {}

  /** @brief 开启或关闭物理探测链路。 */
  DetectionEditor& EnablePhysicsDetection(bool enable = true) {
    builder_->enable_physics_detection_ = enable;
    builder_->detection_dirty_ = true;
    return *this;
  }
  /** @brief 设置硬件语义档位。 */
  DetectionEditor& WithHardwareProfile(profiles::RadarHardwareProfile profile) {
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

  RadarSessionConfigBuilder& End() { return *builder_; }

 private:
  RadarSessionConfigBuilder* builder_;
};

/**
 * @brief 任务态与波束控制配置编辑器。
 */
class ONEQ_API RadarSessionConfigBuilder::MissionEditor {
 public:
  explicit MissionEditor(RadarSessionConfigBuilder* builder) : builder_(builder) {}

  /** @brief 设置雷达工作子模式。 */
  MissionEditor& WithRadarWorkSubMode(RadarWorkSubMode work_sub_mode) {
    builder_->orientation_.work_sub_mode = work_sub_mode;
    return *this;
  }
  /** @brief 设置扫描中心。 */
  MissionEditor& WithScanCenterDeg(const AzimuthElevationDeg& scan_center_deg) {
    builder_->orientation_.scan_center_deg = scan_center_deg;
    return *this;
  }
  /** @brief 更新指令态波束使能。 */
  MissionEditor& EnableCommandedBeamwidth(bool enable = true) {
    builder_->orientation_.commanded_beamwidth_enabled = enable;
    return *this;
  }
  /** @brief 设置指令态波束宽度。 */
  MissionEditor& WithCommandedBeamwidthDeg(const CommandedBeamwidthDeg& beamwidth_deg) {
    builder_->orientation_.commanded_beamwidth_deg = beamwidth_deg;
    return *this;
  }

  RadarSessionConfigBuilder& End() { return *builder_; }

 private:
  RadarSessionConfigBuilder* builder_;
};

/**
 * @brief 语义跟踪配置编辑器。
 */
class ONEQ_API RadarSessionConfigBuilder::TrackingEditor {
 public:
  explicit TrackingEditor(RadarSessionConfigBuilder* builder) : builder_(builder) {}

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

  RadarSessionConfigBuilder& End() { return *builder_; }

 private:
  RadarSessionConfigBuilder* builder_;
};

/**
 * @brief 语义生命周期配置编辑器。
 */
class ONEQ_API RadarSessionConfigBuilder::LifecycleEditor {
 public:
  explicit LifecycleEditor(RadarSessionConfigBuilder* builder) : builder_(builder) {}

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

  RadarSessionConfigBuilder& End() { return *builder_; }

 private:
  RadarSessionConfigBuilder* builder_;
};

/**
 * @brief 环境默认配置编辑器。
 */
class ONEQ_API RadarSessionConfigBuilder::EnvironmentEditor {
 public:
  explicit EnvironmentEditor(RadarSessionConfigBuilder* builder) : builder_(builder) {}

  /** @brief 整块替换环境默认配置。 */
  EnvironmentEditor& WithEnvironmentDefault(const environment::EnvironmentDefaultConfig& env) {
    builder_->env_ = env;
    return *this;
  }
  /** @brief 设置干扰判定灵敏度语义档位。 */
  EnvironmentEditor& WithJammingSensitivityProfile(environment::JammingSensitivityProfile profile) {
    builder_->jamming_sensitivity_profile_ = profile;
    return *this;
  }

  RadarSessionConfigBuilder& End() { return *builder_; }

 private:
  RadarSessionConfigBuilder* builder_;
};

inline RadarSessionConfigBuilder::DetectionEditor RadarSessionConfigBuilder::Detection() {
  return DetectionEditor(this);
}

inline RadarSessionConfigBuilder::MissionEditor RadarSessionConfigBuilder::Mission() {
  return MissionEditor(this);
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
