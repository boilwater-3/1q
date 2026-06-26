/**
 * @file EosSessionConfigBuilder.h
 * @brief EOS 分域语义式会话配置构造器。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_BUILDER_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_BUILDER_H_

#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/config/EosSessionConfig.h"

namespace electro_optical_sensor {
namespace config {

/**
 * @brief ConfigValidationCode 表示构造器校验问题编码。
 */
enum class ConfigValidationCode {
  kNone = 0,
  kHorizontalFovNotPositive,  /**< 水平视场角 <= 0 */
  kVerticalFovNotPositive,    /**< 垂直视场角 <= 0 */
  kScanRateNotPositive,       /**< 扫描角速度 <= 0 */
  kFrameRateNotPositive,      /**< 帧率 <= 0 */
  kScanRangeAzSwapped         /**< 扫描方位起止颠倒（起始 >= 结束） */
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
 * @brief EosMissionProfile 表示 EOS 任务剖面语义档位。
 *
 * 选择剖面后 Builder 在 Build() 时自动填写 FOV、扫描率、帧率、
 * 工作模式及探测信噪比门限，免去逐项手工配置。
 */
enum class ONEQ_API EosMissionProfile {
  kWideAreaSearch = 0,          /**< 大范围搜索：Fused，12°×8°，30°/s，15Hz，snr=6dB */
  kLongRangeSurveillance,       /**< 远程监视：InfraredOnly，3°×2°，10°/s，10Hz，snr=3dB */
  kHighResolutionTrack          /**< 高精度跟踪：Fused，1.5°×1°，5°/s，60Hz，snr=2dB */
};

/**
 * @brief EosHardwareProfile 表示 EOS 硬件规格语义档位。
 *
 * 选择档位后 Builder 在 Build() 时自动填写波长范围、光学口径、
 * 探测器比探测率等硬件参数。
 */
enum class ONEQ_API EosHardwareProfile {
  kStandardMidWaveIR = 0,       /**< 标准中波红外：3-5μm，0.2m 口径，D*=1e10 */
  kLongRangeLargeAperture,      /**< 远程大口径：3-5μm，0.4m 口径，D*=2e10 */
  kWideAreaCompact              /**< 广域紧凑：8-12μm，0.1m 口径，D*=5e9 */
};

/**
 * @brief EosSessionConfigBuilder 提供初始化配置语义积木。
 * @note 该构造器只表达 mission/detection/stray-light/environment 的初始化语义。
 *       常见场景推荐配置应在 example 或业务层以具名函数封装，并返回
 *       config::EosSessionConfig 传入 EosSessionFactory。
 * @note 推荐路径：
 *       会话初始化优先使用本构造器；
 *       运行期热更新统一使用 EosRuntimeConfigBuilder。
 */
class ONEQ_API EosSessionConfigBuilder {
 public:
  class MissionEditor;
  class PolicyEditor;
  class HardwareEditor;
  class EnvironmentEditor;

  explicit EosSessionConfigBuilder(const config::EosSessionConfig& config = {}) noexcept
      : config_(config) {}

  EosSessionConfigBuilder& WithSessionConfig(const config::EosSessionConfig& config) noexcept {
    config_ = config;
    return *this;
  }

  MissionEditor Mission() noexcept;
  PolicyEditor Policy() noexcept;
  HardwareEditor Hardware() noexcept;
  EnvironmentEditor Environment() noexcept;

  /**
   * @brief 将语义档位翻译为配置字段，生成最终会话配置。
   *
   * 如果通过 Editor 设置了 Profile 枚举，Build() 会将语义设定翻译为
   * mission / hardware / policy.detection 中的对应字段。直接字段 setter
   * 的赋值在 Profile 应用之后被覆盖——Profile 是高层语义入口，直接 setter
   * 适合在 Profile 未设置的场景下做精细调整。
   *
   * @return 构建完成的 `config::EosSessionConfig`。
   */
  config::EosSessionConfig Build() const noexcept;

  /**
   * @brief 校验当前 Builder 状态的合法性，用于构造完成前的早期反馈。
   *
   * 检查项包括：
   * - 视场角为正；
   * - 扫描角速度与帧率为正；
   * - 扫描方位起止角一致。
   *
   * @note 本校验仅为构造期早期反馈，完整的运行期校验仍由
   *       RuntimeConfigResolver 执行。`Build()` 的行为不受校验结果影响。
   * @return 按发现顺序返回的校验问题列表。
   */
  ValidationIssueList Validate() const noexcept;

 private:
  friend class MissionEditor;
  friend class PolicyEditor;
  friend class HardwareEditor;
  friend class EnvironmentEditor;

  config::EosSessionConfig config_{};
  EosMissionProfile mission_profile_{EosMissionProfile::kWideAreaSearch};
  EosHardwareProfile hardware_profile_{EosHardwareProfile::kStandardMidWaveIR};
  bool mission_profile_dirty_{false};
  bool hardware_profile_dirty_{false};
};

class ONEQ_API EosSessionConfigBuilder::MissionEditor {
 public:
  explicit MissionEditor(EosSessionConfigBuilder* builder) noexcept : builder_(builder) {}

  MissionEditor& WithWorkMode(config::EosWorkMode mode) noexcept {
    builder_->config_.mission.work_mode = mode;
    return *this;
  }
  MissionEditor& WithScanRateDegPerSec(float value) noexcept {
    builder_->config_.mission.scan_rate_deg_per_sec = value;
    return *this;
  }
  MissionEditor& WithFrameRateHz(float value) noexcept {
    builder_->config_.mission.frame_rate_hz = value;
    return *this;
  }
  MissionEditor& WithPowerOn(bool power_on) noexcept {
    builder_->config_.mission.power_on = power_on;
    return *this;
  }
  MissionEditor& WithHorizontalFovDeg(float value) noexcept {
    builder_->config_.mission.horizontal_fov_deg = value;
    return *this;
  }
  MissionEditor& WithVerticalFovDeg(float value) noexcept {
    builder_->config_.mission.vertical_fov_deg = value;
    return *this;
  }
  MissionEditor& WithScanStartAzDeg(float value) noexcept {
    builder_->config_.mission.scan_start_az_deg = value;
    return *this;
  }
  MissionEditor& WithScanEndAzDeg(float value) noexcept {
    builder_->config_.mission.scan_end_az_deg = value;
    return *this;
  }
  MissionEditor& WithScanCenterElDeg(float value) noexcept {
    builder_->config_.mission.scan_center_el_deg = value;
    return *this;
  }
  MissionEditor& WithBoresightDepressionDeg(float value) noexcept {
    builder_->config_.mission.boresight_depression_deg = value;
    return *this;
  }
  /** @brief 设置任务剖面语义档位。Build() 时自动翻译为 mission 及 policy.detection 字段。 */
  MissionEditor& WithMissionProfile(EosMissionProfile profile) noexcept {
    builder_->mission_profile_ = profile;
    builder_->mission_profile_dirty_ = true;
    return *this;
  }
  EosSessionConfigBuilder& End() noexcept { return *builder_; }

 private:
  EosSessionConfigBuilder* builder_;
};

class ONEQ_API EosSessionConfigBuilder::EnvironmentEditor {
 public:
  explicit EnvironmentEditor(EosSessionConfigBuilder* builder) noexcept : builder_(builder) {}

  EnvironmentEditor& WithEnvironmentDefault(
      const config::EosEnvironmentConfig& config) noexcept {
    builder_->config_.environment = config;
    return *this;
  }
  EnvironmentEditor& WithEnvironmentModelType(
      config::EosEnvironmentModelType model_type) noexcept {
    builder_->config_.environment.scenario_config.model_type = model_type;
    return *this;
  }
  EnvironmentEditor& WithEnvironmentPreset(config::EosEnvironmentPreset preset) noexcept {
    builder_->config_.environment.scenario_config.preset = preset;
    return *this;
  }
  EosSessionConfigBuilder& End() noexcept { return *builder_; }

 private:
  EosSessionConfigBuilder* builder_;
};

class ONEQ_API EosSessionConfigBuilder::PolicyEditor {
 public:
  explicit PolicyEditor(EosSessionConfigBuilder* builder) noexcept : builder_(builder) {}

  PolicyEditor& WithMinSnrDb(float value) noexcept {
    builder_->config_.policy.detection.minimum_snr_db = value;
    return *this;
  }
  PolicyEditor& WithDetectionSensitivityW(float value) noexcept {
    builder_->config_.policy.detection.detection_sensitivity_w = value;
    return *this;
  }
  PolicyEditor& WithVisibleReferenceIrradianceWM2(float value) noexcept {
    builder_->config_.policy.detection.visible_reference_irradiance_w_m2 = value;
    return *this;
  }
  PolicyEditor& WithEnableStraylightFilter(bool value) noexcept {
    builder_->config_.policy.stray_light.enable_straylight_filter = value;
    return *this;
  }
  PolicyEditor& WithHoodInnerHalfAngleDeg(float value) noexcept {
    builder_->config_.policy.stray_light.hood_inner_half_angle_deg = value;
    return *this;
  }
  PolicyEditor& WithHoodOuterHalfAngleDeg(float value) noexcept {
    builder_->config_.policy.stray_light.hood_outer_half_angle_deg = value;
    return *this;
  }
  PolicyEditor& WithHoodMinSuppressionRatio(float value) noexcept {
    builder_->config_.policy.stray_light.hood_min_suppression_ratio = value;
    return *this;
  }
  PolicyEditor& WithHoodMaxSuppressionRatio(float value) noexcept {
    builder_->config_.policy.stray_light.hood_max_suppression_ratio = value;
    return *this;
  }
  EosSessionConfigBuilder& End() noexcept { return *builder_; }

 private:
  EosSessionConfigBuilder* builder_;
};

class ONEQ_API EosSessionConfigBuilder::HardwareEditor {
 public:
  explicit HardwareEditor(EosSessionConfigBuilder* builder) noexcept : builder_(builder) {}

  HardwareEditor& WithWavelengthLowerUm(float value) noexcept {
    builder_->config_.hardware.wavelength_lower_um = value;
    return *this;
  }
  HardwareEditor& WithWavelengthUpperUm(float value) noexcept {
    builder_->config_.hardware.wavelength_upper_um = value;
    return *this;
  }
  HardwareEditor& WithOpticalApertureM(float value) noexcept {
    builder_->config_.hardware.optical_aperture_m = value;
    return *this;
  }
  HardwareEditor& WithFocalLengthM(float value) noexcept {
    builder_->config_.hardware.focal_length_m = value;
    return *this;
  }
  HardwareEditor& WithDetectorDetectivity(float value) noexcept {
    builder_->config_.hardware.detector_detectivity_cm_sqrt_hz_per_w = value;
    return *this;
  }
  HardwareEditor& WithDetectorAreaCm2(float value) noexcept {
    builder_->config_.hardware.detector_area_cm2 = value;
    return *this;
  }
  HardwareEditor& WithMinDetectionDepressionDeg(float value) noexcept {
    builder_->config_.hardware.min_detection_depression_deg = value;
    return *this;
  }
  HardwareEditor& WithMaxDetectionDepressionDeg(float value) noexcept {
    builder_->config_.hardware.max_detection_depression_deg = value;
    return *this;
  }
  /** @brief 设置硬件规格语义档位。Build() 时自动翻译为 hardware 字段。 */
  HardwareEditor& WithHardwareProfile(EosHardwareProfile profile) noexcept {
    builder_->hardware_profile_ = profile;
    builder_->hardware_profile_dirty_ = true;
    return *this;
  }
  EosSessionConfigBuilder& End() noexcept { return *builder_; }

 private:
  EosSessionConfigBuilder* builder_;
};

inline EosSessionConfigBuilder::MissionEditor EosSessionConfigBuilder::Mission() noexcept {
  return MissionEditor(this);
}

inline EosSessionConfigBuilder::PolicyEditor EosSessionConfigBuilder::Policy() noexcept {
  return PolicyEditor(this);
}

inline EosSessionConfigBuilder::HardwareEditor EosSessionConfigBuilder::Hardware() noexcept {
  return HardwareEditor(this);
}

inline EosSessionConfigBuilder::EnvironmentEditor EosSessionConfigBuilder::Environment() noexcept {
  return EnvironmentEditor(this);
}

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_BUILDER_H_
