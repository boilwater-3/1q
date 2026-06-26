/**
 * @file EsrSessionConfigBuilder.h
 * @brief ESR 分域语义式会话配置构造器。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_BUILDER_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_BUILDER_H_

#include <string>
#include <vector>

#include "1q/electronic_surveillance_radar/config/EsrSessionConfig.h"

namespace electronic_surveillance_radar {
namespace config {

/**
 * @brief ConfigValidationCode 表示构造器校验问题编码。
 */
enum class ConfigValidationCode {
  kNone = 0,
  kScanRateNotPositive,            /**< 扫描数据率 <= 0 */
  kReceiverBandLowerAboveUpper,    /**< 接收频段下限 >= 上限 */
  kBeamAzWidthNotPositive,         /**< 方位波束宽度 <= 0 */
  kBeamElWidthNotPositive,         /**< 俯仰波束宽度 <= 0 */
  kExplicitScanBoundsAzSwapped,    /**< 显式扫描方位起止颠倒 */
  kExplicitScanBoundsElSwapped     /**< 显式扫描俯仰起止颠倒 */
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
 * @brief EsrSessionConfigBuilder 提供初始化配置语义积木。
 * @note 推荐路径：
 *       会话初始化优先使用本构造器表达 mission/detection/environment 语义；
 *       运行期热更新统一使用 EsrRuntimeConfigBuilder；
 *       需要直接编辑四域细项时使用直接字段赋值。
 */
class ONEQ_API EsrSessionConfigBuilder {
 public:
  class MissionEditor;
  class DetectionEditor;
  class EnvironmentEditor;

  explicit EsrSessionConfigBuilder(const config::EsrSessionConfig& config = {})
      : config_(config) {}

  EsrSessionConfigBuilder& WithSessionConfig(const config::EsrSessionConfig& config) {
    config_ = config;
    return *this;
  }
  MissionEditor Mission();
  DetectionEditor Detection();
  EnvironmentEditor Environment();

  config::EsrSessionConfig Build() const { return config_; }

  /**
   * @brief 校验当前 Builder 状态的合法性，用于构造完成前的早期反馈。
   *
   * 检查项包括：
   * - 扫描数据率为正；
   * - 接收频段上下限一致；
   * - 波束宽度为正；
   * - 显式扫描边界起止一致。
   *
   * @note 本校验仅为构造期早期反馈，完整的运行期校验仍由
   *       RuntimeConfigResolver 执行。`Build()` 的行为不受校验结果影响。
   * @return 按发现顺序返回的校验问题列表。
   */
  ValidationIssueList Validate() const;

 private:
  friend class MissionEditor;
  friend class DetectionEditor;
  friend class EnvironmentEditor;

  config::EsrSessionConfig config_{};
};

class ONEQ_API EsrSessionConfigBuilder::MissionEditor {
 public:
  explicit MissionEditor(EsrSessionConfigBuilder* builder) : builder_(builder) {}

  MissionEditor& WithWorkMode(config::EsrWorkMode mode) {
    builder_->config_.mission.work_mode = mode;
    return *this;
  }
  MissionEditor& WithPowerOn(bool power_on) {
    builder_->config_.mission.power_on = power_on;
    return *this;
  }
  MissionEditor& WithScanRateHz(float value) {
    builder_->config_.mission.scan.scan_rate_hz = value;
    return *this;
  }
  MissionEditor& WithScanCenterAzDeg(float value) {
    builder_->config_.mission.scan.scan_center_az_deg = value;
    return *this;
  }
  MissionEditor& WithScanCenterElDeg(float value) {
    builder_->config_.mission.scan.scan_center_el_deg = value;
    return *this;
  }
  MissionEditor& WithScanStartPosition(config::EsrScanStartPosition position) {
    builder_->config_.mission.scan.scan_start_position = position;
    return *this;
  }
  MissionEditor& WithScanSequence(config::EsrScanSequence sequence) {
    builder_->config_.mission.scan.scan_sequence = sequence;
    return *this;
  }
  MissionEditor& WithUseExplicitScanBounds(bool value) {
    builder_->config_.mission.scan.use_explicit_scan_bounds = value;
    return *this;
  }
  MissionEditor& WithScanStartAzDeg(float value) {
    builder_->config_.mission.scan.scan_start_az_deg = value;
    return *this;
  }
  MissionEditor& WithScanEndAzDeg(float value) {
    builder_->config_.mission.scan.scan_end_az_deg = value;
    return *this;
  }
  MissionEditor& WithScanStartElDeg(float value) {
    builder_->config_.mission.scan.scan_start_el_deg = value;
    return *this;
  }
  MissionEditor& WithScanEndElDeg(float value) {
    builder_->config_.mission.scan.scan_end_el_deg = value;
    return *this;
  }
  EsrSessionConfigBuilder& End() { return *builder_; }

 private:
  EsrSessionConfigBuilder* builder_;
};

class ONEQ_API EsrSessionConfigBuilder::DetectionEditor {
 public:
  explicit DetectionEditor(EsrSessionConfigBuilder* builder) : builder_(builder) {}

  DetectionEditor& WithMinDetectSnrDb(float value) {
    builder_->config_.policy.detection.minimum_snr_db = value;
    return *this;
  }
  DetectionEditor& WithPfa(float value) {
    builder_->config_.policy.detection.pfa = value;
    return *this;
  }
  DetectionEditor& WithPulseCount(std::uint32_t value) {
    builder_->config_.policy.detection.pulse_count = value;
    return *this;
  }
  DetectionEditor& WithThresholdScale(float value) {
    builder_->config_.policy.detection.threshold_scale = value;
    return *this;
  }
  DetectionEditor& EnableStatisticalDetection(bool enable) {
    builder_->config_.policy.detection.enable_statistical_detection = enable;
    return *this;
  }
  EsrSessionConfigBuilder& End() { return *builder_; }

 private:
  EsrSessionConfigBuilder* builder_;
};

class ONEQ_API EsrSessionConfigBuilder::EnvironmentEditor {
 public:
  explicit EnvironmentEditor(EsrSessionConfigBuilder* builder) : builder_(builder) {}

  EnvironmentEditor& WithEnvironmentDefault(
      const config::EsrEnvironmentConfig& config) {
    builder_->config_.environment = config;
    return *this;
  }
  EnvironmentEditor& WithEnvironmentPreset(config::EsrEnvironmentPreset preset) {
    builder_->config_.environment.scenario_config.preset = preset;
    return *this;
  }
  EnvironmentEditor& WithAtmosphericPhysics(
      const config::EsrAtmosphericPhysicsConfig& config) {
    builder_->config_.environment.scenario_config.atmospheric_physics = config;
    return *this;
  }
  EnvironmentEditor& WithAtmosphericContext(
      const config::EsrAtmosphericDerivedContext& context) {
    builder_->config_.environment.scenario_config.atmospheric_context = context;
    return *this;
  }
  EsrSessionConfigBuilder& End() { return *builder_; }

 private:
  EsrSessionConfigBuilder* builder_;
};

inline EsrSessionConfigBuilder::MissionEditor EsrSessionConfigBuilder::Mission() {
  return MissionEditor(this);
}

inline EsrSessionConfigBuilder::DetectionEditor EsrSessionConfigBuilder::Detection() {
  return DetectionEditor(this);
}

inline EsrSessionConfigBuilder::EnvironmentEditor EsrSessionConfigBuilder::Environment() {
  return EnvironmentEditor(this);
}

inline ValidationIssueList EsrSessionConfigBuilder::Validate() const {
  ValidationIssueList issues;

  if (config_.mission.scan.scan_rate_hz <= 0.0f) {
    ConfigValidationIssue issue;
    issue.code = ConfigValidationCode::kScanRateNotPositive;
    issue.field = "mission.scan.scan_rate_hz";
    issue.message = "Scan rate must be positive.";
    issues.push_back(issue);
  }

  if (config_.hardware.receiver_band_lower_hz >= config_.hardware.receiver_band_upper_hz) {
    ConfigValidationIssue issue;
    issue.code = ConfigValidationCode::kReceiverBandLowerAboveUpper;
    issue.field = "hardware.receiver_band_lower_hz / receiver_band_upper_hz";
    issue.message = "Receiver band lower bound must be below upper bound.";
    issues.push_back(issue);
  }

  if (config_.hardware.beam_az_width_deg <= 0.0f) {
    ConfigValidationIssue issue;
    issue.code = ConfigValidationCode::kBeamAzWidthNotPositive;
    issue.field = "hardware.beam_az_width_deg";
    issue.message = "Azimuth beamwidth must be positive.";
    issues.push_back(issue);
  }
  if (config_.hardware.beam_el_width_deg <= 0.0f) {
    ConfigValidationIssue issue;
    issue.code = ConfigValidationCode::kBeamElWidthNotPositive;
    issue.field = "hardware.beam_el_width_deg";
    issue.message = "Elevation beamwidth must be positive.";
    issues.push_back(issue);
  }

  if (config_.mission.scan.use_explicit_scan_bounds) {
    if (config_.mission.scan.scan_start_az_deg >= config_.mission.scan.scan_end_az_deg) {
      ConfigValidationIssue issue;
      issue.code = ConfigValidationCode::kExplicitScanBoundsAzSwapped;
      issue.field = "mission.scan.scan_start_az_deg / scan_end_az_deg";
      issue.message = "Scan start azimuth must be less than end azimuth.";
      issues.push_back(issue);
    }
    if (config_.mission.scan.scan_start_el_deg >= config_.mission.scan.scan_end_el_deg) {
      ConfigValidationIssue issue;
      issue.code = ConfigValidationCode::kExplicitScanBoundsElSwapped;
      issue.field = "mission.scan.scan_start_el_deg / scan_end_el_deg";
      issue.message = "Scan start elevation must be less than end elevation.";
      issues.push_back(issue);
    }
  }

  return issues;
}

}  // namespace config
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_BUILDER_H_
