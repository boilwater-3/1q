/**
 * @file EosSessionConfigBuilder.h
 * @brief EOS 分域语义式会话配置构造器。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_BUILDER_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_BUILDER_H_

#include <string>
#include <vector>

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
  class EnvironmentEditor;

  explicit EosSessionConfigBuilder(const config::EosSessionConfig& config = {}) noexcept
      : config_(config) {}

  EosSessionConfigBuilder& WithSessionConfig(const config::EosSessionConfig& config) noexcept {
    config_ = config;
    return *this;
  }

  MissionEditor Mission() noexcept;
  EnvironmentEditor Environment() noexcept;

  config::EosSessionConfig Build() const noexcept { return config_; }

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
  friend class EnvironmentEditor;

  config::EosSessionConfig config_{};
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
  EosSessionConfigBuilder& End() noexcept { return *builder_; }

 private:
  EosSessionConfigBuilder* builder_;
};

class ONEQ_API EosSessionConfigBuilder::EnvironmentEditor {
 public:
  explicit EnvironmentEditor(EosSessionConfigBuilder* builder) noexcept : builder_(builder) {}

  EnvironmentEditor& WithEnvironmentDefault(
      const environment::EosEnvironmentDefaultConfig& config) noexcept {
    builder_->config_.environment = config;
    return *this;
  }
  EnvironmentEditor& WithEnvironmentModelType(
      environment::EosEnvironmentModelType model_type) noexcept {
    builder_->config_.environment.scenario_config.model_type = model_type;
    return *this;
  }
  EnvironmentEditor& WithEnvironmentPreset(environment::EosEnvironmentPreset preset) noexcept {
    builder_->config_.environment.scenario_config.preset = preset;
    return *this;
  }
  EosSessionConfigBuilder& End() noexcept { return *builder_; }

 private:
  EosSessionConfigBuilder* builder_;
};

inline EosSessionConfigBuilder::MissionEditor EosSessionConfigBuilder::Mission() noexcept {
  return MissionEditor(this);
}

inline EosSessionConfigBuilder::EnvironmentEditor EosSessionConfigBuilder::Environment() noexcept {
  return EnvironmentEditor(this);
}

inline ValidationIssueList EosSessionConfigBuilder::Validate() const noexcept {
  ValidationIssueList issues;

  if (config_.mission.horizontal_fov_deg <= 0.0f) {
    ConfigValidationIssue issue;
    issue.code = ConfigValidationCode::kHorizontalFovNotPositive;
    issue.field = "mission.horizontal_fov_deg";
    issue.message = "Horizontal FOV must be positive.";
    issues.push_back(issue);
  }
  if (config_.mission.vertical_fov_deg <= 0.0f) {
    ConfigValidationIssue issue;
    issue.code = ConfigValidationCode::kVerticalFovNotPositive;
    issue.field = "mission.vertical_fov_deg";
    issue.message = "Vertical FOV must be positive.";
    issues.push_back(issue);
  }
  if (config_.mission.scan_rate_deg_per_sec <= 0.0f) {
    ConfigValidationIssue issue;
    issue.code = ConfigValidationCode::kScanRateNotPositive;
    issue.field = "mission.scan_rate_deg_per_sec";
    issue.message = "Scan rate must be positive.";
    issues.push_back(issue);
  }
  if (config_.mission.frame_rate_hz <= 0.0f) {
    ConfigValidationIssue issue;
    issue.code = ConfigValidationCode::kFrameRateNotPositive;
    issue.field = "mission.frame_rate_hz";
    issue.message = "Frame rate must be positive.";
    issues.push_back(issue);
  }
  if (config_.mission.scan_start_az_deg >= config_.mission.scan_end_az_deg) {
    ConfigValidationIssue issue;
    issue.code = ConfigValidationCode::kScanRangeAzSwapped;
    issue.field = "mission.scan_start_az_deg / scan_end_az_deg";
    issue.message = "Scan start azimuth must be less than end azimuth.";
    issues.push_back(issue);
  }

  return issues;
}

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_BUILDER_H_
