/**
 * @file EsrSessionConfigBuilder.h
 * @brief ESR 分域语义式会话配置构造器。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_BUILDER_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_BUILDER_H_

#include "1q/electronic_surveillance_radar/config/EsrSessionConfig.h"

namespace electronic_surveillance_radar {
namespace config {

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
  EsrSessionConfigBuilder& End() { return *builder_; }

 private:
  EsrSessionConfigBuilder* builder_;
};

class ONEQ_API EsrSessionConfigBuilder::DetectionEditor {
 public:
  explicit DetectionEditor(EsrSessionConfigBuilder* builder) : builder_(builder) {}

  DetectionEditor& WithDetectionProfile(config::EsrDetectionProfile profile) {
    builder_->config_.policy.detection.profile = profile;
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
      const environment::EsrEnvironmentDefaultConfig& config) {
    builder_->config_.environment = config;
    return *this;
  }
  EnvironmentEditor& WithEnvironmentPreset(config::EsrEnvironmentPreset preset) {
    builder_->config_.environment.scenario_config.preset = preset;
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

}  // namespace config
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_BUILDER_H_
