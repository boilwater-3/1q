/**
 * @file EosSessionConfigBuilder.h
 * @brief EOS 分域语义式会话配置构造器。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_BUILDER_H_
#define ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_BUILDER_H_

#include "1q/electro_optical_sensor/config/EosSessionConfig.h"

namespace electro_optical_sensor {
namespace config {

/**
 * @brief EosSessionConfigBuilder 提供初始化配置语义积木。
 * @note 该构造器只表达 mission/detection/stray-light/environment 的初始化语义。
 *       常见场景推荐配置应在 example 或业务层以具名函数封装，并返回
 *       EosSessionConfig 传入 EosSessionFactory。
 * @note 推荐路径：
 *       会话初始化优先使用本构造器；
 *       运行期热更新统一使用 EosRuntimeConfigBuilder。
 */
class ONEQ_API EosSessionConfigBuilder {
 public:
  class MissionEditor;
  class DetectionEditor;
  class StrayLightEditor;
  class EnvironmentEditor;

  explicit EosSessionConfigBuilder(const session::EosSessionConfig& config = {}) noexcept
      : config_(config) {}

  EosSessionConfigBuilder& WithSessionConfig(const session::EosSessionConfig& config) noexcept {
    config_ = config;
    return *this;
  }

  MissionEditor Mission() noexcept;
  DetectionEditor Detection() noexcept;
  StrayLightEditor StrayLight() noexcept;
  EnvironmentEditor Environment() noexcept;

  session::EosSessionConfig Build() const noexcept { return config_; }

 private:
  friend class MissionEditor;
  friend class DetectionEditor;
  friend class StrayLightEditor;
  friend class EnvironmentEditor;

  session::EosSessionConfig config_{};
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
  EosSessionConfigBuilder& End() noexcept { return *builder_; }

 private:
  EosSessionConfigBuilder* builder_;
};

class ONEQ_API EosSessionConfigBuilder::DetectionEditor {
 public:
  explicit DetectionEditor(EosSessionConfigBuilder* builder) noexcept : builder_(builder) {}

  DetectionEditor& WithDetectionProfile(EosDetectionProfile profile) noexcept {
    builder_->config_.policy.detection.profile = profile;
    builder_->config_.policy.detection.use_profile_defaults = true;
    return *this;
  }
  EosSessionConfigBuilder& End() noexcept { return *builder_; }

 private:
  EosSessionConfigBuilder* builder_;
};

class ONEQ_API EosSessionConfigBuilder::StrayLightEditor {
 public:
  explicit StrayLightEditor(EosSessionConfigBuilder* builder) noexcept : builder_(builder) {}

  StrayLightEditor& WithStrayLightProfile(EosStrayLightProfile profile) noexcept {
    builder_->config_.policy.stray_light.profile = profile;
    builder_->config_.policy.stray_light.use_profile_defaults = true;
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
  EnvironmentEditor& WithEnvironmentPreset(EosEnvironmentPreset preset) noexcept {
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

inline EosSessionConfigBuilder::DetectionEditor EosSessionConfigBuilder::Detection() noexcept {
  return DetectionEditor(this);
}

inline EosSessionConfigBuilder::StrayLightEditor EosSessionConfigBuilder::StrayLight() noexcept {
  return StrayLightEditor(this);
}

inline EosSessionConfigBuilder::EnvironmentEditor EosSessionConfigBuilder::Environment() noexcept {
  return EnvironmentEditor(this);
}

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_BUILDER_H_
