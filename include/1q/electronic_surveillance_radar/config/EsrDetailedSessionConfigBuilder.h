/**
 * @file EsrDetailedSessionConfigBuilder.h
 * @brief ESR 细粒度会话配置构造器。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_DETAILED_SESSION_CONFIG_BUILDER_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_DETAILED_SESSION_CONFIG_BUILDER_H_

#include "1q/electronic_surveillance_radar/config/EsrSessionConfig.h"

namespace electronic_surveillance_radar {
namespace config {

/**
 * @brief EsrDetailedSessionConfigBuilder 提供四域详细配置构造入口。
 *
 * 面向高级调用方，可直接编辑 hardware/mission/policy/environment
 * 以及策略详细参数。
 *
 * @note 推荐路径：该构造器用于高级调参与工程化配置；
 *       常规业务初始化优先使用 EsrSessionConfigBuilder。
 */
class ONEQ_API EsrDetailedSessionConfigBuilder {
 public:
  explicit EsrDetailedSessionConfigBuilder(const session::EsrSessionConfig& config = {})
      : config_(config) {}

  EsrDetailedSessionConfigBuilder& WithSessionConfig(const session::EsrSessionConfig& config) {
    config_ = config;
    return *this;
  }

  EsrDetailedSessionConfigBuilder& WithHardware(const EsrHardwareConfig& hardware) {
    config_.hardware = hardware;
    return *this;
  }

  EsrDetailedSessionConfigBuilder& WithMission(const EsrMissionConfig& mission) {
    config_.mission = mission;
    return *this;
  }

  EsrDetailedSessionConfigBuilder& WithPolicy(const EsrPolicyConfig& policy) {
    config_.policy = policy;
    return *this;
  }

  EsrDetailedSessionConfigBuilder& WithEnvironment(const EsrEnvironmentConfig& environment) {
    config_.environment = environment;
    return *this;
  }

  EsrDetailedSessionConfigBuilder& WithPowerOn(bool power_on) {
    config_.mission.power_on = power_on;
    return *this;
  }

  EsrDetailedSessionConfigBuilder& WithWorkMode(EsrWorkMode mode) {
    config_.mission.work_mode = mode;
    return *this;
  }

  EsrDetailedSessionConfigBuilder& WithScanRateHz(float value) {
    config_.mission.scan.scan_rate_hz = value;
    return *this;
  }

  EsrDetailedSessionConfigBuilder& WithScanCenterAzDeg(float value) {
    config_.mission.scan.scan_center_az_deg = value;
    return *this;
  }

  EsrDetailedSessionConfigBuilder& WithScanCenterElDeg(float value) {
    config_.mission.scan.scan_center_el_deg = value;
    return *this;
  }

  EsrDetailedSessionConfigBuilder& WithScanStartPosition(EsrScanStartPosition position) {
    config_.mission.scan.scan_start_position = position;
    return *this;
  }

  EsrDetailedSessionConfigBuilder& WithScanSequence(EsrScanSequence sequence) {
    config_.mission.scan.scan_sequence = sequence;
    return *this;
  }

  EsrDetailedSessionConfigBuilder& WithExplicitScanBoundsDeg(float start_az, float end_az,
                                                             float start_el, float end_el) {
    config_.mission.scan.use_explicit_scan_bounds = true;
    config_.mission.scan.scan_start_az_deg = start_az;
    config_.mission.scan.scan_end_az_deg = end_az;
    config_.mission.scan.scan_start_el_deg = start_el;
    config_.mission.scan.scan_end_el_deg = end_el;
    return *this;
  }

  EsrDetailedSessionConfigBuilder& SetUseExplicitScanBounds(bool enable) {
    config_.mission.scan.use_explicit_scan_bounds = enable;
    return *this;
  }

  EsrDetailedSessionConfigBuilder& WithDetectionProfile(EsrDetectionProfile profile) {
    config_.policy.detection.profile = profile;
    config_.policy.detection.use_profile_defaults = true;
    return *this;
  }

  EsrDetailedSessionConfigBuilder& WithDetectionDetails(float min_detect_snr_db, float pfa,
                                                        std::uint32_t pulse_count,
                                                        float threshold_scale,
                                                        bool enable_statistical_detection) {
    config_.policy.detection.use_profile_defaults = false;
    config_.policy.detection.min_detect_snr_db = min_detect_snr_db;
    config_.policy.detection.pfa = pfa;
    config_.policy.detection.pulse_count = pulse_count;
    config_.policy.detection.threshold_scale = threshold_scale;
    config_.policy.detection.enable_statistical_detection = enable_statistical_detection;
    return *this;
  }

  EsrDetailedSessionConfigBuilder& WithEnvironmentPreset(EsrEnvironmentPreset preset) {
    config_.environment.scenario_config.preset = preset;
    return *this;
  }

  EsrDetailedSessionConfigBuilder& WithAtmosphericPhysicsConfig(
      const EsrAtmosphericPhysicsConfig& atmospheric_physics) {
    config_.environment.scenario_config.atmospheric_physics = atmospheric_physics;
    return *this;
  }

  EsrDetailedSessionConfigBuilder& WithAtmosphericContext(
      const EsrAtmosphericDerivedContext& atmospheric_context) {
    config_.environment.scenario_config.atmospheric_context = atmospheric_context;
    return *this;
  }

  session::EsrSessionConfig Build() const { return config_; }

 private:
  session::EsrSessionConfig config_{};
};

}  // namespace config
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_DETAILED_SESSION_CONFIG_BUILDER_H_
