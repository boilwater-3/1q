/**
 * @file EsrSessionConfigBuilder.h
 * @brief ESR 对外配置入口：初始化配置类型与 SessionConfig Builder。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_BUILDER_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_BUILDER_H_

#include "1q/electronic_surveillance_radar/session/EsrSession.h"

namespace electronic_surveillance_radar {
namespace config {

/**
 * @brief EsrSessionConfigBuilder 提供初始化配置链式构造。
 */
class ONEQ_API EsrSessionConfigBuilder {
 public:
  explicit EsrSessionConfigBuilder(
      const session::EsrSessionConfig& config = {}) : config_(config) {}

  EsrSessionConfigBuilder& EnableLayeredConfig(bool enable = true) {
    config_.enable_layered_config = enable;
    return *this;
  }
  EsrSessionConfigBuilder& WithSessionConfig(const session::EsrSessionConfig& config) {
    config_ = config;
    return *this;
  }
  EsrSessionConfigBuilder& WithWorkMode(session::EsrWorkMode mode) {
    config_.layered_config.mission.work_mode = mode;
    return *this;
  }
  EsrSessionConfigBuilder& WithScanRateHz(float value) {
    config_.layered_config.mission.scan_rate_hz = value;
    return *this;
  }
  EsrSessionConfigBuilder& EnableSpectralAnalysis(bool enable = true) {
    config_.pipeline_config.spectral_analysis.enable = enable;
    return *this;
  }
  EsrSessionConfigBuilder& WithDetectionMinSnrDb(float value) {
    config_.pipeline_config.detection.min_detect_snr_db = value;
    return *this;
  }
  EsrSessionConfigBuilder& WithJammingDetectionThresholdW(float value) {
    config_.environment_default_config.model_config.jamming_detection_threshold_w = value;
    return *this;
  }
  session::EsrSessionConfig Build() const { return config_; }

 private:
  session::EsrSessionConfig config_{};
};

}  // namespace config
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_BUILDER_H_
