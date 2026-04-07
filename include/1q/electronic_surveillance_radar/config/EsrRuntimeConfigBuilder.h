/**
 * @file EsrRuntimeConfigBuilder.h
 * @brief ESR 对外配置入口：运行期补丁类型与 RuntimeConfig Builder。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_RUNTIME_CONFIG_BUILDER_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_RUNTIME_CONFIG_BUILDER_H_

#include "1q/electronic_surveillance_radar/config/EsrSessionConfigBuilder.h"

namespace electronic_surveillance_radar {
namespace config {

/**
 * @brief EsrRuntimeConfigBuilder 提供运行期补丁链式构造。
 */
class ONEQ_API EsrRuntimeConfigBuilder {
 public:
  explicit EsrRuntimeConfigBuilder(
      const core::session::EsrRuntimeConfigPatch& patch = {}) : patch_(patch) {}

  EsrRuntimeConfigBuilder& WithRuntimeConfigPatch(
      const core::session::EsrRuntimeConfigPatch& patch) {
    patch_ = patch;
    return *this;
  }

  EsrRuntimeConfigBuilder& WithSensorEnabled(bool enable) {
    patch_.has_sensor_enabled = true;
    patch_.sensor_enabled = enable;
    return *this;
  }
  EsrRuntimeConfigBuilder& WithScanRateHz(float value) {
    patch_.has_scan_rate_hz = true;
    patch_.scan_rate_hz = value;
    return *this;
  }
  EsrRuntimeConfigBuilder& WithIntegratedReceiveLossDb(float value) {
    patch_.has_integrated_receive_loss_db = true;
    patch_.integrated_receive_loss_db = value;
    return *this;
  }
  EsrRuntimeConfigBuilder& WithFixedReceiverWindowHz(double lower_hz, double upper_hz) {
    patch_.has_fixed_receiver_window_hz = true;
    patch_.has_use_fixed_receiver_window = true;
    patch_.use_fixed_receiver_window = true;
    patch_.receiver_lower_hz = lower_hz;
    patch_.receiver_upper_hz = upper_hz;
    return *this;
  }
  EsrRuntimeConfigBuilder& SetFixedReceiverWindowEnabled(bool enable) {
    patch_.has_use_fixed_receiver_window = true;
    patch_.use_fixed_receiver_window = enable;
    return *this;
  }
  EsrRuntimeConfigBuilder& DisableFixedReceiverWindow(bool disable = true) {
    patch_.has_use_fixed_receiver_window = true;
    patch_.use_fixed_receiver_window = !disable;
    return *this;
  }
  EsrRuntimeConfigBuilder& EnableStatisticalDetection(bool enable = true) {
    patch_.has_enable_statistical_detection = true;
    patch_.enable_statistical_detection = enable;
    return *this;
  }
  EsrRuntimeConfigBuilder& EnableSpectralAnalysis(bool enable = true) {
    patch_.has_enable_spectral_analysis = true;
    patch_.enable_spectral_analysis = enable;
    return *this;
  }
  EsrRuntimeConfigBuilder& WithDetectionMinSnrDb(float value) {
    patch_.has_detection_min_snr_db = true;
    patch_.detection_min_snr_db = value;
    return *this;
  }
  EsrRuntimeConfigBuilder& WithJammingDetectionThresholdW(float value) {
    patch_.has_environment_runtime_config = true;
    patch_.environment_runtime_config.has_jamming_detection_threshold_w = true;
    patch_.environment_runtime_config.jamming_detection_threshold_w = value;
    return *this;
  }
  EsrRuntimeConfigBuilder& WithEnvironmentRuntimeConfigPatch(
      const environment::EsrEnvironmentRuntimeConfigPatch& patch) {
    patch_.has_environment_runtime_config = true;
    patch_.environment_runtime_config = patch;
    return *this;
  }
  EsrRuntimeConfigBuilder& WithObservationJamMarkThresholdW(float value) {
    patch_.has_observation_jam_mark_threshold_w = true;
    patch_.observation_jam_mark_threshold_w = value;
    return *this;
  }
  core::session::EsrRuntimeConfigPatch Build() const { return patch_; }

 private:
  core::session::EsrRuntimeConfigPatch patch_{};
};

}  // namespace config
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_RUNTIME_CONFIG_BUILDER_H_
