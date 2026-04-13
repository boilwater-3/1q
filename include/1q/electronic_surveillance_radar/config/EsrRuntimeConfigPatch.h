/**
 * @file EsrRuntimeConfigPatch.h
 * @brief 定义 ESR 会话运行期配置补丁结构。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_RUNTIME_CONFIG_PATCH_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_RUNTIME_CONFIG_PATCH_H_

#include "1q/electronic_surveillance_radar/config/EsrSessionConfig.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief EsrRuntimeConfigPatch 描述运行期可变参数补丁。
 */
struct ONEQ_API EsrRuntimeConfigPatch {
  bool has_sensor_enabled{false};
  bool sensor_enabled{true};

  bool has_scan_rate_hz{false};
  float scan_rate_hz{1.0f};

  bool has_integrated_receive_loss_db{false};
  float integrated_receive_loss_db{0.0f};

  bool has_fixed_receiver_window_hz{false};
  double receiver_lower_hz{0.0};
  double receiver_upper_hz{0.0};

  bool has_use_fixed_receiver_window{false};
  bool use_fixed_receiver_window{true};

  bool has_enable_statistical_detection{false};
  bool enable_statistical_detection{true};

  bool has_enable_spectral_analysis{false};
  bool enable_spectral_analysis{true};

  bool has_detection_min_snr_db{false};
  float detection_min_snr_db{6.0f};

  bool has_environment_runtime_config{false};
  environment::EsrEnvironmentRuntimeConfigPatch environment_runtime_config{};

  bool has_observation_jam_mark_threshold_w{false};
  float observation_jam_mark_threshold_w{0.0f};
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_RUNTIME_CONFIG_PATCH_H_
