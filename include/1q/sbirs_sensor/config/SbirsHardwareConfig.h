/**
 * @file SbirsHardwareConfig.h
 * @brief 定义 SBIRS-inspired 传感器硬件参数。
 */

#ifndef ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_HARDWARE_CONFIG_H_
#define ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_HARDWARE_CONFIG_H_

#include "1q/api.hpp"

namespace sbirs_sensor {
namespace config {

struct ONEQ_API SbirsHardwareConfig {
  float wavelength_lower_um{3.0f};
  float wavelength_upper_um{5.0f};
  float optical_aperture_m{0.5f};
  float detector_area_m2{1.0e-4f};
  float optical_transmission{0.8f};
  float detector_quantum_efficiency{0.7f};
  float integration_time_sec{0.02f};
  float noise_equivalent_power_w{1.0e-12f};
  // 2.8 噪声分解：背景/热项参数（默认 0 表示只使用 NEP 标量）。
  float background_radiance_w_sr_m2{0.0f};   // 背景辐射亮度（用于光子噪声）
  float detector_temperature_k{80.0f};        // 探测器工作温度（热噪声）
  float readout_noise_rms_w{0.0f};            // 读出噪声 RMS（W 等效）
};

}  // namespace config
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_HARDWARE_CONFIG_H_
