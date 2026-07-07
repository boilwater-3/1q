/**
 * @file SbirsHardwareConfig.h
 * @brief 定义 SBIRS-inspired 传感器硬件参数。
 */

#ifndef ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_HARDWARE_CONFIG_H_
#define ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_HARDWARE_CONFIG_H_

#include "1q/api.hpp"

namespace sbirs_sensor {
namespace config {

/**
 * @brief SBIRS-inspired 传感器硬件参数。
 * @note 纯数据类型 (POD)。波段、孔径等参数进入 foundation 物理链路（Planck 辐射、
 *       接收功率、噪声合成、IR SNR）。背景/热/读出三项 RMS 默认为 0 时回退到 NEP 标量。
 */
struct ONEQ_API SbirsHardwareConfig {
  float wavelength_lower_um{3.0f};   /**< 工作波段下限，单位 μm（默认 MWIR 下沿 3.0） */
  float wavelength_upper_um{5.0f};   /**< 工作波段上限，单位 μm（默认 MWIR 上沿 5.0） */
  float optical_aperture_m{0.5f};    /**< 光学孔径，单位 m */
  float detector_area_m2{1.0e-4f};   /**< 探测器像元面积，单位 m² */
  float optical_transmission{0.8f};  /**< 光学透过率，无量纲 */
  float detector_quantum_efficiency{0.7f}; /**< 探测器量子效率，无量纲 */
  float integration_time_sec{0.02f};       /**< 积分时间，单位 s */
  float noise_equivalent_power_w{1.0e-12f}; /**< 等效噪声功率 NEP，单位 W */
  // 2.8 噪声分解：背景/热项参数（默认 0 表示只使用 NEP 标量）。
  float background_radiance_w_sr_m2{0.0f};   /**< 背景辐射亮度，用于光子噪声（W/(sr·m²)） */
  float detector_temperature_k{80.0f};        /**< 探测器工作温度，用于热噪声，单位 K */
  float readout_noise_rms_w{0.0f};            /**< 读出噪声 RMS，单位 W 等效 */
};

}  // namespace config
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_CONFIG_SBIRS_HARDWARE_CONFIG_H_
