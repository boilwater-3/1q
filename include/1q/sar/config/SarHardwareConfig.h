/**
 * @file SarHardwareConfig.h
 * @brief 定义 SAR 传感器硬件与波形基础配置。
 */

#ifndef ONEQ_SAR_CONFIG_SAR_HARDWARE_CONFIG_H_
#define ONEQ_SAR_CONFIG_SAR_HARDWARE_CONFIG_H_

#include "1q/api.hpp"

namespace sar {
namespace config {

/**
 * @brief SAR 传感器硬件与线性调频波形配置。
 */
struct ONEQ_API SarHardwareConfig {
  double carrier_frequency_hz{9.6e9};
  double bandwidth_hz{100.0e6};
  double pulse_width_s{20.0e-6};
  double pulse_repetition_frequency_hz{1500.0};
  double sample_rate_hz{120.0e6};
  double peak_power_w{1000.0};
  double antenna_length_m{1.2};
  double antenna_width_m{0.3};
  double antenna_gain_db{30.0};
  double receiver_noise_figure_db{4.0};
  double system_loss_db{3.0};
};

}  // namespace config
}  // namespace sar

#endif  // ONEQ_SAR_CONFIG_SAR_HARDWARE_CONFIG_H_
