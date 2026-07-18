/**
 * @file EosHardwareConfig.h
 * @brief 定义 EOS 硬件域配置。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_HARDWARE_CONFIG_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_HARDWARE_CONFIG_H_

#include "1q/api.hpp"

namespace electro_optical_sensor {
namespace config {

/**
 * @brief EosHardwareConfig 描述外部可观测的硬件规格。
 */
struct ONEQ_API EosHardwareConfig {
  float wavelength_lower_um{3.0f};               /**< 工作波段下限（单位：um） */
  float wavelength_upper_um{5.0f};               /**< 工作波段上限（单位：um） */
  float optical_aperture_m{0.2f};                /**< 光学口径（单位：m） */
  float focal_length_m{0.8f};                    /**< 焦距（单位：m） */
  float detector_detectivity_cm_sqrt_hz_per_w{1.0e10f}; /**< 探测器比探测率（D*） */
  /** 探测器面积（cm²），与厘米制 D* 共同进入 NEP 计算。 */
  float detector_area_cm2{0.25f};
  float min_detection_depression_deg{1.0f};              /**< 最小探测俯仰角（单位：deg） */
  float max_detection_depression_deg{89.0f};             /**< 最大探测俯仰角（单位：deg） */
};

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_HARDWARE_CONFIG_H_
