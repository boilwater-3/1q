/**
 * @file EosOpticalConfig.h
 * @brief 定义 EOS 光学硬件配置。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_OPTICAL_CONFIG_H_
#define ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_OPTICAL_CONFIG_H_

namespace electro_optical_sensor {
namespace config {

/**
 * @brief EosOpticalHardwareConfig 描述外部可观测的光学硬件规格。
 */
struct EosOpticalHardwareConfig {
  float wavelength_lower_um{3.0f}; /**< 工作波段下限（单位：um） */
  float wavelength_upper_um{5.0f}; /**< 工作波段上限（单位：um） */
  float optical_aperture_m{0.2f};  /**< 光学口径（单位：m） */
  float focal_length_m{0.8f};      /**< 焦距（单位：m） */
};

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_OPTICAL_CONFIG_H_
