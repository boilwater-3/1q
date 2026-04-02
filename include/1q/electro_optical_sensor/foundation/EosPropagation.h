/**
 * @file EosPropagation.h
 * @brief 定义大气传输、接收功率与 SNR 计算函数。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_FOUNDATION_EOS_PROPAGATION_H_
#define ELECTRO_OPTICAL_SENSOR_FOUNDATION_EOS_PROPAGATION_H_

#include "1q/api.hpp"

namespace electro_optical_sensor {
namespace foundation {
namespace propagation {

/**
 * @brief 计算 Beer-Lambert 透过率 `exp(-alpha * L)`。
 * @param[in] attenuation_coeff_per_m 衰减系数（单位：1/m）。
 * @param[in] path_length_m 传播距离（单位：m）。
 * @return 透过率，范围 [0, 1]。
 */
ONEQ_API float ComputeBeerLambertTransmittance(float attenuation_coeff_per_m, float path_length_m);

/**
 * @brief 计算合成大气透过率。
 * @param[in] base_extinction_coeff_per_m 基础消光系数（单位：1/m）。
 * @param[in] humidity_absorption_coeff_per_m 湿度吸收系数（单位：1/m）。
 * @param[in] path_length_m 传播距离（单位：m）。
 * @return 透过率，范围 [0, 1]。
 */
ONEQ_API float ComputeAtmosphericTransmittance(float base_extinction_coeff_per_m,
                                               float humidity_absorption_coeff_per_m,
                                               float path_length_m);

/**
 * @brief 计算背景通量。
 * @param[in] background_radiance_w_sr_m2 背景辐亮度（单位：W/(sr*m^2)）。
 * @param[in] aperture_area_m2 入瞳面积（单位：m^2）。
 * @param[in] fov_solid_angle_sr 视场立体角（单位：sr）。
 * @param[in] optical_transmittance 光学系统透过率，范围 [0, 1]。
 * @return 背景通量（单位：W）。
 */
ONEQ_API float ComputeBackgroundFluxW(float background_radiance_w_sr_m2, float aperture_area_m2,
                                      float fov_solid_angle_sr, float optical_transmittance);

/**
 * @brief 计算焦平面接收功率。
 * @param[in] source_radiance_w_sr_m2 源辐亮度（单位：W/(sr*m^2)）。
 * @param[in] projected_area_m2 目标投影面积（单位：m^2）。
 * @param[in] range_m 目标距离（单位：m）。
 * @param[in] aperture_area_m2 入瞳面积（单位：m^2）。
 * @param[in] atmospheric_transmittance 大气透过率，范围 [0, 1]。
 * @param[in] optical_transmittance 光学系统透过率，范围 [0, 1]。
 * @return 接收功率（单位：W）。
 */
ONEQ_API float ComputeReceivedPowerW(float source_radiance_w_sr_m2, float projected_area_m2,
                                     float range_m, float aperture_area_m2,
                                     float atmospheric_transmittance,
                                     float optical_transmittance);

/**
 * @brief 计算线性信噪比。
 * @param[in] received_power_w 接收功率（单位：W）。
 * @param[in] nep_w 噪声等效功率（单位：W）。
 * @return 线性 SNR。
 */
ONEQ_API float ComputeSnrLinear(float received_power_w, float nep_w);

/**
 * @brief 计算 dB 信噪比。
 * @param[in] snr_linear 线性 SNR。
 * @return dB SNR。
 */
ONEQ_API float ComputeSnrDb(float snr_linear);

}  // namespace propagation
}  // namespace foundation
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_FOUNDATION_EOS_PROPAGATION_H_
