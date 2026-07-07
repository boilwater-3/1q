/**
 * @file SbirsRadiometry.h
 * @brief SBIRS-inspired 红外辐射与 SNR 标量链路。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_FOUNDATION_SBIRS_RADIOMETRY_H_
#define ONEQ_SRC_SBIRS_SENSOR_FOUNDATION_SBIRS_RADIOMETRY_H_

#include "1q/sbirs_sensor/config/SbirsHardwareConfig.h"

namespace sbirs_sensor {
namespace foundation {

/**
 * @brief 计算指定波长下的普朗克谱辐射亮度。
 * @param[in] wavelength_um 波长，单位 μm
 * @param[in] temperature_k 目标温度，单位 K
 * @return 谱辐射亮度；波长或温度非正时返回 0
 */
double ComputePlanckRadiance(double wavelength_um, double temperature_k);
/**
 * @brief 计算波段积分辐射亮度（取波段中心 Planck 值 × 带宽近似）。
 * @param[in] wavelength_lower_um 波段下限，单位 μm
 * @param[in] wavelength_upper_um 波段上限，单位 μm
 * @param[in] temperature_k 目标温度，单位 K
 * @return 波段辐射亮度
 */
double ComputeBandRadiance(double wavelength_lower_um, double wavelength_upper_um,
                           double temperature_k);
/**
 * @brief 计算探测器接收功率 P_sig = Φ_atm · A_det · η / d²。
 * @param[in] band_radiance 入射波段辐射亮度
 * @param[in] projected_area_m2 目标投影面积，单位 m²
 * @param[in] range_m 目标距离，单位 m
 * @param[in] aperture_m 光学孔径，单位 m
 * @param[in] optical_transmission 光学透过率
 * @param[in] path_transmittance 路径透过率（含气象修正）
 * @param[in] detector_quantum_efficiency 探测器量子效率
 * @return 接收功率，单位 W；距离或孔径非正时返回 0
 */
double ComputeReceivedPowerW(double band_radiance, double projected_area_m2, double range_m,
                             double aperture_m, double optical_transmission,
                             double path_transmittance, double detector_quantum_efficiency);
/**
 * @brief 计算线性红外 IR SNR = P_sig · t_int / N。
 * @param[in] received_power_w 接收功率，单位 W
 * @param[in] hardware 硬件配置（取积分时间与 NEP）
 * @return 线性 IR SNR
 */
double ComputeInfraredSnrLinear(double received_power_w,
                                const config::SbirsHardwareConfig& hardware);

}  // namespace foundation
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_FOUNDATION_SBIRS_RADIOMETRY_H_
