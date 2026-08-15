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
 * @brief 计算探测器接收功率 P_sig = I_t · A_ap · τ_opt · τ_atm · η / d²。
 * @details 目标红外签名由调用方以辐射强度（W/sr）直接提供；温度/发射率/投影面积
 *          已由调用方折算进 I_t，本模块不做 Planck 换算。
 * @param[in] radiant_intensity_w_per_sr 目标辐射强度（朝向传感器方向），单位 W/sr
 * @param[in] range_m 目标距离，单位 m
 * @param[in] aperture_m 光学孔径，单位 m
 * @param[in] optical_transmission 光学透过率
 * @param[in] path_transmittance 路径透过率（含气象修正）
 * @param[in] detector_quantum_efficiency 探测器量子效率
 * @return 接收功率，单位 W；辐射强度为负、距离或孔径非正时返回 0
 */
double ComputeReceivedPowerW(double radiant_intensity_w_per_sr, double range_m,
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
