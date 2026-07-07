/**
 * @file SbirsNoiseModel.h
 * @brief SBIRS-inspired 红外噪声分解（design 2.8）。
 *
 * 将原 NEP 标量分解为光子噪声、热噪声、读出噪声三项 RMS 合成：
 *   N_total = sqrt(N_photon² + N_thermal² + N_readout²)
 * 其中：
 *   - 光子噪声：背景辐射在探测器上产生的信号散粒噪声。
 *   - 热噪声：探测器工作温度相关的 Johnson 噪声等效功率。
 *   - 读出噪声：读出电路等效噪声（W）。
 * 若三项背景/热/读出参数均为 0（默认），退化为单一 NEP 标量，保持向后兼容。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_FOUNDATION_SBIRS_NOISE_MODEL_H_
#define ONEQ_SRC_SBIRS_SENSOR_FOUNDATION_SBIRS_NOISE_MODEL_H_

#include "1q/sbirs_sensor/config/SbirsHardwareConfig.h"

namespace sbirs_sensor {
namespace foundation {

/**
 * @brief 噪声分量统计（design 2.8 ComputeBackgroundNoiseStatistics）。
 * @note 内部数据结构，承载光子/热/读出三项 RMS 及其合成总噪声。
 */
struct SbirsNoiseStatistics {
  double photon_noise_w{0.0};    /**< 光子（散粒）噪声 RMS，单位 W */
  double thermal_noise_w{0.0};   /**< 热噪声 RMS，单位 W */
  double readout_noise_w{0.0};   /**< 读出噪声 RMS，单位 W */
  double total_noise_w{0.0};     /**< 三项 RMS 合成总噪声，单位 W */
};

/**
 * @brief 计算背景噪声统计。
 *
 * 背景辐射经孔径与积分时间转换为光子噪声；热噪声由探测器温度与带宽估算；
 * 读出噪声取硬件 readout_noise_rms_w。三者均方根合成得 total_noise_w。
 * @param[in] hardware 硬件配置
 * @return 噪声分量统计
 */
SbirsNoiseStatistics ComputeBackgroundNoiseStatistics(const config::SbirsHardwareConfig& hardware);

/**
 * @brief 由噪声统计与硬件 NEP 取大，得到有效噪声（W）。
 *
 * 当背景/热/读出参数为默认 0 时，total_noise_w 为 0，回退到 NEP，保持向后兼容。
 * @param[in] hardware 硬件配置（取 NEP）
 * @param[in] statistics 噪声分量统计
 * @return 有效噪声，单位 W
 */
double ResolveEffectiveNoiseW(const config::SbirsHardwareConfig& hardware,
                              const SbirsNoiseStatistics& statistics);

}  // namespace foundation
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_FOUNDATION_SBIRS_NOISE_MODEL_H_
