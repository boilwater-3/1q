/**
 * @file SbirsEnvironmentModel.h
 * @brief SBIRS-inspired 气象衰减模型。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_ENVIRONMENT_SBIRS_ENVIRONMENT_MODEL_H_
#define ONEQ_SRC_SBIRS_SENSOR_ENVIRONMENT_SBIRS_ENVIRONMENT_MODEL_H_

#include "1q/sbirs_sensor/config/SbirsEnvironmentConfig.h"

namespace sbirs_sensor {
namespace environment {

/**
 * @brief 计算气象衰减比例 `A_total`（独立项加权叠加 + 可选交互项）。
 * @param[in] environment 环境与气象配置
 * @return 衰减比例，夹紧到 [0, 1]（1 表示完全衰减）
 * @note 内部实现细节。交互项仅在对应权重 > 0 时启用，默认 0 保持向后兼容。
 */
float ResolveWeatherAttenuation(const config::SbirsEnvironmentConfig& environment);
/**
 * @brief 计算气象修正后的有效透过率 τ_eff = base × (1 − A_total)。
 * @param[in] environment 环境与气象配置
 * @return 有效透过率，夹紧到 [0, 1]
 */
float ResolveEffectiveTransmittance(const config::SbirsEnvironmentConfig& environment);

}  // namespace environment
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_ENVIRONMENT_SBIRS_ENVIRONMENT_MODEL_H_
