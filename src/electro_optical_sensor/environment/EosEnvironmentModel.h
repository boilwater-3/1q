/**
 * @file EosEnvironmentModel.h
 * @brief 定义 EOS 环境模型参数派生接口（内部使用）。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_MODEL_H_
#define ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_MODEL_H_

#include "1q/electro_optical_sensor/session/EosEnvironmentInput.h"

namespace electro_optical_sensor {
namespace environment {

/**
 * @brief 解析当前环境参数对辐射传输的修正。
 * @param[in] inputs 环境模型输入。
 * @return 环境模型输出。
 */
session::EosEnvironmentModelResult ResolveEnvironmentFactors(const session::EosEnvironmentModelInputs& inputs);

}  // namespace environment
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_ENVIRONMENT_EOS_ENVIRONMENT_MODEL_H_
