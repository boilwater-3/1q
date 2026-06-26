/**
 * @file electro_optical_sensor.hpp
 * @brief 光电传感器模块稳定会话入口。
 * @note 输出类型请直接使用 `session/EosOutputTypes.h`；
 *       trace/replay 工具头按需单独包含。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_ELECTRO_OPTICAL_SENSOR_HPP_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_ELECTRO_OPTICAL_SENSOR_HPP_

#include "1q/electro_optical_sensor/config/electro_optical_sensor_config.hpp"
#include "1q/electro_optical_sensor/session/EosEnvironmentInput.h"
#include "1q/electro_optical_sensor/foundation/EosRadiativeTransfer.h"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosCycleInputBuilder.h"
#include "1q/electro_optical_sensor/session/EosCycleOutputBuilder.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/session/EosEnvironmentInputPatch.h"
#include "1q/electro_optical_sensor/session/EosEnvironmentInputState.h"
#include "1q/electro_optical_sensor/session/EosExternalInputAdapter.h"
#include "1q/electro_optical_sensor/session/EosExternalOutputAdapter.h"
#include "1q/electro_optical_sensor/session/EosInputValidation.h"
#include "1q/electro_optical_sensor/session/EosSession.h"

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_ELECTRO_OPTICAL_SENSOR_HPP_
