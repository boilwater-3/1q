/**
 * @file EosCycleInput.h
 * @brief 定义光学传感器组件单周期输入载荷。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CORE_CONTEXT_EOS_CYCLE_INPUT_H_
#define ELECTRO_OPTICAL_SENSOR_CORE_CONTEXT_EOS_CYCLE_INPUT_H_

#include <cstdint>

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/session/EosEnvironmentInput.h"
#include "1q/electro_optical_sensor/session/EosSceneInput.h"
#include "1q/foundation/pose_types.h"

namespace electro_optical_sensor {
namespace session {

/**
 * @brief EosCycleInput 描述光学传感器单周期输入。
 */
struct ONEQ_API EosCycleInput {
  std::uint32_t cycle_index{0U};            /**< 当前周期号 */
  float dt_sec{1.0f};                       /**< 当前周期步长（单位：s） */
  oneq::foundation::PoseState platform_pose{};  /**< 平台位姿状态 */
  EosSceneInput scene{};                    /**< 当前周期场景实体输入 */
  EosEnvironmentInput environment{};        /**< 当前周期环境事实输入 */
};

}  // namespace session

}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CORE_CONTEXT_EOS_CYCLE_INPUT_H_
