/**
 * @file EosWorkMode.h
 * @brief 定义 EOS 工作模式枚举。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_WORK_MODE_H_
#define ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_WORK_MODE_H_

#include "1q/api.hpp"

namespace electro_optical_sensor {
namespace config {

/**
 * @brief EosWorkMode 表示传感器工作模式。
 */
enum class ONEQ_API EosWorkMode {
  kInfraredOnly = 0, /**< 红外探测 */
  kVisibleOnly,      /**< 可见光探测 */
  kFused             /**< 红外/可见光融合探测 */
};

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_WORK_MODE_H_
