/**
 * @file ExecutionDefaults.h
 * @brief 定义内部执行配置的默认值与安全兜底常量。
 */

#ifndef AIRBORNE_RADAR_SRC_CONFIG_DEFAULTS_EXECUTION_DEFAULTS_H_
#define AIRBORNE_RADAR_SRC_CONFIG_DEFAULTS_EXECUTION_DEFAULTS_H_

#include <vector>

namespace airborne_radar {
namespace config {
namespace defaults {

inline std::vector<float> DefaultImmModelNoiseDiffCoeffs() {
  return std::vector<float>{0.5f, 4.0f};
}

}  // namespace defaults
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_CONFIG_DEFAULTS_EXECUTION_DEFAULTS_H_
