/**
 * @file SbirsSceneTypes.h
 * @brief 定义 SBIRS-inspired 场景目标输入类型。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_SCENE_TYPES_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_SCENE_TYPES_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"

namespace sbirs_sensor {
namespace session {

struct ONEQ_API SbirsVector3M {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct ONEQ_API SbirsSceneTarget {
  std::uint64_t target_id{0U};
  std::string target_name{};
  SbirsVector3M position_ecef_m{};
  float temperature_k{1200.0f};
  float emissivity{0.85f};
  float projected_area_m2{1.0f};
  bool active{true};
};

using SbirsSceneTargetList = std::vector<SbirsSceneTarget>;

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_SCENE_TYPES_H_
