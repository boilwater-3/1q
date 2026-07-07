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

/** @brief ECEF 坐标系下的三维位置向量，单位 m。 */
struct ONEQ_API SbirsVector3M {
  double x{0.0}; /**< X 分量，单位 m */
  double y{0.0}; /**< Y 分量，单位 m */
  double z{0.0}; /**< Z 分量，单位 m */
};

/**
 * @brief 输入场景中的单个目标描述（仿真真值）。
 * @note 纯数据类型 (POD)。`target_id` 是目标级状态机的键；真值位置仅用于真值辅助跟踪
 *       与仿真判定，不进入 `SbirsOutputFrame` raw output。速度真值用于 cue 延迟外推与
 *       动态滞后误差，缺省（`has_velocity_ecef_m_per_s=false`）时按 0 处理，保持旧行为。
 */
struct ONEQ_API SbirsSceneTarget {
  std::uint64_t target_id{0U};     /**< 目标唯一标识，状态机以此键管理状态 */
  std::string target_name{};       /**< 目标名称，仅进入归属/调试层，不进 raw output */
  SbirsVector3M position_ecef_m{}; /**< ECEF 位置真值，单位 m */
  float temperature_k{1200.0f};    /**< 目标温度，单位 K */
  float emissivity{0.85f};         /**< 发射率，无量纲 */
  float projected_area_m2{1.0f};   /**< 投影面积，单位 m² */
  SbirsVector3M velocity_ecef_m_per_s{}; /**< ECEF 速度真值，单位 m/s（仅 cue 外推与动态滞后用） */
  bool has_velocity_ecef_m_per_s{false}; /**< 是否提供速度；为 false 时按 0 处理，保持旧行为 */
  bool active{true};               /**< 目标是否在场景中有效 */
};

/** @brief 目标列表。 */
using SbirsSceneTargetList = std::vector<SbirsSceneTarget>;

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_SCENE_TYPES_H_
