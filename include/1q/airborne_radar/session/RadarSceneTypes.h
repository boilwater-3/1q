/**
 * @file RadarSceneTypes.h
 * @brief 定义机载雷达单周期场景实体输入类型。
 */

#ifndef AIRBORNE_RADAR_SESSION_RADAR_SCENE_TYPES_H_
#define AIRBORNE_RADAR_SESSION_RADAR_SCENE_TYPES_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"

namespace airborne_radar {
namespace session {

/**
 * @brief RadarSceneTarget 描述雷达单周期场景目标输入。
 */
struct ONEQ_API RadarSceneTarget {
  std::uint64_t external_target_id{0};   /**< 外部输入原始目标标识符（0 表示未知/未提供） */
  float velocity_x{0.0f};               /**< 雷达局部坐标速度 x 分量（单位：m/s） */
  float velocity_y{0.0f};               /**< 雷达局部坐标速度 y 分量（单位：m/s） */
  float velocity_z{0.0f};               /**< 雷达局部坐标速度 z 分量（单位：m/s） */
  float rcs{0.0f};                       /**< 目标雷达散射截面积（单位：m^2） */
  float range_m{0.0f};                   /**< 目标到雷达的斜距（单位：m） */
  float position_x{0.0f};                /**< 雷达局部笛卡尔坐标 x（单位：m） */
  float position_y{0.0f};                /**< 雷达局部笛卡尔坐标 y（单位：m） */
  float position_z{0.0f};                /**< 雷达局部笛卡尔坐标 z（单位：m） */
  int target_swerling_type{0};           /**< 目标起伏模型 */

  RadarSceneTarget() = default;
  RadarSceneTarget(float velocity_x_mps_in, float velocity_y_mps_in, float velocity_z_mps_in,
                   float rcs_m2_in, float range_m_in = 0.0f, int swerling_type_in = 0,
                   std::uint64_t external_target_id_in = 0)
      : external_target_id(external_target_id_in),
        velocity_x(velocity_x_mps_in),
        velocity_y(velocity_y_mps_in),
        velocity_z(velocity_z_mps_in),
        rcs(rcs_m2_in),
        range_m(range_m_in),
        target_swerling_type(swerling_type_in) {}
};

/** @brief RadarSceneTargetList 表示雷达场景目标输入列表。 */
using RadarSceneTargetList = std::vector<RadarSceneTarget>;

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SESSION_RADAR_SCENE_TYPES_H_
