/**
 * @file RadarSceneInput.h
 * @brief 定义机载雷达单周期场景输入聚合类型。
 */

#ifndef AIRBORNE_RADAR_SESSION_RADAR_SCENE_INPUT_H_
#define AIRBORNE_RADAR_SESSION_RADAR_SCENE_INPUT_H_

#include <cmath>
#include <cstdint>
#include <vector>

namespace airborne_radar {
namespace session {

/**
 * @brief RadarSceneTarget 描述雷达单周期场景目标输入。
 */
struct RadarSceneTarget {
  std::uint64_t external_target_id{0};   /**< 外部输入原始目标标识符（0 表示未知/未提供） */
  float current_track_velocity_x{0.0f};  /**< 雷达局部坐标速度 x 分量（单位：m/s） */
  float current_track_velocity_y{0.0f};  /**< 雷达局部坐标速度 y 分量（单位：m/s） */
  float current_track_velocity_z{0.0f};  /**< 雷达局部坐标速度 z 分量（单位：m/s） */
  float current_track_speed{0.0f};       /**< 目标速度模长（单位：m/s） */
  float current_track_rcs{0.0f};         /**< 目标雷达散射截面积（单位：m^2） */
  float range_m{0.0f};                   /**< 目标到雷达的斜距（单位：m） */
  bool has_cartesian_position{false};    /**< 是否显式提供笛卡尔位置 */
  float position_x{0.0f};                /**< 雷达局部笛卡尔坐标 x（单位：m） */
  float position_y{0.0f};                /**< 雷达局部笛卡尔坐标 y（单位：m） */
  float position_z{0.0f};                /**< 雷达局部笛卡尔坐标 z（单位：m） */
  int target_swerling_type{0};           /**< 目标起伏模型 */

  RadarSceneTarget() = default;
  RadarSceneTarget(float velocity_x_mps_in, float velocity_y_mps_in, float velocity_z_mps_in,
                   float rcs_m2_in, float range_m_in = 0.0f, int swerling_type_in = 0,
                   std::uint64_t external_target_id_in = 0)
      : external_target_id(external_target_id_in),
        current_track_velocity_x(velocity_x_mps_in),
        current_track_velocity_y(velocity_y_mps_in),
        current_track_velocity_z(velocity_z_mps_in),
        current_track_speed(std::sqrt(velocity_x_mps_in * velocity_x_mps_in +
                                      velocity_y_mps_in * velocity_y_mps_in +
                                      velocity_z_mps_in * velocity_z_mps_in)),
        current_track_rcs(rcs_m2_in),
        range_m(range_m_in),
        target_swerling_type(swerling_type_in) {}
};

/** @brief RadarSceneTargetList 表示雷达场景目标输入列表。 */
using RadarSceneTargetList = std::vector<RadarSceneTarget>;

/**
 * @brief RadarSceneInput 聚合雷达单周期场景实体输入。
 * @note 当前仅含 `targets`，是统一骨架 `CycleInput.scene` 的固定语义槽位，
 *       用于外部输入边界与内部模型解耦。
 */
struct RadarSceneInput {
  RadarSceneTargetList targets{}; /**< 当前周期场景目标输入列表 */
};

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SESSION_RADAR_SCENE_INPUT_H_
