/**
 * @file ArSceneTypes.h
 * @brief 机载雷达场景实体输入类型集合。
 *
 * 场景目标输入的主头文件。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_SCENE_TYPES_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_SCENE_TYPES_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"

namespace airborne_radar {
namespace session {

/**
 * @brief ArSceneTarget 描述雷达单周期场景目标输入。
 */
struct ONEQ_API ArSceneTarget {
  std::uint64_t external_target_id{0};   /**< 外部输入原始目标标识符（0 表示未知/未提供） */
  std::string target_name{};             /**< 可选目标名称，仅用于人读、trace 与调试视图，不参与关联 */
  float velocity_x{0.0f};               /**< 雷达局部坐标速度 x 分量（单位：m/s） */
  float velocity_y{0.0f};               /**< 雷达局部坐标速度 y 分量（单位：m/s） */
  float velocity_z{0.0f};               /**< 雷达局部坐标速度 z 分量（单位：m/s） */
  float rcs{0.0f};                       /**< 目标雷达散射截面积（单位：m^2） */
  float range_m{0.0f};                   /**< 目标到雷达的斜距（单位：m） */
  float position_x{0.0f};                /**< 雷达局部笛卡尔坐标 x（单位：m） */
  float position_y{0.0f};                /**< 雷达局部笛卡尔坐标 y（单位：m） */
  float position_z{0.0f};                /**< 雷达局部笛卡尔坐标 z（单位：m） */
  int target_swerling_type{0};           /**< 目标起伏模型 */

  ArSceneTarget() = default;
  ArSceneTarget(float velocity_x_mps_in, float velocity_y_mps_in, float velocity_z_mps_in,
                float rcs_m2_in, float range_m_in = 0.0f, int swerling_type_in = 0,
                std::uint64_t external_target_id_in = 0, std::string target_name_in = {})
      : external_target_id(external_target_id_in),
        target_name(std::move(target_name_in)),
        velocity_x(velocity_x_mps_in),
        velocity_y(velocity_y_mps_in),
        velocity_z(velocity_z_mps_in),
        rcs(rcs_m2_in),
        range_m(range_m_in),
        target_swerling_type(swerling_type_in) {}
};

/**
 * @brief ArTargetInput 描述 AR 单周期场景目标输入（平台锚点 radar-local ENU）。
 * @note ENU 契约见 docs/common/contract.md「场景目标平台锚点 ENU 输入契约」：
 *       原点 = 当周期平台 ECEF 位置（逐周期重锚），轴 = 锚点 ENU（x=东/y=北/z=天）；
 *       速度 = 目标 ECEF 速度旋入锚点 ENU 轴（固定锚点旋转，无传输率修正）。
 *       集成层以 `oneq::coordinate::TryEcefToLla`（锚点）+ `TryMakeEnuSceneState`
 *       （逐目标）完成 ECEF/LLA→ENU 转换后直填本结构。
 * @note 与 `ArSceneTarget`（库内雷达局部系）区分：本结构是外部输入面，库内旋入
 *       雷达体系（平台姿态∘安装角复合）后使用。
 */
struct ONEQ_API ArTargetInput {
  std::uint64_t target_id{0U}; /**< 外部输入原始目标标识符（0 表示未知/未提供） */
  std::string target_name{};   /**< 可选目标名称，仅用于人读、trace 与调试视图，不参与关联 */
  float position_x{0.0f};      /**< 平台锚点 ENU 位置 x（东向，单位：m） */
  float position_y{0.0f};      /**< 平台锚点 ENU 位置 y（北向，单位：m） */
  float position_z{0.0f};      /**< 平台锚点 ENU 位置 z（天向，单位：m） */
  float velocity_x{0.0f};      /**< 锚点 ENU 速度 x 分量（单位：m/s） */
  float velocity_y{0.0f};      /**< 锚点 ENU 速度 y 分量（单位：m/s） */
  float velocity_z{0.0f};      /**< 锚点 ENU 速度 z 分量（单位：m/s） */
  float rcs{1.0f};             /**< 目标雷达散射截面积（单位：m^2） */
  int swerling_type{0};        /**< 目标起伏模型 */

  ArTargetInput() = default;
};

/** @brief ArTargetInputList 表示 AR 场景目标输入列表。 */
using ArTargetInputList = std::vector<ArTargetInput>;

/** @brief ArSceneTargetList 表示雷达场景目标输入列表。 */
using ArSceneTargetList = std::vector<ArSceneTarget>;


}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_SCENE_TYPES_H_
