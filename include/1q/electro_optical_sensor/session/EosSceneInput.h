/**
 * @file EosSceneInput.h
 * @brief 定义 EOS 单周期场景输入聚合类型。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SCENE_INPUT_H_
#define ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SCENE_INPUT_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"

namespace electro_optical_sensor {
namespace session {

/**
 * @brief EosSceneTarget 描述单目标输入特征。
 */
struct ONEQ_API EosSceneTarget {
  std::uint64_t target_id{0U};          /**< 目标标识 */
  float range_m{0.0f};                  /**< 斜距（单位：m） */
  float azimuth_deg{0.0f};              /**< 目标方位角（单位：deg） */
  float elevation_deg{0.0f};            /**< 目标仰角（单位：deg） */
  float apparent_temperature_k{290.0f}; /**< 目标等效温度（单位：K） */
  float emissivity{0.9f};               /**< 红外辐射效率，范围 [0, 1] */
  float reflectance{0.2f};              /**< 可见光反射率，范围 [0, 1] */
  float projected_area_m2{1.0f};        /**< 等效投影面积（单位：m^2） */
};

/** @brief EosSceneTargetList 表示 EOS 场景目标输入列表。 */
using EosSceneTargetList = std::vector<EosSceneTarget>;
using EosTargetState = EosSceneTarget;
using EosTargetStateList = EosSceneTargetList;

/**
 * @brief EosSceneInput 聚合 EOS 单周期场景实体输入。
 * @note 当前仅含 `targets`，是统一骨架 `CycleInput.scene` 的固定语义槽位，
 *       用于稳定外部输入边界；不是临时包装层。
 */
struct ONEQ_API EosSceneInput {
  EosSceneTargetList targets{}; /**< 当前周期场景目标输入列表 */
};

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SCENE_INPUT_H_
