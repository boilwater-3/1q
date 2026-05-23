/**
 * @file EosSceneTypes.h
 * @brief 定义 EOS 单周期场景实体输入类型。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SCENE_TYPES_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SCENE_TYPES_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"

namespace electro_optical_sensor {
namespace session {

/**
 * @brief EosTargetAppearance 描述目标辐射与外观参数。
 */
struct ONEQ_API EosTargetAppearance {
  float apparent_temperature_k{290.0f}; /**< 目标等效温度（单位：K） */
  float emissivity{0.9f};               /**< 红外辐射效率，范围 [0, 1] */
  float reflectance{0.2f};              /**< 可见光反射率，范围 [0, 1] */
  float projected_area_m2{1.0f};        /**< 等效投影面积（单位：m^2） */
};

/**
 * @brief EosSceneTarget 描述单目标输入特征。
 */
struct ONEQ_API EosSceneTarget {
  std::uint64_t target_id{0U};          /**< 目标标识 */
  float range_m{0.0f};                  /**< 斜距（单位：m） */
  float azimuth_deg{0.0f};              /**< 目标方位角（单位：deg） */
  float elevation_deg{0.0f};            /**< 目标仰角（单位：deg） */
  EosTargetAppearance appearance{};     /**< 目标辐射与外观参数 */
};

/** @brief EosSceneTargetList 表示 EOS 场景目标输入列表。 */
using EosSceneTargetList = std::vector<EosSceneTarget>;

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SCENE_TYPES_H_
