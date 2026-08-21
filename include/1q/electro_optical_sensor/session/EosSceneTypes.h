/**
 * @file EosSceneTypes.h
 * @brief 定义 EOS 单周期场景实体输入类型。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SCENE_TYPES_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SCENE_TYPES_H_

#include <cstdint>
#include <string>
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
 * @brief EosSceneTarget 描述单目标输入特征（平台锚点 radar-local ENU）。
 * @note ENU 契约见 docs/common/contract.md「场景目标平台锚点 ENU 输入契约」：
 *       原点 = 当周期平台 ECEF 位置（逐周期重锚），轴 = 锚点 ENU（x=东/y=北/z=天）；
 *       速度 = 目标 ECEF 速度旋入锚点 ENU 轴。集成层以
 *       `oneq::coordinate::TryEcefToLla`（锚点）+ `TryMakeEnuSceneState`（逐目标）
 *       完成转换后直填本结构。
 * @note 库内由 ENU 位置 + `EosCycleInput::platform_attitude_deg`（Body->ENU）派生
 *       体系球坐标（斜距/方位/仰角）供探测链使用；速度字段当前仅校验有限性，
 *       不参与探测计算（保留 ENU 契约统一形状）。
 */
struct ONEQ_API EosSceneTarget {
  std::uint64_t target_id{0U};      /**< 目标标识 */
  std::string target_name{};        /**< 可选目标名称，仅用于人读、trace 与调试视图，不参与关联 */
  float position_x{0.0f};           /**< 平台锚点 ENU 位置 x（东向，单位：m） */
  float position_y{0.0f};           /**< 平台锚点 ENU 位置 y（北向，单位：m） */
  float position_z{0.0f};           /**< 平台锚点 ENU 位置 z（天向，单位：m） */
  float velocity_x{0.0f};           /**< 锚点 ENU 速度 x 分量（单位：m/s） */
  float velocity_y{0.0f};           /**< 锚点 ENU 速度 y 分量（单位：m/s） */
  float velocity_z{0.0f};           /**< 锚点 ENU 速度 z 分量（单位：m/s） */
  EosTargetAppearance appearance{}; /**< 目标辐射与外观参数 */
};

/** @brief EosSceneTargetList 表示 EOS 场景目标输入列表。 */
using EosSceneTargetList = std::vector<EosSceneTarget>;

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_SCENE_TYPES_H_
