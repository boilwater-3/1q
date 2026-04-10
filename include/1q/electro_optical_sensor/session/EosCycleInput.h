/**
 * @file EosCycleInput.h
 * @brief 定义光学传感器组件单周期输入载荷。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CORE_CONTEXT_EOS_CYCLE_INPUT_H_
#define ELECTRO_OPTICAL_SENSOR_CORE_CONTEXT_EOS_CYCLE_INPUT_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/common/pose_types.h"

namespace electro_optical_sensor {
namespace session {

/**
 * @brief DayNightType 表示昼夜环境类型。
 */
enum class DayNightType {
  kDay = 0,  /**< 白天 */
  kNight,    /**< 夜间 */
  kTwilight  /**< 晨昏 */
};

/**
 * @brief EosTargetState 描述单目标输入特征。
 */
struct ONEQ_API EosTargetState {
  std::uint64_t target_id{0U};          /**< 目标标识 */
  float range_m{0.0f};                  /**< 斜距（单位：m） */
  float azimuth_deg{0.0f};              /**< 目标方位角（单位：deg） */
  float elevation_deg{0.0f};            /**< 目标仰角（单位：deg） */
  float apparent_temperature_k{290.0f}; /**< 目标等效温度（单位：K） */
  float emissivity{0.9f};               /**< 红外辐射效率，范围 [0, 1] */
  float reflectance{0.2f};              /**< 可见光反射率，范围 [0, 1] */
  float projected_area_m2{1.0f};        /**< 等效投影面积（单位：m^2） */
};

/** @brief EosTargetStateList 表示单周期目标列表。 */
using EosTargetStateList = std::vector<EosTargetState>;

/**
 * @brief EosCycleInput 描述光学传感器单周期输入。
 */
struct ONEQ_API EosCycleInput {
  std::uint32_t cycle_index{0U};            /**< 当前周期号 */
  float dt_sec{1.0f};                       /**< 当前周期步长（单位：s） */
  oneq::common::PoseState platform_pose{};  /**< 平台位姿状态 */
  float solar_altitude_deg{45.0f};          /**< 太阳高度角（单位：deg） */
  float solar_azimuth_deg{180.0f};          /**< 太阳方位角（单位：deg） */
  float solar_irradiance_w_m2{800.0f};      /**< 地表太阳辐照度（单位：W/m^2） */
  float atmospheric_transmittance{0.85f};   /**< 大气透明度，范围 [0, 1] */
  float cloud_coverage_ratio{0.2f};         /**< 云量，范围 [0, 1] */
  float ambient_wind_speed_mps{0.0f};       /**< 环境风速（单位：m/s，范围 [0, +inf)） */
  DayNightType day_night_type{DayNightType::kDay}; /**< 昼夜环境类型 */
  float background_temperature_k{290.0f};   /**< 背景温度（单位：K） */
  EosTargetStateList scene_targets{};       /**< 当前周期候选目标列表 */
};

}  // namespace session

namespace core {
namespace context {
using ::electro_optical_sensor::session::DayNightType;
using ::electro_optical_sensor::session::EosTargetState;
using ::electro_optical_sensor::session::EosTargetStateList;
using ::electro_optical_sensor::session::EosCycleInput;
}  // namespace context
}  // namespace core

}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CORE_CONTEXT_EOS_CYCLE_INPUT_H_
