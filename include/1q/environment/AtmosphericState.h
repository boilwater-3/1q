/**
 * @file AtmosphericState.h
 * @brief 定义统一大气状态查询结果（SI 单位制）。
 */

#ifndef ONEQ_ENVIRONMENT_ATMOSPHERIC_STATE_H_
#define ONEQ_ENVIRONMENT_ATMOSPHERIC_STATE_H_

#include "1q/api.hpp"

namespace oneq {
namespace environment {

/**
 * @brief 统一大气状态查询结果（SI 单位制）。
 */
struct ONEQ_API AtmosphericState {
  float altitude_m{0.0f};            /**< 查询高度（输入回显，单位：m） */
  float temperature_k{288.15f};      /**< 温度（单位：K） */
  float pressure_pa{101325.0f};      /**< 气压（单位：Pa） */
  float density_kg_m3{1.225f};       /**< 密度（单位：kg/m^3） */
  float speed_of_sound_mps{340.29f}; /**< 声速（单位：m/s） */
  float pressure_hpa{1013.25f};      /**< 气压（单位：hPa，= pressure_pa / 100） */
};

}  // namespace environment
}  // namespace oneq

#endif  // ONEQ_ENVIRONMENT_ATMOSPHERIC_STATE_H_
