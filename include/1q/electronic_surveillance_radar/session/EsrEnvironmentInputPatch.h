/**
 * @file EsrEnvironmentInputPatch.h
 * @brief 定义 ESR 单周期环境输入状态的局部更新载荷。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_ENVIRONMENT_INPUT_PATCH_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_ENVIRONMENT_INPUT_PATCH_H_

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/session/EsrEnvironmentInput.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief EsrEnvironmentInputPatch 表示调用方侧环境事实状态的局部更新。
 *
 * @note 本类型不直接进入 EsrSession::StepWithResult()。调用方应先用
 *       EsrEnvironmentInputState 合成完整 EsrEnvironmentInput 快照，再写入
 *       EsrCycleInput::environment。
 */
struct ONEQ_API EsrEnvironmentInputPatch {
  bool has_propagation_profile{false}; /**< 是否更新传播环境类型 */
  session::EsrPropagationEnvironmentProfile propagation_profile{
      session::EsrPropagationEnvironmentProfile::kTypical}; /**< 新传播环境类型 */
  bool has_clutter_density{false};                              /**< 是否更新杂波密度 */
  session::EsrClutterDensityLevel clutter_density{
      session::EsrClutterDensityLevel::kMedium}; /**< 新杂波密度 */
  bool has_spectrum_occupancy_ratio{false};          /**< 是否更新频谱占用率 */
  float spectrum_occupancy_ratio{0.0f};              /**< 新频谱占用率，范围 [0, 1] */
  bool has_atmospheric_observation{false};           /**< 是否更新天气观测 */
  session::EsrAtmosphericObservation atmospheric_observation{}; /**< 新天气观测 */
  bool has_jammer_sources{false};                                   /**< 是否更新干扰源列表 */
  session::EsrJammerSourceList jammer_sources{};                /**< 新干扰源列表 */
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_ENVIRONMENT_INPUT_PATCH_H_
