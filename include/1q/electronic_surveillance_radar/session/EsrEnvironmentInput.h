/**
 * @file EsrEnvironmentInput.h
 * @brief 定义 ESR 单周期环境输入、快照与干扰源类型。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_ENVIRONMENT_INPUT_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_ENVIRONMENT_INPUT_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/config/EsrEnvironmentConfig.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief EsrJammingTechnique 表示干扰技术类型。
 */
enum class ONEQ_API EsrJammingTechnique {
  kUnknown = 0,      /**< 未知或未分类干扰 */
  kNoiseSuppression, /**< 压制式/噪声式干扰 */
  kDeception,        /**< 欺骗式干扰 */
  kMixed             /**< 压制与欺骗并存 */
};

/**
 * @brief EsrJammerSource 描述场景中的单个干扰源输入。
 */
struct ONEQ_API EsrJammerSource {
  EsrJammingTechnique technique{EsrJammingTechnique::kUnknown};
  bool active{false};
  double center_hz{0.0};
  double bandwidth_hz{0.0};
  float power_w{0.0f};
  float deception_risk{0.0f};
  float confidence{1.0f};
};

/** @brief EsrJammerSourceList 表示干扰源列表。 */
using EsrJammerSourceList = std::vector<EsrJammerSource>;

/**
 * @brief EsrPropagationEnvironmentProfile 描述高层传播环境类型。
 */
enum class ONEQ_API EsrPropagationEnvironmentProfile {
  kOpen = 0,
  kTypical = 1,
  kComplex = 2
};

/**
 * @brief EsrClutterDensityLevel 描述高层杂波密度级别。
 */
enum class ONEQ_API EsrClutterDensityLevel {
  kLow = 0,
  kMedium = 1,
  kHigh = 2
};

/**
 * @brief EsrAtmosphericObservation 描述外部可观测天气事实。
 */
struct ONEQ_API EsrAtmosphericObservation {
  float relative_humidity_ratio{0.5f};
  float precipitation_rate_mmph{0.0f};
  float visibility_km{20.0f};
};

/**
 * @brief EsrEnvironmentInput 描述待冻结环境高层观测输入。
 */
struct ONEQ_API EsrEnvironmentInput {
  EsrPropagationEnvironmentProfile propagation_profile{
      EsrPropagationEnvironmentProfile::kTypical};
  EsrClutterDensityLevel clutter_density{EsrClutterDensityLevel::kMedium};
  float spectrum_occupancy_ratio{0.0f};
  EsrAtmosphericObservation atmospheric_observation{};
  EsrJammerSourceList jammer_sources{};
};

/**
 * @brief EsrEnvironmentCycleContext 描述单周期冻结上下文。
 */
struct ONEQ_API EsrEnvironmentCycleContext {
  std::uint32_t cycle_index{0U};
  float dt_sec{0.0f};
  EsrEnvironmentInput observation{};
};

/**
 * @brief EsrEnvironmentSnapshot 描述单周期环境快照。
 */
struct ONEQ_API EsrEnvironmentSnapshot {
  std::uint32_t cycle_index{0U};
  float dt_sec{0.0f};
  float propagation_loss_db{0.0f};
  float clutter_noise_w{0.0f};
  float suppression_power_w{0.0f};
  float deception_risk{0.0f};
  float spectrum_occupancy_ratio{0.0f};
  bool jamming_detected{false};
  EsrJammerSourceList jammer_sources{};
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_ENVIRONMENT_INPUT_H_
