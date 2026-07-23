/**
 * @file EsrEnvironmentInput.h
 * @brief 定义 ESR 单周期环境输入、快照与干扰源类型。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_ENVIRONMENT_INPUT_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_ENVIRONMENT_INPUT_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/electromagnetics/RfLinkBudget.h"
#include "1q/electronic_surveillance_radar/config/EsrEnvironmentConfig.h"
#include "1q/electronic_surveillance_radar/session/EsrSceneTypes.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief EsrPropagationEnvironmentProfile 描述高层传播环境类型。
 */
enum class ONEQ_API EsrPropagationEnvironmentProfile { kOpen = 0, kTypical = 1, kComplex = 2 };

/**
 * @brief EsrClutterDensityLevel 描述高层杂波密度级别。
 */
enum class ONEQ_API EsrClutterDensityLevel { kLow = 0, kMedium = 1, kHigh = 2 };

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
  EsrPropagationEnvironmentProfile propagation_profile{EsrPropagationEnvironmentProfile::kTypical};
  EsrClutterDensityLevel clutter_density{EsrClutterDensityLevel::kMedium};
  float spectrum_occupancy_ratio{0.0f}; /**< 频谱占用率 [0,1]；按 1+9ρ 放大接收机与杂波底噪 */
  EsrAtmosphericObservation atmospheric_observation{};
};

/**
 * @brief EsrEnvironmentCycleContext 描述单周期冻结上下文。
 */
struct ONEQ_API EsrEnvironmentCycleContext {
  std::uint32_t cycle_index{0U};
  float dt_sec{0.0f};
  float platform_altitude_m{0.0f}; /**< 接收平台 WGS84 绝对海拔（单位：m） */
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
  float spectrum_occupancy_ratio{0.0f}; /**< 冻结占用率；检测链按 1+9ρ 计算环境噪声倍率 */
  /** @deprecated Dead legacy-executor state pending source deletion. */
  oneq::electromagnetics::RfInterferenceMode interference_mode{
      oneq::electromagnetics::RfInterferenceMode::kNone};
  /** @deprecated Dead legacy-executor state pending source deletion. */
  EsrJammerSourceList jammer_sources{};
  /** @deprecated Dead legacy-executor state pending source deletion. */
  std::vector<oneq::electromagnetics::RfEmission> engineering_emissions{};
};

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
  bool has_clutter_density{false};                          /**< 是否更新杂波密度 */
  session::EsrClutterDensityLevel clutter_density{
      session::EsrClutterDensityLevel::kMedium};                /**< 新杂波密度 */
  bool has_spectrum_occupancy_ratio{false};                     /**< 是否更新频谱占用率 */
  float spectrum_occupancy_ratio{0.0f};                         /**< 新频谱占用率，范围 [0, 1] */
  bool has_atmospheric_observation{false};                      /**< 是否更新天气观测 */
  session::EsrAtmosphericObservation atmospheric_observation{}; /**< 新天气观测 */
};

/**
 * @brief EsrEnvironmentInputState 维护调用方侧当前环境事实状态。
 */
class ONEQ_API EsrEnvironmentInputState {
 public:
  EsrEnvironmentInputState() = default;
  explicit EsrEnvironmentInputState(const EsrEnvironmentInput& snapshot) : snapshot_(snapshot) {}

  EsrEnvironmentInputState& Reset(const EsrEnvironmentInput& snapshot) {
    snapshot_ = snapshot;
    return *this;
  }

  EsrEnvironmentInputState& Update(const EsrEnvironmentInputPatch& patch) {
    if (patch.has_propagation_profile) {
      snapshot_.propagation_profile = patch.propagation_profile;
    }
    if (patch.has_clutter_density) {
      snapshot_.clutter_density = patch.clutter_density;
    }
    if (patch.has_spectrum_occupancy_ratio) {
      snapshot_.spectrum_occupancy_ratio = patch.spectrum_occupancy_ratio;
    }
    if (patch.has_atmospheric_observation) {
      snapshot_.atmospheric_observation = patch.atmospheric_observation;
    }
    return *this;
  }

  EsrEnvironmentInput Snapshot() const { return snapshot_; }

 private:
  EsrEnvironmentInput snapshot_{};
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_ENVIRONMENT_INPUT_H_
