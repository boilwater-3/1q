/**
 * @file EsrSceneInput.h
 * @brief 定义 ESR 单周期场景输入聚合类型。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_SCENE_INPUT_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_SCENE_INPUT_H_

#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/model/EsrOrientationConfig.h"

namespace electronic_surveillance_radar {
namespace session {

/** @brief EsrEmitterBeamState 描述 ESR 场景辐射源波束参数。 */
struct ONEQ_API EsrEmitterBeamState {
  double center_az_deg{0.0};     /**< 波束中心方位（单位：deg） */
  double center_el_deg{0.0};     /**< 波束中心俯仰（单位：deg） */
  double az_beamwidth_deg{20.0}; /**< 方位波束宽度（单位：deg） */
  double el_beamwidth_deg{20.0}; /**< 俯仰波束宽度（单位：deg） */
  bool beam_state_valid{false};  /**< 波束参数是否已显式配置 */
};

/** @brief EsrSceneEmitter 描述 ESR 场景辐射源输入。 */
struct ONEQ_API EsrSceneEmitter {
  std::string emitter_id{};          /**< 辐射源标识 */
  model::EsrPoseState pose{};        /**< 辐射源位置、速度与姿态状态 */
  double carrier_hz{0.0};            /**< 发射中心频率（单位：Hz） */
  double bandwidth_hz{0.0};          /**< 发射带宽（单位：Hz） */
  double tx_power_w{0.0};            /**< 发射功率（单位：W） */
  double pulse_width_s{0.0};         /**< 脉宽（单位：s） */
  double pri_s{0.0};                 /**< 脉冲重复间隔（单位：s） */
  EsrEmitterBeamState beam_state{};  /**< 当前波束状态 */
  bool is_emitting{true};            /**< 当前周期是否处于发射状态 */
};

/** @brief EsrSceneEmitterList 表示 ESR 场景辐射源输入列表。 */
using EsrSceneEmitterList = std::vector<EsrSceneEmitter>;

/**
 * @brief EsrSceneInput 聚合 ESR 单周期场景实体输入。
 */
struct ONEQ_API EsrSceneInput {
  EsrSceneEmitterList emitters{}; /**< 当前周期场景辐射源输入列表 */
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_SCENE_INPUT_H_
