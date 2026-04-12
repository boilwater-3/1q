/**
 * @file EmitterTruthState.h
 * @brief 定义电子侦察输入侧使用的辐射源真值状态类型。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_MODEL_EMITTER_TRUTH_STATE_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_MODEL_EMITTER_TRUTH_STATE_H_

#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/model/EsrOrientationConfig.h"

namespace electronic_surveillance_radar {
namespace model {

/**
 * @brief EmitterBeamState 描述发射源当前波束状态。
 */
struct ONEQ_API EmitterBeamState {
  float center_az_deg{0.0f};     /**< 波束中心方位（单位：deg） */
  float center_el_deg{0.0f};     /**< 波束中心俯仰（单位：deg） */
  float az_beamwidth_deg{20.0f}; /**< 方位波束宽度（单位：deg） */
  float el_beamwidth_deg{20.0f}; /**< 俯仰波束宽度（单位：deg） */
  bool beam_state_valid{false};  /**< 波束参数是否已显式配置，`false` 表示历史默认值 */
};

/**
 * @brief EmitterTruthState 描述场景中的单个辐射源真值输入。
 * @note 该结构仅用于生成接收机观测，不应直接作为侦察输出对外发布。
 */
struct ONEQ_API EmitterTruthState {
  std::string emitter_id{};      /**< 场景真值中的辐射源标识 */
  EsrPoseState pose{};           /**< 辐射源位置、速度与姿态状态 */
  double carrier_hz{0.0};        /**< 发射中心频率（单位：Hz） */
  double bandwidth_hz{0.0};      /**< 发射带宽（单位：Hz） */
  double tx_power_w{0.0};        /**< 发射功率（单位：W） */
  double pulse_width_s{0.0};     /**< 脉宽（单位：s） */
  double pri_s{0.0};             /**< 脉冲重复间隔（单位：s） */
  EmitterBeamState beam_state{}; /**< 当前波束状态 */
  bool is_emitting{true};        /**< 当前周期是否处于发射状态 */
};

/** @brief EmitterTruthStateList 表示场景辐射源真值列表。 */
using EmitterTruthStateList = std::vector<EmitterTruthState>;

}  // namespace model
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_MODEL_EMITTER_TRUTH_STATE_H_
