/**
 * @file EsrSceneTypes.h
 * @brief 定义 ESR 单周期场景实体输入类型。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_SCENE_TYPES_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_SCENE_TYPES_H_

#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/coordinate/types.h"
#include "1q/foundation/pose_types.h"

namespace electronic_surveillance_radar {
namespace session {

/** @deprecated Internal legacy support pending executor deletion. */
enum class ONEQ_API EsrJammingTechnique { kUnknown = 0, kNoiseSuppression, kDeception, kMixed };

/** @deprecated Internal legacy support pending executor deletion. */
struct ONEQ_API EsrJammerSource {
  EsrJammingTechnique technique{EsrJammingTechnique::kUnknown};
  bool active{false};
  double center_hz{0.0};
  double bandwidth_hz{0.0};
  float power_w{0.0f};
  float deception_risk{0.0f};
  float confidence{1.0f};
};

using EsrJammerSourceList = std::vector<EsrJammerSource>;

/** @brief ESR 会话输入三维向量别名。 */
using EsrVector3f = oneq::foundation::Vector3f;

/** @brief ESR 会话输入欧拉角姿态别名（单位：deg）。 */
using EsrEulerAngleDeg = oneq::foundation::EulerAnglesDeg;

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
  std::uint64_t emitter_id{0U}; /**< 辐射源标识 */
  std::string emitter_name{};   /**< 可选辐射源名称，仅用于人读、trace 与调试视图，不参与关联 */
  bool has_ecef_kinematics{false}; /**< 是否提供工程 RF 链路所需的 ECEF 运动学。 */
  oneq::coordinate::EcefPositionM position_ecef_m{}; /**< 辐射源 ECEF 位置（m）。 */
  oneq::coordinate::EcefVelocityMps velocity_ecef_mps{}; /**< 辐射源 ECEF 速度（m/s）。 */
  oneq::foundation::PoseState pose{}; /**< 辐射源位置、速度与姿态状态 */
  double carrier_hz{0.0};             /**< 发射中心频率（单位：Hz） */
  double bandwidth_hz{0.0};           /**< 发射带宽（单位：Hz） */
  double tx_power_w{0.0};             /**< 发射功率（单位：W） */
  double pulse_width_s{0.0};          /**< 脉宽（单位：s） */
  double pri_s{0.0};                  /**< 脉冲重复间隔（单位：s） */
  EsrEmitterBeamState beam_state{};   /**< 当前波束状态 */
  bool is_emitting{true};             /**< 当前周期是否处于发射状态 */
};

/** @brief EsrSceneEmitterList 表示 ESR 场景辐射源输入列表。 */
using EsrSceneEmitterList = std::vector<EsrSceneEmitter>;

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_SCENE_TYPES_H_
