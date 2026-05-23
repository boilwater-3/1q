/**
 * @file EsrExternalInputAdapter.h
 * @brief ESR 外部输入适配统一入口。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_EXTERNAL_INPUT_ADAPTER_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_EXTERNAL_INPUT_ADAPTER_H_

#include <string>

#include "1q/api.hpp"
#include "1q/coordinate/types.h"
#include "1q/electronic_surveillance_radar/session/EsrSceneTypes.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief ESR 局部坐标参考系定义。
 * @note origin_lla 定义 ENU 原点；frame_attitude_deg 定义 ESR 局部坐标相对 ENU 的姿态。
 */

/**
 * @brief ESR 外部平台运动学输入。
 * @note 速度固定为 ECEF 坐标系。
 */
struct ONEQ_API EsrExternalPoseInput {
  oneq::coordinate::EcefPositionM platform_position_ecef_m{};     /**< 平台位置（ECEF，m） */
  oneq::coordinate::EcefVelocityMps platform_velocity_mps{};      /**< 平台速度（ECEF，单位：m/s） */
  oneq::coordinate::EulerAnglesDeg platform_attitude_deg{};       /**< 平台姿态角（Body->ENU，deg） */
};

/**
 * @brief ESR 外部辐射源输入（统一入口）。
 */
struct ONEQ_API EsrExternalEmitterInput {
  std::uint64_t emitter_id{0U};                                  /**< 辐射源标识 */
  oneq::coordinate::ExternalKinematics kinematics{};            /**< 外部运动学输入 */
  double carrier_hz{0.0};                                   /**< 发射中心频率（Hz） */
  double bandwidth_hz{0.0};                                 /**< 发射带宽（Hz） */
  double tx_power_w{0.0};                                   /**< 发射功率（W） */
  double pulse_width_s{0.0};                                /**< 脉宽（s） */
  double pri_s{0.0};                                        /**< 脉冲重复间隔（s） */
  EsrEmitterBeamState beam_state{};                         /**< 发射波束状态 */
  bool is_emitting{true};                                   /**< 当前周期是否发射 */
};

/**
 * @brief ESR 坐标适配执行状态。
 */
enum class ONEQ_API EsrCoordinateStatus {
  kOk = 0,
  kNullOutput,
  kCoordinateTransformFail
};

ONEQ_API bool TryMakeEsrPoseFromExternalKinematics(const EsrExternalPoseInput& input,
                                                   const oneq::coordinate::LocalFrameReference& reference,
                                                   EsrPoseState* pose,
                                                   EsrCoordinateStatus* status = nullptr);

ONEQ_API bool TryMakeEsrSceneEmitterFromExternalInput(
    const EsrExternalEmitterInput& input,
    const oneq::coordinate::LocalFrameReference& reference,
    EsrSceneEmitter* emitter,
    EsrCoordinateStatus* status = nullptr);

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_EXTERNAL_INPUT_ADAPTER_H_
