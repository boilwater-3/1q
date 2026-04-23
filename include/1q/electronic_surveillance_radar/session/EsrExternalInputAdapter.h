/**
 * @file EsrExternalInputAdapter.h
 * @brief ESR 外部输入适配统一入口。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_EXTERNAL_INPUT_ADAPTER_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_EXTERNAL_INPUT_ADAPTER_H_

#include <string>

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/model/EsrOrientationConfig.h"
#include "1q/electronic_surveillance_radar/session/EsrSceneTypes.h"
#include "1q/foundation/coordinate_transform.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief ESR 局部坐标参考系定义。
 * @note origin_lla 定义 ENU 原点；frame_attitude_deg 定义 ESR 局部坐标相对 ENU 的姿态。
 */
struct ONEQ_API EsrCoordinateReference {
  oneq::foundation::LlaCoordinateDegM origin_lla{};      /**< 参考原点（WGS84 LLA） */
  oneq::foundation::EulerAnglesDeg frame_attitude_deg{}; /**< ESR 局部坐标相对 ENU 的姿态角 */
};

/**
 * @brief ESR 平台速度输入参考系类型。
 */
enum class ONEQ_API EsrVelocityFrame {
  kEsrLocal = 0, /**< ESR 局部坐标速度 */
  kEcef = 1,     /**< 地固 ECEF 速度 */
  kEnu = 2,      /**< 局部 ENU 速度（x=east, y=north, z=up） */
  kNed = 3       /**< 局部 NED 速度（x=north, y=east, z=down） */
};

/**
 * @brief ESR 外部平台运动学输入（统一入口）。
 */
struct ONEQ_API EsrExternalPoseInput {
  oneq::foundation::EcefCoordinateM platform_position_ecef_m{};          /**< 平台位置（ECEF，m） */
  model::EsrVector3f platform_velocity_mps{};                            /**< 平台速度（m/s） */
  EsrVelocityFrame platform_velocity_frame{EsrVelocityFrame::kEsrLocal}; /**< 速度参考系 */
  model::EsrEulerAngleDeg platform_attitude_deg{}; /**< 平台姿态角（ESR 局部系，deg） */
};

/**
 * @brief ESR 外部辐射源输入（统一入口）。
 */
struct ONEQ_API EsrExternalEmitterInput {
  std::string emitter_id{};                                  /**< 辐射源标识 */
  oneq::foundation::EcefCoordinateM emitter_position_ecef_m{}; /**< 辐射源位置（ECEF，m） */
  model::EsrVector3f emitter_velocity_mps{};                /**< 辐射源速度（m/s） */
  EsrVelocityFrame emitter_velocity_frame{EsrVelocityFrame::kEsrLocal}; /**< 速度参考系 */
  model::EsrEulerAngleDeg emitter_attitude_deg{};           /**< 辐射源姿态角（ESR 局部系，deg） */
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
                                                   const EsrCoordinateReference& reference,
                                                   model::EsrPoseState* pose,
                                                   EsrCoordinateStatus* status = nullptr);

ONEQ_API bool TryMakeEsrSceneEmitterFromExternalInput(
    const EsrExternalEmitterInput& input,
    const EsrCoordinateReference& reference,
    EsrSceneEmitter* emitter,
    EsrCoordinateStatus* status = nullptr);

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_EXTERNAL_INPUT_ADAPTER_H_
