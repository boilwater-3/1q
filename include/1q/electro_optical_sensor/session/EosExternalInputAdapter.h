/**
 * @file EosExternalInputAdapter.h
 * @brief EOS 外部输入适配统一入口。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_EXTERNAL_INPUT_ADAPTER_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_EXTERNAL_INPUT_ADAPTER_H_

#include <cstdint>

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/model/EosCycleInput.h"
#include "1q/foundation/coordinate_transform.h"

namespace electro_optical_sensor {
namespace session {

/**
 * @brief EOS 局部坐标参考系定义。
 * @note origin_lla 定义 ENU 原点；frame_attitude_deg 定义 EOS 局部坐标相对 ENU 的姿态。
 */
struct ONEQ_API EosCoordinateReference {
  oneq::foundation::LlaCoordinateDegM origin_lla{};      /**< 参考原点（WGS84 LLA） */
  oneq::foundation::EulerAnglesDeg frame_attitude_deg{}; /**< EOS 局部坐标相对 ENU 的姿态角 */
};

/**
 * @brief EOS 目标辐射/外观参数集合。
 */
struct ONEQ_API EosTargetAppearance {
  float apparent_temperature_k{290.0f}; /**< 目标等效温度（单位：K） */
  float emissivity{0.9f};               /**< 红外辐射效率，范围 [0, 1] */
  float reflectance{0.2f};              /**< 可见光反射率，范围 [0, 1] */
  float projected_area_m2{1.0f};        /**< 等效投影面积（单位：m^2） */
};

/**
 * @brief EOS 平台速度输入参考系类型。
 */
enum class ONEQ_API EosVelocityFrame {
  kEosLocal = 0, /**< EOS 局部坐标速度 */
  kEcef = 1,     /**< 地固 ECEF 速度 */
  kEnu = 2,      /**< 局部 ENU 速度（x=east, y=north, z=up） */
  kNed = 3       /**< 局部 NED 速度（x=north, y=east, z=down） */
};

/**
 * @brief EOS 外部平台运动学输入（统一入口）。
 */
struct ONEQ_API EosExternalPoseInput {
  oneq::foundation::EcefCoordinateM platform_position_ecef_m{};          /**< 平台位置（ECEF，m） */
  oneq::foundation::Vector3f platform_velocity_mps{};                    /**< 平台速度（m/s） */
  EosVelocityFrame platform_velocity_frame{EosVelocityFrame::kEosLocal}; /**< 速度参考系 */
  oneq::foundation::EulerAnglesDeg platform_attitude_deg{}; /**< 平台姿态角（EOS 局部系，deg） */
};

/**
 * @brief EOS 坐标适配执行状态。
 */
enum class ONEQ_API EosCoordinateStatus {
  kOk = 0,
  kNullOutput,
  kCoordinateTransformFail,
  kDegenerateGeometry
};

ONEQ_API bool TryMakeEosPoseFromExternalKinematics(const EosExternalPoseInput& input,
                                                   const EosCoordinateReference& reference,
                                                   oneq::foundation::PoseState* pose,
                                                   EosCoordinateStatus* status = nullptr);

ONEQ_API bool TryMakeEosTargetFromEcef(std::uint64_t target_id,
                                       const oneq::foundation::EcefCoordinateM& target_ecef_m,
                                       const EosCoordinateReference& reference,
                                       const oneq::foundation::PoseState& platform_pose,
                                       const EosTargetAppearance& appearance,
                                       EosTargetState* target,
                                       EosCoordinateStatus* status = nullptr);

ONEQ_API bool TryMakeEosTargetFromLla(std::uint64_t target_id,
                                      const oneq::foundation::LlaCoordinateDegM& target_lla_deg_m,
                                      const EosCoordinateReference& reference,
                                      const oneq::foundation::PoseState& platform_pose,
                                      const EosTargetAppearance& appearance, EosTargetState* target,
                                      EosCoordinateStatus* status = nullptr);

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_EXTERNAL_INPUT_ADAPTER_H_
