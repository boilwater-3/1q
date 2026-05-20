/**
 * @file RadarExternalInputAdapter.h
 * @brief 机载雷达外部输入适配统一入口。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_RADAR_EXTERNAL_INPUT_ADAPTER_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_RADAR_EXTERNAL_INPUT_ADAPTER_H_

#include <cstdint>

#include "1q/airborne_radar/model/RadarOrientationConfig.h"
#include "1q/airborne_radar/session/RadarSceneTypes.h"
#include "1q/api.hpp"
#include "1q/coordinate/types.h"
#include "1q/foundation/pose_types.h"

namespace airborne_radar {
namespace session {

/**
 * @brief 描述外部坐标转换为雷达局部坐标所需的参考系信息。
 * @note `origin_lla` 定义局部 ENU 原点，`radar_attitude_deg` 定义雷达局部坐标相对 ENU 的姿态。
 */
struct ONEQ_API RadarLocalFrameReference {
  oneq::coordinate::LlaPositionDegM origin_lla{};        /**< 雷达参考原点（WGS84 LLA） */
  oneq::coordinate::EulerAnglesDeg radar_attitude_deg{}; /**< 雷达局部坐标相对 ENU 的姿态角 */
};

/**
 * @brief 外部平台运动学输入。
 * @note 速度固定为 ECEF 坐标系，姿态角采用 Body->ENU 约定。
 */
struct ONEQ_API RadarExternalPoseInput {
  oneq::coordinate::EcefPositionM platform_position_ecef_m{}; /**< 平台位置（ECEF，m） */
  oneq::coordinate::EcefVelocityMps platform_velocity_mps{};  /**< 平台速度（ECEF，单位：m/s） */
  oneq::coordinate::EulerAnglesDeg platform_attitude_deg{};   /**< 平台姿态角（Body->ENU，deg） */
  oneq::coordinate::EulerAnglesDeg radar_mount_angles_deg{};  /**< 雷达安装角（Body->Radar，deg） */
};

/**
 * @brief 外部目标运动学输入（纯目标字段）。
 * @note 速度固定为 ECEF 坐标系，适配器内自动扣除平台速度得到相对速度。
 */
struct ONEQ_API TargetExternalKinematics {
  std::uint64_t external_target_id{0};                      /**< 外部目标标识符。0 视为未设置，将触发 kUnknownExternalTargetId 警告 */
  oneq::coordinate::EcefPositionM target_position_ecef_m{}; /**< 目标位置（ECEF，m） */
  oneq::coordinate::EcefVelocityMps target_velocity_mps{};  /**< 目标速度（ECEF，单位：m/s） */
  float rcs{1.0f};                                          /**< 目标 RCS（m^2） */
  int swerling_type{0};                                     /**< 目标起伏模型 */
};

/**
 * @brief 严格复合平台姿态与雷达安装角，得到雷达局部姿态角。
 * @param[in] platform_attitude_deg 平台姿态角（ENU->Body，单位：deg）。
 * @param[in] radar_mount_angles_deg 雷达安装角（Body->Radar，单位：deg）。
 * @return 复合后的雷达姿态角（ENU->Radar，单位：deg）。
 * @note 该接口采用旋转矩阵复合，不使用欧拉角分量直接相加。
 */
ONEQ_API oneq::coordinate::EulerAnglesDeg ComposeRadarAttitudeDeg(
    const oneq::coordinate::EulerAnglesDeg& platform_attitude_deg,
    const oneq::coordinate::EulerAnglesDeg& radar_mount_angles_deg);

/**
 * @brief 雷达坐标适配执行状态。
 */
enum class ONEQ_API RadarCoordinateStatus {
  kOk = 0,
  kNullOutput,
  kCoordinateTransformFail
};

/**
 * @brief 两步模式——第一步：将外部平台运动学转换为雷达局部位姿。
 * @param[in] input 外部平台运动学输入。
 * @param[out] reference 输出雷达局部参考系信息，用于后续目标转换。
 * @param[out] platform_pose 输出雷达局部平台位姿。
 * @param[out] status 可选输出状态，nullptr 表示不关心失败原因。
 * @return 成功返回 true，输入非法或输出为空返回 false。
 * @note AR 场景输入使用“雷达自身为局部原点”的坐标约定，因此
 *       `platform_pose.position_m` 固定为 `(0,0,0)`；`input.platform_position_ecef_m`
 *       会写入 `reference.origin_lla`，用于把目标位置转换为相对雷达的局部坐标。
 */
ONEQ_API bool TryMakeRadarPoseFromExternalKinematics(const RadarExternalPoseInput& input,
                                                     RadarLocalFrameReference* reference,
                                                     oneq::foundation::PoseState* platform_pose,
                                                     RadarCoordinateStatus* status = nullptr);

/**
 * @brief 两步模式——第二步：使用预计算的参考系将外部目标转换为 RadarSceneTarget。
 * @param[in] external_target_id 外部目标标识符。
 * @param[in] target_input 目标运动学输入（纯目标字段）。
 * @param[in] reference 第一步产出的雷达局部参考系。
 * @param[in] radar_local_velocity_mps 雷达局部速度（用于计算相对速度）。
 * @param[out] target 输出场景目标输入，可为 nullptr。
 * @param[out] status 可选输出状态，nullptr 表示不关心失败原因。
 * @return 成功返回 true，输入非法或输出为空返回 false。
 *
 * @note 当 `radar_local_velocity_mps` 非零且目标速度参考系非 kRadarLocal 时，
 *       在雷达局部坐标系内扣除雷达自身速度：`v_rel = v_target_local - v_radar_local`。
 */
ONEQ_API bool TryMakeTargetFromExternalKinematics(
    std::uint64_t external_target_id, const TargetExternalKinematics& target_input,
    const RadarLocalFrameReference& reference, oneq::foundation::Vector3f radar_local_velocity_mps,
    RadarSceneTarget* target, RadarCoordinateStatus* status = nullptr);

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_RADAR_EXTERNAL_INPUT_ADAPTER_H_
