/**
 * @file ArRadarFrameTransform.h
 * @brief AR 平台锚点 ENU → 雷达局部系变换（Session/校验内部使用）。
 *
 * 这不是场景目标 ECEF→ENU 适配。集成方应先用公共
 * `TryMakeEnuSceneState` 填好 `ArTargetInput`，再交给 Session；本头在 ENU
 * 之后按平台姿态∘安装角旋入雷达体系。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_RADAR_FRAME_TRANSFORM_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_RADAR_FRAME_TRANSFORM_H_

#include "1q/airborne_radar/session/ArPlatformInput.h"
#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "1q/api.hpp"
#include "1q/coordinate/types.h"
#include "1q/foundation/pose_types.h"

namespace airborne_radar {
namespace session {

/**
 * @brief 严格复合平台姿态与雷达安装角，得到雷达局部姿态角。
 * @param[in] platform_attitude_deg 平台姿态角（Body→ENU，deg）。
 * @param[in] mount_angles_deg 雷达安装角（Body→Radar，deg）。
 * @return 复合后的雷达姿态角（ENU→Radar，deg）。
 */
ONEQ_API oneq::coordinate::EulerAnglesDeg ComposeRadarAttitudeDeg(
    const oneq::coordinate::EulerAnglesDeg& platform_attitude_deg,
    const oneq::coordinate::EulerAnglesDeg& mount_angles_deg);

/**
 * @brief 雷达坐标变换执行状态。
 */
enum class ONEQ_API ArCoordinateStatus { kOk = 0, kNullOutput, kCoordinateTransformFail };

/**
 * @brief 将平台 ECEF 运动学转换为雷达局部参考系与速度。
 * @param[in] input 平台 ECEF 位姿。
 * @param[in] mount_angles_deg 雷达安装角（Body→Radar）。
 * @param[out] reference 雷达局部参考系（含复合后的 ENU→Radar 姿态）。
 * @param[out] radar_local_velocity_mps 雷达局部系下平台速度（m/s）。
 * @param[out] status 可选失败原因。
 * @return 成功返回 true。
 */
ONEQ_API bool TryMakeArPoseFromPlatform(const ArPlatformInput& input,
                                        const oneq::coordinate::EulerAnglesDeg& mount_angles_deg,
                                        oneq::coordinate::LocalFrameReference* reference,
                                        oneq::foundation::Vector3f* radar_local_velocity_mps,
                                        ArCoordinateStatus* status = nullptr);

/**
 * @brief 将平台锚点 ENU 场景目标旋入雷达局部系得到 ArSceneTarget。
 * @note ENU 契约见 docs/common/contract.md；速度扣除平台速度得到相对速度。
 */
ONEQ_API bool TryMakeArTargetFromEnu(const ArTargetInput& target_input,
                                     const oneq::coordinate::LocalFrameReference& reference,
                                     oneq::foundation::Vector3f radar_local_velocity_mps,
                                     ArSceneTarget* target, ArCoordinateStatus* status = nullptr);

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_RADAR_FRAME_TRANSFORM_H_
