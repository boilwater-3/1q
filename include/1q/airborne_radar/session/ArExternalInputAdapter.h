/**
 * @file ArExternalInputAdapter.h
 * @brief 机载雷达外部输入适配类型集合。
 *
 * 外部坐标系输入适配（平台位姿、目标转换）的主头文件。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_AR_EXTERNAL_INPUT_ADAPTER_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_AR_EXTERNAL_INPUT_ADAPTER_H_

#include <cstdint>

#include "1q/airborne_radar/config/ArOrientationConfig.h"
#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "1q/api.hpp"
#include "1q/coordinate/types.h"
#include "1q/foundation/pose_types.h"

namespace airborne_radar {
namespace session {

/**
 * @brief 外部平台运动学输入。
 * @note 速度固定为 ECEF 坐标系，姿态角采用 Body->ENU 约定。
 * @note 雷达安装角通过 ArOrientationConfig::mount_angles_deg 配置，不在此结构中。
 *       EOS/ESR 外部 Pose 结构同样不含安装角，因为对应适配器按传感器视轴与机体系对齐处理。
 */
struct ONEQ_API ArExternalPoseInput {
  std::uint64_t platform_entity_id{0}; /**< 平台实体标识；用于同平台 RF 耦合路径判定 */
  oneq::coordinate::EcefPositionM platform_position_ecef_m{}; /**< 平台位置（ECEF，m） */
  oneq::coordinate::EcefVelocityMps platform_velocity_mps{};  /**< 平台速度（ECEF，单位：m/s） */
  oneq::coordinate::EulerAnglesDeg platform_attitude_deg{};   /**< 平台姿态角（Body->ENU，deg） */
};

/**
 * @brief 严格复合平台姿态与雷达安装角，得到雷达局部姿态角。
 * @param[in] platform_attitude_deg 平台姿态角（Body->ENU，单位：deg）。
 * @param[in] mount_angles_deg 雷达安装角（Body->Radar，单位：deg）。
 * @return 复合后的雷达姿态角（ENU->Radar，单位：deg）。
 * @note 该接口采用旋转矩阵复合，不使用欧拉角分量直接相加。
 */
ONEQ_API oneq::coordinate::EulerAnglesDeg ComposeRadarAttitudeDeg(
    const oneq::coordinate::EulerAnglesDeg& platform_attitude_deg,
    const oneq::coordinate::EulerAnglesDeg& mount_angles_deg);

/**
 * @brief 雷达坐标适配执行状态。
 * @note 当前 `kCoordinateTransformFail` 统一覆盖了 "输入 NaN/Inf" 和 "坐标变换数值失败"
 *       两种不同的失败原因。未来可考虑拆分为更细粒度的枚举值（如
 *       kInvalidInput、kTransformFailed）。
 */
enum class ONEQ_API ArCoordinateStatus { kOk = 0, kNullOutput, kCoordinateTransformFail };

/**
 * @brief 两步模式——第一步：将外部平台运动学转换为雷达局部参考系与速度。
 * @param[in] input 外部平台运动学输入。
 * @param[in] mount_angles_deg 雷达安装角（Body->Radar，来自 ArOrientationConfig::mount_angles_deg）。
 * @param[out] reference 输出雷达局部参考系信息，用于后续目标转换。
 *             其中 `reference->frame_attitude_deg` 为复合后的 ENU->Radar 姿态角。
 * @param[out] radar_local_velocity_mps 输出雷达局部坐标系下的平台速度（m/s）。
 * @param[out] status 可选输出状态，nullptr 表示不关心失败原因。
 * @return 成功返回 true，输入非法或输出为空返回 false。
 * @note AR 以传感器自身为局部原点，平台位置无需输出。
 *       `input.platform_position_ecef_m` 写入 `reference->origin_lla`，
 *       用于把平台 ECEF 速度旋转到锚点 ENU 轴。
 */
ONEQ_API bool TryMakeArPoseFromExternalKinematics(const ArExternalPoseInput& input,
                                                  const oneq::coordinate::EulerAnglesDeg& mount_angles_deg,
                                                  oneq::coordinate::LocalFrameReference* reference,
                                                  oneq::foundation::Vector3f* radar_local_velocity_mps,
                                                  ArCoordinateStatus* status = nullptr);

/**
 * @brief 两步模式——第二步：将平台锚点 ENU 场景目标旋入雷达局部系得到 ArSceneTarget。
 * @param[in] target_input 平台锚点 ENU 场景目标输入（`target_id` 字段用作输出的 external_target_id）。
 * @param[in] reference 第一步产出的雷达局部参考系（仅消费 `frame_attitude_deg`，ENU→Radar 旋转）。
 * @param[in] radar_local_velocity_mps 雷达局部速度（用于计算相对速度）。
 * @param[out] target 输出场景目标输入，可为 nullptr。
 * @param[out] status 可选输出状态，nullptr 表示不关心失败原因。
 * @return 成功返回 true，输入非有限、非法或输出为空返回 false。
 * @note ENU 契约见 docs/common/contract.md「场景目标平台锚点 ENU 输入契约」；
 *       位置/速度先按 `frame_attitude_deg` 旋入雷达体系，速度再扣除平台速度得到相对速度。
 */
ONEQ_API bool TryMakeArTargetFromEnu(const ArTargetInput& target_input,
                                     const oneq::coordinate::LocalFrameReference& reference,
                                     oneq::foundation::Vector3f radar_local_velocity_mps,
                                     ArSceneTarget* target, ArCoordinateStatus* status = nullptr);

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_AR_EXTERNAL_INPUT_ADAPTER_H_
