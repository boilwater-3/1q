/**
 * @file EosExternalInputAdapter.h
 * @brief EOS 外部输入适配统一入口。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_EXTERNAL_INPUT_ADAPTER_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_EXTERNAL_INPUT_ADAPTER_H_

#include <cstdint>
#include <string>

#include "1q/api.hpp"
#include "1q/coordinate/types.h"
#include "1q/electro_optical_sensor/session/EosSceneTypes.h"
#include "1q/foundation/pose_types.h"

namespace electro_optical_sensor {
namespace session {

/**
 * @brief EOS 局部坐标参考系定义。
 * @note origin_lla 定义 ENU 原点；frame_attitude_deg 定义 EOS 局部坐标相对 ENU 的姿态。
 */

/**
 * @brief EOS 外部平台运动学输入。
 * @note 速度固定为 ECEF 坐标系；EOS 没有独立 mount 字段，示例适配器按传感器视轴与
 *       机体系对齐处理。AR 的 Pose 结构多出 radar_mount_angles_deg 是雷达安装角需求。
 */
struct ONEQ_API EosExternalPoseInput {
  oneq::coordinate::EcefPositionM platform_position_ecef_m{}; /**< 平台位置（ECEF，m） */
  oneq::coordinate::EcefVelocityMps platform_velocity_mps{};  /**< 平台速度（ECEF，单位：m/s） */
  oneq::coordinate::EulerAnglesDeg platform_attitude_deg{};   /**< 平台姿态角（Body->ENU，deg） */
};

/**
 * @brief EOS 外部目标输入（统一入口）。
 * @note `kinematics.velocity_mps` 始终为 ECEF 速度；LLA 位置输入不改变速度坐标系。
 */
struct ONEQ_API EosExternalTargetInput {
  std::uint64_t target_id{0U}; /**< 目标标识 */
  std::string target_name{};   /**< 可选目标名称，仅用于人读、trace 与调试视图 */
  oneq::coordinate::ExternalKinematics kinematics{}; /**< 外部运动学输入 */
  EosTargetAppearance appearance{};                  /**< 目标辐射与外观参数 */
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

/**
 * @brief 将外部平台运动学输入转换为 EOS 平台位姿。
 * @param input 外部平台输入，位置与速度固定为 ECEF 坐标系。
 * @param reference EOS 局部坐标参考系，决定 ECEF/ENU 到 EOS 局部坐标的转换基准。
 * @param pose 输出平台位姿。
 * @param status 可选输出状态，nullptr 表示不关心失败原因。
 * @return 转换成功返回 true；输入非法、坐标变换失败或输出为空返回 false。
 */
ONEQ_API bool TryMakeEosPoseFromExternalKinematics(
    const EosExternalPoseInput& input, const oneq::coordinate::LocalFrameReference& reference,
    oneq::foundation::PoseState* pose, EosCoordinateStatus* status = nullptr);

/**
 * @brief 两步模式第二步：将外部目标输入转换为 EOS 场景目标状态。
 * @param target_id 目标标识。
 * @param input 外部目标输入。
 * @param reference EOS 局部坐标参考系。
 * @param platform_pose 平台位姿，用于计算目标相对平台的几何关系。
 * @param target 输出目标状态。
 * @param status 可选输出状态，nullptr 表示不关心失败原因。
 * @return 转换成功返回 true；输入非法、坐标变换失败或输出为空返回 false。
 */
ONEQ_API bool TryMakeEosSceneTargetFromExternalInput(
    std::uint64_t target_id, const EosExternalTargetInput& input,
    const oneq::coordinate::LocalFrameReference& reference,
    const oneq::foundation::PoseState& platform_pose, EosSceneTarget* target,
    EosCoordinateStatus* status = nullptr);

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_EXTERNAL_INPUT_ADAPTER_H_
