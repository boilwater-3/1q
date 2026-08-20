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

namespace electro_optical_sensor {
namespace session {

/**
 * @brief EOS 外部平台运动学输入。
 * @note 速度固定为 ECEF 坐标系；EOS 没有独立 mount 字段，示例适配器按传感器视轴与
 *       机体系对齐处理。AR 的雷达安装角通过 ArOrientationConfig::mount_angles_deg 配置。
 */
struct ONEQ_API EosExternalPoseInput {
  oneq::coordinate::EcefPositionM platform_position_ecef_m{}; /**< 平台位置（ECEF，m） */
  oneq::coordinate::EcefVelocityMps platform_velocity_mps{};  /**< 平台速度（ECEF，单位：m/s） */
  oneq::coordinate::EulerAnglesDeg platform_attitude_deg{};   /**< 平台姿态角（Body->ENU，deg） */
};

/**
 * @brief EOS 外部目标输入（统一入口，世界真值 ECEF/LLA）。
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
  kCoordinateTransformFail
};

/**
 * @brief 将外部目标输入（世界 ECEF/LLA + ECEF 速度）转换为平台锚点 ENU 场景目标。
 * @param[in] target_id 目标标识。
 * @param[in] input 外部目标输入。
 * @param[in] reference EOS 局部坐标参考系（`origin_lla` 为平台锚点；内部经公共
 *            `oneq::coordinate::TryMakeEnuSceneState` 一站式转换）。
 * @param[out] target 输出平台锚点 ENU 场景目标。
 * @param[out] status 可选输出状态，nullptr 表示不关心失败原因。
 * @return 转换成功返回 true；输入非法、坐标变换失败或输出为空返回 false。
 * @note ENU 契约见 docs/common/contract.md「场景目标平台锚点 ENU 输入契约」。
 */
ONEQ_API bool TryMakeEosSceneTargetFromExternalInput(
    std::uint64_t target_id, const EosExternalTargetInput& input,
    const oneq::coordinate::LocalFrameReference& reference, EosSceneTarget* target,
    EosCoordinateStatus* status = nullptr);

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_EXTERNAL_INPUT_ADAPTER_H_
