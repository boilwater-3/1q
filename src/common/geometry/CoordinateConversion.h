/**
 * @file CoordinateConversion.h
 * @brief 定义三模块共享的坐标帧转换与速度帧适配工具。
 *
 * 本文件提取了 AR/EOS/ESR 三个 ExternalInputAdapter 中重复的匿名 namespace 转换逻辑，
 * 统一放置在 `oneq::internal::geometry` 命名空间下作为库内共享基础能力。
 */

#ifndef COMMON_GEOMETRY_COORDINATE_CONVERSION_H_
#define COMMON_GEOMETRY_COORDINATE_CONVERSION_H_

#include "1q/foundation/coordinate_transform.h"
#include "1q/foundation/pose_types.h"

namespace oneq {
namespace internal {
namespace geometry {

/**
 * @brief 描述外部坐标转换为本地局部坐标所需的参考系信息。
 */
struct LocalFrameReference {
  oneq::foundation::LlaCoordinateDegM origin_lla{};
  oneq::foundation::EulerAnglesDeg frame_attitude_deg{};
};

/**
 * @brief 速度输入的参考系类型。
 */
enum class VelocityFrame {
  kLocal = 0,
  kEcef = 1,
  kEnu = 2,
  kNed = 3
};

/**
 * @brief 将 ENU 坐标通过 frame_attitude_deg 旋转到局部坐标系。
 * @param[in] enu ENU 坐标（单位：m 或 m/s）。
 * @param[in] attitude_deg 局部坐标系相对 ENU 的姿态角（单位：deg）。
 * @return 局部坐标系下的向量。
 */
oneq::foundation::Vector3f ConvertEnuToLocal(
    const oneq::foundation::EnuCoordinateM& enu,
    const oneq::foundation::EulerAnglesDeg& attitude_deg);

/**
 * @brief 将 ECEF 速度向量转换为 ENU 速度向量。
 * @param[in] velocity_ecef_mps ECEF 速度（单位：m/s）。
 * @param[in] origin_lla 参考原点（WGS84 LLA）。
 * @param[out] velocity_enu_mps 输出 ENU 速度；可为 nullptr。
 * @return 成功返回 true。
 */
bool TryConvertEcefVelocityToEnu(
    const oneq::foundation::Vector3f& velocity_ecef_mps,
    const oneq::foundation::LlaCoordinateDegM& origin_lla,
    oneq::foundation::EnuCoordinateM* velocity_enu_mps);

/**
 * @brief 将 NED 速度转换为 ENU 速度（轴重排）。
 * @param[in] ned_mps NED 速度（x=north, y=east, z=down）。
 * @return ENU 速度（x=east, y=north, z=up）。
 */
oneq::foundation::EnuCoordinateM ToEnuFromNed(
    const oneq::foundation::Vector3f& ned_mps);

/**
 * @brief 将 `foundation::Vector3f` 表示的 ENU 速度包装为 `EnuCoordinateM`。
 * @param[in] enu_mps ENU 速度值。
 * @return 包装后的 `EnuCoordinateM`。
 */
oneq::foundation::EnuCoordinateM ToEnuFromEnuVector(
    const oneq::foundation::Vector3f& enu_mps);

/**
 * @brief 将 ECEF 位置转换到局部坐标系。
 * @param[in] ecef ECEF 位置（单位：m）。
 * @param[in] reference 局部坐标参考系。
 * @param[out] local 输出局部位置；可为 nullptr。
 * @return 成功返回 true。
 */
bool TryConvertEcefPositionToLocal(
    const oneq::foundation::EcefCoordinateM& ecef,
    const LocalFrameReference& reference,
    oneq::foundation::Vector3f* local);

/**
 * @brief 将指定参考系的速度向量转换到局部坐标系。
 * @param[in] velocity 源速度（单位：m/s）。
 * @param[in] frame 速度的当前参考系。
 * @param[in] reference 局部坐标参考系。
 * @param[out] local 输出局部速度；可为 nullptr。
 * @return 成功返回 true。
 */
bool TryConvertVelocityToLocal(
    const oneq::foundation::Vector3f& velocity,
    VelocityFrame frame,
    const LocalFrameReference& reference,
    oneq::foundation::Vector3f* local);

}  // namespace geometry
}  // namespace internal
}  // namespace oneq

#endif  // COMMON_GEOMETRY_COORDINATE_CONVERSION_H_
