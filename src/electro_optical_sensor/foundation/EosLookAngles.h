/**
 * @file EosLookAngles.h
 * @brief 平台锚点 ENU 位置 → EOS 体系球坐标（斜距/方位/仰角）的库内换算。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_FOUNDATION_EOS_LOOK_ANGLES_H_
#define ELECTRO_OPTICAL_SENSOR_FOUNDATION_EOS_LOOK_ANGLES_H_

#include <cstdint>

#include "1q/coordinate/types.h"

namespace electro_optical_sensor {
namespace foundation {

/**
 * @brief 由平台锚点 ENU 位置与平台姿态派生体系球坐标。
 * @param[in] position_x 平台锚点 ENU 位置 x（东向，单位：m）。
 * @param[in] position_y 平台锚点 ENU 位置 y（北向，单位：m）。
 * @param[in] position_z 平台锚点 ENU 位置 z（天向，单位：m）。
 * @param[in] platform_attitude_deg 平台姿态角（Body->ENU，单位：deg）。
 * @param[out] range_m 输出斜距（单位：m）。
 * @param[out] azimuth_deg 输出体系方位角（单位：deg；atan2(body_y, body_x)）。
 * @param[out] elevation_deg 输出体系仰角（单位：deg；出水平面为正）。
 * @return 成功返回 true；位置模长低于数值下限（退化几何）返回 false。
 * @note 方位/仰角定义在平台体系（ENU 旋入体系后取角），与扫描中心配置同帧；
 *       输入面 ENU 契约见 docs/common/contract.md「场景目标平台锚点 ENU 输入契约」。
 */
bool TryResolveEosLookAngles(double position_x, double position_y, double position_z,
                             const oneq::coordinate::EulerAnglesDeg& platform_attitude_deg,
                             float* range_m, float* azimuth_deg, float* elevation_deg);

/** @brief TryResolveEosLookAngles 认定的退化几何位置模长下限（单位：m）。 */
float EosLookAngleNormFloorM();

}  // namespace foundation
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_FOUNDATION_EOS_LOOK_ANGLES_H_
