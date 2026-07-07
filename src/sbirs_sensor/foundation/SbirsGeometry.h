/**
 * @file SbirsGeometry.h
 * @brief SBIRS-inspired 基础几何工具。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_FOUNDATION_SBIRS_GEOMETRY_H_
#define ONEQ_SRC_SBIRS_SENSOR_FOUNDATION_SBIRS_GEOMETRY_H_

#include "1q/sbirs_sensor/session/SbirsSceneTypes.h"

namespace sbirs_sensor {
namespace foundation {

/** @brief 计算三维向量点积。 */
double Dot(const session::SbirsVector3M& lhs, const session::SbirsVector3M& rhs);
/** @brief 计算三维向量模长（L2 范数）。 */
double Norm(const session::SbirsVector3M& value);
/** @brief 计算向量逐分量差 `lhs − rhs`。 */
session::SbirsVector3M Subtract(const session::SbirsVector3M& lhs,
                                const session::SbirsVector3M& rhs);
/** @brief 返回单位向量；零向量行为由实现约定。 */
session::SbirsVector3M Unit(const session::SbirsVector3M& value);
/** @brief 由视线向量计算方位角 Az = atan2(y, x)，单位 deg。 */
float ComputeAzimuthDeg(const session::SbirsVector3M& los);
/** @brief 由视线向量计算俯仰角（基于 z 分量与模长），单位 deg；零向量返回 0。 */
float ComputeElevationDeg(const session::SbirsVector3M& los);
/** @brief 计算两个 (az, el) 角度对之间的角距离，单位 deg。 */
float AngularSeparationDeg(float az_a_deg, float el_a_deg, float az_b_deg, float el_b_deg);
/**
 * @brief 用有限 LOS 线段判定目标视线是否被地球球体遮挡。
 * @param[in] satellite_position_ecef_m 卫星 ECEF 位置，单位 m
 * @param[in] target_position_ecef_m 目标 ECEF 位置，单位 m
 * @param[in] earth_radius_m 地球半径，单位 m
 * @return 视线在卫星到目标的有限线段内穿过地球球体返回 true，否则返回 false
 * @note 仅做球体相交判定，不负责地形、云图或三维大气廓线。
 */
bool IsEarthOcculted(const session::SbirsVector3M& satellite_position_ecef_m,
                     const session::SbirsVector3M& target_position_ecef_m, double earth_radius_m);

}  // namespace foundation
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_FOUNDATION_SBIRS_GEOMETRY_H_
