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
/**
 * @brief 由视线向量计算方位角 Az = atan2(y, x)，单位 deg。
 * @param[in] los 视线向量（卫星→目标，ECEF 分量）。
 * @return ECEF 极坐标方位角：相对 ECEF x 轴（y/x 分量），取值范围 (-180°, 180°]。
 * @note 该参考系是 ECEF 极坐标，**非卫星局部地平系**；场景几何编排（扫描中心/
 *       覆盖）须按此参考系配置，约定见 docs/common/session_contract.md。
 */
float ComputeAzimuthDeg(const session::SbirsVector3M& los);
/**
 * @brief 由视线向量计算俯仰角 El = asin(z / |los|)，单位 deg；零向量返回 0。
 * @param[in] los 视线向量（卫星→目标，ECEF 分量）。
 * @return ECEF 极坐标俯仰角：相对赤道面（z 为 ECEF z 轴），星下点方向 ≈ −90°。
 * @note 该参考系是 ECEF 极坐标，**非卫星局部地平系**；场景几何编排（扫描中心/
 *       覆盖）须按此参考系配置，约定见 docs/common/session_contract.md。
 */
float ComputeElevationDeg(const session::SbirsVector3M& los);
/** @brief 计算两个 (az, el) 角度对之间的角距离，单位 deg。 */
float AngularSeparationDeg(float az_a_deg, float el_a_deg, float az_b_deg, float el_b_deg);
/**
 * @brief 由相对位置与相对速度推导视线角速度（标量，单位 deg/s）。
 * @param[in] relative_position_m 相对位置向量（卫星→目标），单位 m
 * @param[in] relative_velocity_m_per_s 相对速度向量，单位 m/s
 * @return 视线角速度模长 ω = |v_perp| / range，单位 deg/s；range ≤ 0 时返回 0
 * @note v_perp = v − (v·r̂) r̂ 为速度在视线垂直方向的分量；返回值用于动态滞后误差与 cue 外推。
 */
float ComputeRelativeAngularRateDegPerSec(const session::SbirsVector3M& relative_position_m,
                                          const session::SbirsVector3M& relative_velocity_m_per_s);
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
