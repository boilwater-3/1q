/**
 * @file inertial_transform.h
 * @brief 定义 ECEF ↔ ECI（J2000 平赤道面）惯性坐标转换与 GMST 恒星时计算。
 *
 * 约定与近似（工程档）：
 * - 仅考虑地球自转绕 z 轴的旋转，旋转角 = GMST（格林尼治平恒星时）；忽略章动、
 *   极移与岁差（J2000 平赤道面近似为瞬时平赤道面）。
 * - 时间输入为 UTC 儒略日（JD_UTC），UT1 ≈ UTC 处理（差异 < 0.9 s，对应方位角
 *   误差 < 0.004°），符合本库仿真精度需求。
 * - 旋转约定：r_ECI = R3(θ_GMST) · r_ECEF，R3(θ) 为绕 z 轴旋转矩阵
 *   [[cosθ, −sinθ, 0], [sinθ, cosθ, 0], [0, 0, 1]]；θ_GMST = 0 时两系重合。
 * - 速度含输运项：v_ECI = R3(θ)·v_ECEF + ω_e × r_ECI（ω_e 沿 z 轴）。
 */

#ifndef ONEQ_COORDINATE_INERTIAL_TRANSFORM_H_
#define ONEQ_COORDINATE_INERTIAL_TRANSFORM_H_

#include "1q/api.hpp"
#include "1q/coordinate/types.h"

namespace oneq {
namespace coordinate {

/** @brief 地球自转角速率（WGS-84，rad/s）。 */
constexpr double kEarthRotationRateRadPerSec = 7.292115146706979e-5;

/**
 * @brief 由 UTC 儒略日计算 GMST（格林尼治平恒星时）。
 * @param[in] utc_julian_day UTC 儒略日（须为有限正数）
 * @param[out] gmst_rad GMST，单位 rad，范围 [0, 2π)
 * @return 输入非法（非有限或非正）时返回 false
 * @note 采用 IAU 1982 近似公式（Vallado 第三版式 3-47）：
 *       θ(deg) = 280.46061837 + 360.98564736629·d + 0.000387933·T² − T³/38710000，
 *       d = JD − 2451545.0，T = d/36525；UT1 ≈ UTC 近似。
 */
ONEQ_API bool TryComputeGmstRad(double utc_julian_day, double* gmst_rad);

/**
 * @brief 将 ECEF 位置旋转到 ECI（J2000 平赤道面）。
 * @param[in] ecef ECEF 位置（单位：m）
 * @param[in] gmst_rad GMST（rad，由 TryComputeGmstRad 计算）
 * @param[out] eci 输出 ECI 位置；可为 nullptr
 * @return 成功返回 true
 * @note r_ECI = R3(θ)·r_ECEF；纯旋转保模长。
 */
ONEQ_API bool TryEcefToEci(const EcefPositionM& ecef, double gmst_rad, EciPositionM* eci);

/**
 * @brief 将 ECI 位置旋转回 ECEF（TryEcefToEci 的逆）。
 * @param[in] eci ECI 位置（单位：m）
 * @param[in] gmst_rad GMST（rad）
 * @param[out] ecef 输出 ECEF 位置；可为 nullptr
 * @return 成功返回 true
 */
ONEQ_API bool TryEciToEcef(const EciPositionM& eci, double gmst_rad, EcefPositionM* ecef);

/**
 * @brief 将 ECEF 速度转换到 ECI（含地球自转输运项）。
 * @param[in] ecef_position ECEF 位置（单位：m，输运项 ω_e × r 需要）
 * @param[in] ecef_velocity ECEF 速度（单位：m/s）
 * @param[in] gmst_rad GMST（rad）
 * @param[out] eci_velocity 输出 ECI 速度；可为 nullptr
 * @return 成功返回 true
 * @note v_ECI = R3(θ)·v_ECEF + ω_e × r_ECI；忽略该项会把地球自转造成的 ~0.5 km/s
 *       地面速度误差引入惯性参考系。
 */
ONEQ_API bool TryEcefVelocityToEci(const EcefPositionM& ecef_position,
                                   const EcefVelocityMps& ecef_velocity, double gmst_rad,
                                   EciVelocityMps* eci_velocity);

}  // namespace coordinate
}  // namespace oneq

#endif  // ONEQ_COORDINATE_INERTIAL_TRANSFORM_H_
