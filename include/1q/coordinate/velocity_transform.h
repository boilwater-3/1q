/**
 * @file velocity_transform.h
 * @brief 定义 WGS-84 坐标系下速度类型的帧间转换。
 *
 * ECEF 速度到局部速度的转换依赖参考点位置信息（origin_lla），
 * 这是因为 ENU 坐标轴的方向是位置相关的。
 * 局部坐标系间的速度转换与位置一致，采用纯轴重排。
 */

#ifndef ONEQ_COORDINATE_VELOCITY_TRANSFORM_H_
#define ONEQ_COORDINATE_VELOCITY_TRANSFORM_H_

#include "1q/api.hpp"
#include "1q/coordinate/types.h"

namespace oneq {
namespace coordinate {

// =============================================================================
// 输入校验
// =============================================================================

/**
 * @brief 校验 ECEF 速度各分量均为有限值。
 * @param[in] velocity 待校验的 ECEF 速度。
 * @return 三个分量均为有限值返回 true，否则返回 false。
 */
ONEQ_API bool IsFinite(const EcefVelocityMps& velocity);

/**
 * @brief 校验 ENU 速度各分量均为有限值。
 * @param[in] velocity 待校验的 ENU 速度。
 * @return 三个分量均为有限值返回 true，否则返回 false。
 */
ONEQ_API bool IsFinite(const EnuVelocityMps& velocity);

/**
 * @brief 校验 NED 速度各分量均为有限值。
 * @param[in] velocity 待校验的 NED 速度。
 * @return 三个分量均为有限值返回 true，否则返回 false。
 */
ONEQ_API bool IsFinite(const NedVelocityMps& velocity);

/**
 * @brief 校验 NUE 速度各分量均为有限值。
 * @param[in] velocity 待校验的 NUE 速度。
 * @return 三个分量均为有限值返回 true，否则返回 false。
 */
ONEQ_API bool IsFinite(const NueVelocityMps& velocity);

// =============================================================================
// ECEF 速度 → 局部坐标系
// =============================================================================

/**
 * @brief 将 ECEF 速度向量转换为 ENU 速度向量。
 * @param[in] ecef_velocity ECEF 速度（单位：m/s）。
 * @param[in] origin_lla 参考点 LLA，决定 ENU 坐标轴方向。
 * @param[out] enu_velocity 输出 ENU 速度；可为 nullptr。
 * @return 成功返回 true。
 * @note ECEF → ENU 的旋转矩阵由 origin_lla 的经度/纬度唯一确定。
 */
ONEQ_API bool TryEcefToEnuVelocity(const EcefVelocityMps& ecef_velocity,
                                   const LlaPositionDegM& origin_lla,
                                   EnuVelocityMps* enu_velocity);

/**
 * @brief 将 ECEF 速度向量转换为 NED 速度向量。
 * @param[in] ecef_velocity ECEF 速度（单位：m/s）。
 * @param[in] origin_lla 参考点 LLA。
 * @param[out] ned_velocity 输出 NED 速度；可为 nullptr。
 * @return 成功返回 true。
 * @note 内部 ECEF → ENU → NED。
 */
ONEQ_API bool TryEcefToNedVelocity(const EcefVelocityMps& ecef_velocity,
                                   const LlaPositionDegM& origin_lla,
                                   NedVelocityMps* ned_velocity);

/**
 * @brief 将 ECEF 速度向量转换为 NUE 速度向量。
 * @param[in] ecef_velocity ECEF 速度（单位：m/s）。
 * @param[in] origin_lla 参考点 LLA。
 * @param[out] nue_velocity 输出 NUE 速度；可为 nullptr。
 * @return 成功返回 true。
 * @note 内部 ECEF → ENU → NUE。
 */
ONEQ_API bool TryEcefToNueVelocity(const EcefVelocityMps& ecef_velocity,
                                   const LlaPositionDegM& origin_lla,
                                   NueVelocityMps* nue_velocity);

// =============================================================================
// 局部坐标系 → ECEF（反向转换）
// =============================================================================

/**
 * @brief 将 ENU 速度向量转换回 ECEF 速度向量。
 * @param[in] enu_velocity ENU 速度（单位：m/s）。
 * @param[in] origin_lla 参考点 LLA，决定 ENU 坐标轴方向。
 * @param[out] ecef_velocity 输出 ECEF 速度；可为 nullptr。
 * @return 成功返回 true。
 * @note 为 TryEcefToEnuVelocity 的逆运算。
 */
ONEQ_API bool TryEnuToEcefVelocity(const EnuVelocityMps& enu_velocity,
                                   const LlaPositionDegM& origin_lla,
                                   EcefVelocityMps* ecef_velocity);

// =============================================================================
// 局部坐标系互转（轴重排，无精度损失）
// =============================================================================

/**
 * @brief ENU 速度 → NED 速度。
 * @details NED.north = ENU.north, NED.east = ENU.east, NED.down = -ENU.up。
 */
ONEQ_API NedVelocityMps ToNedVelocity(const EnuVelocityMps& enu_velocity);

/**
 * @brief NED 速度 → ENU 速度。
 * @details ENU.east = NED.east, ENU.north = NED.north, ENU.up = -NED.down。
 */
ONEQ_API EnuVelocityMps ToEnuVelocity(const NedVelocityMps& ned_velocity);

/**
 * @brief ENU 速度 → NUE 速度。
 * @details NUE.north = ENU.north, NUE.up = ENU.up, NUE.east = ENU.east。
 */
ONEQ_API NueVelocityMps ToNueVelocity(const EnuVelocityMps& enu_velocity);

/**
 * @brief NUE 速度 → ENU 速度。
 * @details ENU.east = NUE.east, ENU.north = NUE.north, ENU.up = NUE.up。
 */
ONEQ_API EnuVelocityMps ToEnuVelocity(const NueVelocityMps& nue_velocity);

/**
 * @brief NED 速度 → NUE 速度。
 * @details NUE.north = NED.north, NUE.up = -NED.down, NUE.east = NED.east。
 */
ONEQ_API NueVelocityMps ToNueVelocity(const NedVelocityMps& ned_velocity);

/**
 * @brief NUE 速度 → NED 速度。
 * @details NED.north = NUE.north, NED.east = NUE.east, NED.down = -NUE.up。
 */
ONEQ_API NedVelocityMps ToNedVelocity(const NueVelocityMps& nue_velocity);

}  // namespace coordinate
}  // namespace oneq

#endif  // ONEQ_COORDINATE_VELOCITY_TRANSFORM_H_
