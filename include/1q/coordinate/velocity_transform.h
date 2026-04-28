/**
 * @file velocity_transform.h
 * @brief 定义不同坐标系速度之间的转换工具。
 */

#ifndef ONEQ_COORDINATE_VELOCITY_TRANSFORM_H_
#define ONEQ_COORDINATE_VELOCITY_TRANSFORM_H_

#include "1q/api.hpp"
#include "1q/coordinate/types.h"

namespace oneq {
namespace coordinate {

ONEQ_API bool IsFinite(const EcefVelocityMps& velocity);
ONEQ_API bool IsFinite(const EnuVelocityMps& velocity);
ONEQ_API bool IsFinite(const NedVelocityMps& velocity);
ONEQ_API bool IsFinite(const NueVelocityMps& velocity);

ONEQ_API bool TryEcefToEnuVelocity(const EcefVelocityMps& ecef_velocity,
                                   const LlaPositionDegM& origin_lla,
                                   EnuVelocityMps* enu_velocity);
ONEQ_API bool TryEcefToNedVelocity(const EcefVelocityMps& ecef_velocity,
                                   const LlaPositionDegM& origin_lla,
                                   NedVelocityMps* ned_velocity);
ONEQ_API bool TryEcefToNueVelocity(const EcefVelocityMps& ecef_velocity,
                                   const LlaPositionDegM& origin_lla,
                                   NueVelocityMps* nue_velocity);

ONEQ_API NedVelocityMps ToNedVelocity(const EnuVelocityMps& enu_velocity);
ONEQ_API EnuVelocityMps ToEnuVelocity(const NedVelocityMps& ned_velocity);
ONEQ_API NueVelocityMps ToNueVelocity(const EnuVelocityMps& enu_velocity);
ONEQ_API EnuVelocityMps ToEnuVelocity(const NueVelocityMps& nue_velocity);
ONEQ_API NueVelocityMps ToNueVelocity(const NedVelocityMps& ned_velocity);
ONEQ_API NedVelocityMps ToNedVelocity(const NueVelocityMps& nue_velocity);

}  // namespace coordinate
}  // namespace oneq

#endif  // ONEQ_COORDINATE_VELOCITY_TRANSFORM_H_
