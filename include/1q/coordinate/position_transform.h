/**
 * @file position_transform.h
 * @brief 定义不同坐标系位置之间的转换工具。
 */

#ifndef ONEQ_COORDINATE_POSITION_TRANSFORM_H_
#define ONEQ_COORDINATE_POSITION_TRANSFORM_H_

#include "1q/api.hpp"
#include "1q/coordinate/types.h"

namespace oneq {
namespace coordinate {

ONEQ_API bool IsValid(const LlaPositionDegM& lla);
ONEQ_API bool IsFinite(const EcefPositionM& ecef);
ONEQ_API bool IsFinite(const EnuPositionM& enu);
ONEQ_API bool IsFinite(const NedPositionM& ned);
ONEQ_API bool IsFinite(const NuePositionM& nue);

ONEQ_API bool TryLlaToEcef(const LlaPositionDegM& lla, EcefPositionM* ecef);
ONEQ_API bool TryEcefToLla(const EcefPositionM& ecef, LlaPositionDegM* lla);

ONEQ_API bool TryEcefToEnu(const EcefPositionM& ecef,
                           const LlaPositionDegM& origin_lla,
                           EnuPositionM* enu);
ONEQ_API bool TryEcefToNed(const EcefPositionM& ecef,
                           const LlaPositionDegM& origin_lla,
                           NedPositionM* ned);
ONEQ_API bool TryEcefToNue(const EcefPositionM& ecef,
                           const LlaPositionDegM& origin_lla,
                           NuePositionM* nue);

ONEQ_API bool TryLlaToEnu(const LlaPositionDegM& lla,
                          const LlaPositionDegM& origin_lla,
                          EnuPositionM* enu);
ONEQ_API bool TryLlaToNed(const LlaPositionDegM& lla,
                          const LlaPositionDegM& origin_lla,
                          NedPositionM* ned);
ONEQ_API bool TryLlaToNue(const LlaPositionDegM& lla,
                          const LlaPositionDegM& origin_lla,
                          NuePositionM* nue);

ONEQ_API NedPositionM ToNed(const EnuPositionM& enu);
ONEQ_API EnuPositionM ToEnu(const NedPositionM& ned);
ONEQ_API NuePositionM ToNue(const EnuPositionM& enu);
ONEQ_API EnuPositionM ToEnu(const NuePositionM& nue);
ONEQ_API NuePositionM ToNue(const NedPositionM& ned);
ONEQ_API NedPositionM ToNed(const NuePositionM& nue);

}  // namespace coordinate
}  // namespace oneq

#endif  // ONEQ_COORDINATE_POSITION_TRANSFORM_H_
