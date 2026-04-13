/**
 * @file coordinate_transform.h
 * @brief 定义 WGS84 下 LLA/ECEF/ENU 之间的轻量坐标转换工具。
 */

#ifndef ONEQ_FOUNDATION_COORDINATE_TRANSFORM_H_
#define ONEQ_FOUNDATION_COORDINATE_TRANSFORM_H_

#include "1q/api.hpp"
#include "1q/foundation/pose_types.h"

namespace oneq {
namespace foundation {

/**
 * @brief LlaCoordinateDegM 表示大地坐标（纬度/经度/高程）。
 */
struct ONEQ_API LlaCoordinateDegM {
  double latitude_deg{0.0};  /**< 纬度（单位：deg，范围 [-90, 90]） */
  double longitude_deg{0.0}; /**< 经度（单位：deg，范围 [-180, 180]） */
  double altitude_m{0.0};    /**< 椭球高（单位：m） */
};

/**
 * @brief EcefCoordinateM 表示地心地固坐标（单位：m）。
 */
struct ONEQ_API EcefCoordinateM {
  double x_m{0.0}; /**< ECEF X（单位：m） */
  double y_m{0.0}; /**< ECEF Y（单位：m） */
  double z_m{0.0}; /**< ECEF Z（单位：m） */
};

/**
 * @brief EnuCoordinateM 表示局部东-北-天坐标（单位：m）。
 * @note 字段含义固定为 x=east, y=north, z=up。
 */
struct ONEQ_API EnuCoordinateM {
  double x_m{0.0}; /**< East 方向分量（单位：m） */
  double y_m{0.0}; /**< North 方向分量（单位：m） */
  double z_m{0.0}; /**< Up 方向分量（单位：m） */
};

/**
 * @brief 校验 LLA 输入是否在可计算范围内。
 * @param[in] lla LLA 输入。
 * @return 若纬度、经度、高程均为有限值且角度范围合法，则返回 true。
 */
ONEQ_API bool IsValidLla(const LlaCoordinateDegM& lla);

/**
 * @brief 将 LLA 坐标转换为 ECEF 坐标。
 * @param[in] lla LLA 输入。
 * @param[out] ecef 输出 ECEF；允许为 nullptr。
 * @return 成功返回 true；输入非法或输出为空返回 false。
 */
ONEQ_API bool TryLlaToEcef(const LlaCoordinateDegM& lla, EcefCoordinateM* ecef);

/**
 * @brief 将 ECEF 坐标转换为 LLA 坐标。
 * @param[in] ecef ECEF 输入。
 * @param[out] lla 输出 LLA；允许为 nullptr。
 * @return 成功返回 true；输入非法或输出为空返回 false。
 */
ONEQ_API bool TryEcefToLla(const EcefCoordinateM& ecef, LlaCoordinateDegM* lla);

/**
 * @brief 将 ECEF 坐标转换到以 origin_lla 为参考点的 ENU 坐标。
 * @param[in] ecef ECEF 输入。
 * @param[in] origin_lla ENU 参考原点（LLA）。
 * @param[out] enu 输出 ENU；允许为 nullptr。
 * @return 成功返回 true；输入非法或输出为空返回 false。
 */
ONEQ_API bool TryEcefToEnu(const EcefCoordinateM& ecef, const LlaCoordinateDegM& origin_lla,
                           EnuCoordinateM* enu);

/**
 * @brief 将 LLA 坐标转换到以 origin_lla 为参考点的 ENU 坐标。
 * @param[in] lla LLA 输入。
 * @param[in] origin_lla ENU 参考原点（LLA）。
 * @param[out] enu 输出 ENU；允许为 nullptr。
 * @return 成功返回 true；输入非法或输出为空返回 false。
 */
ONEQ_API bool TryLlaToEnu(const LlaCoordinateDegM& lla, const LlaCoordinateDegM& origin_lla,
                          EnuCoordinateM* enu);

/**
 * @brief 将 ENU 坐标转换为通用 Vector3f。
 * @param[in] enu ENU 输入。
 * @return `x=east, y=north, z=up` 的三维向量。
 */
ONEQ_API Vector3f ToVector3f(const EnuCoordinateM& enu);

}  // namespace foundation
}  // namespace oneq

#endif  // ONEQ_FOUNDATION_COORDINATE_TRANSFORM_H_
