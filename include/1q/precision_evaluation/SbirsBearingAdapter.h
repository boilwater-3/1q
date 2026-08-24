/**
 * @file SbirsBearingAdapter.h
 * @brief SBIRS 周期结果 → fusion DetectionRecord（双星地面站适配）。
 *
 * 官方 `fusion::AdaptSbirsDetectionsToDetectionRecords` 不填量测原点，且 raw
 * 输出无目标身份（key=0）。本适配器利用归属层 target_id 恢复身份键，并把
 * ECI 极坐标角换算到「以卫星为原点的局部 ENU 方位 + 卫星 LLA 原点」，使双星
 * 角度量测进入融合三维方位滤波通道。
 */

#ifndef ONEQ_PRECISION_EVALUATION_SBIRS_BEARING_ADAPTER_H_
#define ONEQ_PRECISION_EVALUATION_SBIRS_BEARING_ADAPTER_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/coordinate/types.h"
#include "1q/fusion/DetectionRecord.h"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"

namespace precision_evaluation {

/**
 * @brief 把单星一个周期的归属结果适配为融合量测记录。
 * @param[in] result SBIRS 会话 StepWithResult 结果（frame + 归属）
 * @param[in] satellite_position_ecef 该星 ECEF 位置（m）
 * @param[in] gmst_rad 本周期 GMST（rad，ECI→ECEF）
 * @param[in] source_id 源通道标识（双星须互异）
 * @return 每条 detected 检测一条记录：key=归属 target_id、ENU 方位 + 卫星 LLA 原点、
 *         质量=IR SNR/1000 归一截断 [0,1]；无归属或坐标换算失败的检测跳过。
 */
ONEQ_API std::vector<fusion::DetectionRecord> AdaptSbirsResultToDetectionRecords(
    const sbirs_sensor::session::SbirsCycleResult& result,
    const oneq::coordinate::EcefPositionM& satellite_position_ecef, double gmst_rad,
    std::uint32_t source_id);

/**
 * @brief ECI 极坐标方位/俯仰（rad）→ ECI 单位向量。
 * @note 与 SBIRS 输出约定一致：az ∈ [0,2π)、el ∈ [-π/2,π/2]，
 *       u = (cos el · cos az, cos el · sin az, sin el)。
 */
ONEQ_API oneq::coordinate::Vector3d EciDirectionFromAzimuthElevationRad(double azimuth_rad,
                                                                        double elevation_rad);

}  // namespace precision_evaluation

#endif  // ONEQ_PRECISION_EVALUATION_SBIRS_BEARING_ADAPTER_H_
