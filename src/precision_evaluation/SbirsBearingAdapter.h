/**
 * @file SbirsBearingAdapter.h
 * @brief SBIRS 归属结果 → fusion DetectionRecord 的评估侧适配器（内部实现头）。
 *
 * 官方适配器 `fusion::SensorAdapters` 不填量测原点（SBIRS 角度-only 走关联通道），
 * 且 SBIRS raw output 不携带目标身份（key=0）。本评估侧适配器利用归属层
 * （`SbirsCycleResult::detection_attributions` 的 target_id，仿真归属/调试层语义，
 * 分层契约允许）恢复身份键，并把 ECI 极坐标角换算到"以卫星为原点的局部 ENU 方位 +
 * 卫星 LLA 原点"，使双星角度量测进入 fusion 的三维方位滤波通道（跨系对齐归调用方，
 * DetectionRecord 契约明示）。
 */

#ifndef ONEQ_SRC_PRECISION_EVALUATION_SBIRS_BEARING_ADAPTER_H_
#define ONEQ_SRC_PRECISION_EVALUATION_SBIRS_BEARING_ADAPTER_H_

#include <cstdint>
#include <vector>

#include "1q/coordinate/types.h"
#include "1q/fusion/DetectionRecord.h"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"

namespace precision_evaluation {
namespace internal {

/**
 * @brief 把单星一个周期的归属结果适配为融合量测记录。
 * @param[in] result SBIRS 会话 StepWithResult 结果（frame + 归属）
 * @param[in] satellite_position_ecef 该星 ECEF 位置（m）
 * @param[in] gmst_rad 本周期 GMST（rad，ECI→ECEF）
 * @param[in] source_id 源通道标识（双星须互异）
 * @return 每条 detected 检测一条记录：key=归属 target_id、ENU 方位 + 卫星 LLA 原点、
 *         质量=IR SNR/1000 归一截断 [0,1]；无归属或坐标换算失败的检测跳过。
 */
std::vector<fusion::DetectionRecord> AdaptSbirsResultToDetectionRecords(
    const sbirs_sensor::session::SbirsCycleResult& result,
    const oneq::coordinate::EcefPositionM& satellite_position_ecef, double gmst_rad,
    std::uint32_t source_id);

/**
 * @brief ECI 极坐标方位/俯仰（rad）→ ECI 单位向量。
 * @note 与 SBIRS 输出约定一致：az ∈ [0,2π)、el ∈ [-π/2,π/2]，
 *       u = (cos el · cos az, cos el · sin az, sin el)。
 */
oneq::coordinate::Vector3d EciDirectionFromAzimuthElevationRad(double azimuth_rad,
                                                               double elevation_rad);

}  // namespace internal
}  // namespace precision_evaluation

#endif  // ONEQ_SRC_PRECISION_EVALUATION_SBIRS_BEARING_ADAPTER_H_
