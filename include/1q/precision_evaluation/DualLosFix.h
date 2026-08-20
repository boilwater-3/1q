/**
 * @file DualLosFix.h
 * @brief 双视线（双星）交会定位纯函数原语（需求 3.2.1.6.3 双星定位位置误差的几何解算）。
 */

#ifndef ONEQ_PRECISION_EVALUATION_DUAL_LOS_FIX_H_
#define ONEQ_PRECISION_EVALUATION_DUAL_LOS_FIX_H_

#include "1q/api.hpp"
#include "1q/coordinate/types.h"

namespace precision_evaluation {

/**
 * @brief 两条视线（异面直线）最近接近交会解算（双星三角定位）。
 * @param[in] origin_a 视线 A 起点（卫星 A ECEF 位置，单位 m）
 * @param[in] direction_a 视线 A 方向（ECEF 单位向量；无需严格单位化）
 * @param[in] origin_b 视线 B 起点（卫星 B ECEF 位置，单位 m）
 * @param[in] direction_b 视线 B 方向（ECEF 单位向量；无需严格单位化）
 * @param[out] fix_position 交会位置（两最近点连线中点，ECEF，单位 m）
 * @param[out] residual_m 两直线最近距离（几何残差，单位 m；理想交会为 0）
 * @return 几何可解返回 true；两线平行/退化（叉积模长近零）或出参为空返回 false。
 * @note 经典立体交会：对两条不共面的视线各取最近点，取中点为定位解。残差是交会
 *       "虚实"的几何指示（残差大 = 交会角差或视线噪声大），进误差样本一并输出。
 */
bool ONEQ_API TryComputeDualLosFixM(const oneq::coordinate::EcefPositionM& origin_a,
                                    const oneq::coordinate::Vector3d& direction_a,
                                    const oneq::coordinate::EcefPositionM& origin_b,
                                    const oneq::coordinate::Vector3d& direction_b,
                                    oneq::coordinate::EcefPositionM* fix_position,
                                    double* residual_m);

}  // namespace precision_evaluation

#endif  // ONEQ_PRECISION_EVALUATION_DUAL_LOS_FIX_H_
