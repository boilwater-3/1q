/**
 * @file BearingCluster.h
 * @brief 定义库内共享的方位聚类工具，用于按角度一致性统计同方向发射/观测数量。
 */

#ifndef COMMON_GEOMETRY_BEARING_CLUSTER_H_
#define COMMON_GEOMETRY_BEARING_CLUSTER_H_

#include <cstddef>

#include "1q/api.hpp"

namespace oneq {
namespace common {
namespace geometry {

/**
 * @brief 波束宽度下限。低于此值的宽度会被钳制到此值，防止退化为点匹配。
 */
constexpr double kMinimumBeamwidthDeg = 1.0;

/**
 * @brief 判定两个方位是否落在同一波束宽度内。
 *
 * 方位差与俯仰差均小于给定波束宽度时视为同方向。波束宽度会被钳制到不小于
 * @p kMinimumBeamwidthDeg，避免零或负宽度导致任何观测都判为同方向。
 *
 * @param az_a_deg 第一个观测的方位角（单位：度）。
 * @param el_a_deg 第一个观测的俯仰角（单位：度）。
 * @param az_b_deg 第二个观测的方位角（单位：度）。
 * @param el_b_deg 第二个观测的俯仰角（单位：度）。
 * @param beamwidth_deg 波束宽度门限（单位：度）。
 * @return 同方向返回 true。
 */
inline bool AreBearingsCoherent(double az_a_deg, double el_a_deg, double az_b_deg,
                                double el_b_deg, double beamwidth_deg) {
  const double effective_beamwidth = beamwidth_deg < kMinimumBeamwidthDeg
                                         ? kMinimumBeamwidthDeg
                                         : beamwidth_deg;
  const double az_diff = az_a_deg > az_b_deg ? az_a_deg - az_b_deg : az_b_deg - az_a_deg;
  const double el_diff = el_a_deg > el_b_deg ? el_a_deg - el_b_deg : el_b_deg - el_a_deg;
  return az_diff < effective_beamwidth && el_diff < effective_beamwidth;
}

/**
 * @brief 统计同方向（波束宽度内）的元素数量（含自身）。
 *
 * 模板化以接受任意可调用对象（lambda、函数对象），调用方可通过 @p is_member
 * 判定一个索引是否参与聚类（如波形过滤），通过 @p azimuth_of / @p elevation_of
 * 取出每个参与元素的角度。返回 @p for_index 的同方向计数（含自身，因此孤立
 * 元素返回 1）。
 *
 * 该函数是纯几何聚类，不携带任何欺骗语义；"计数 ≥ 阈值即疑似假目标" 的判定
 * 由调用方负责。
 *
 * @tparam MemberPred bool(std::size_t) 谓词。
 * @tparam AzimuthGetter double(std::size_t) 方位角取角函数。
 * @tparam ElevationGetter double(std::size_t) 俯仰角取角函数。
 * @param element_count 元素总数。
 * @param is_member 返回该索引是否参与聚类。
 * @param azimuth_of 返回指定索引的方位角（度）。
 * @param elevation_of 返回指定索引的俯仰角（度）。
 * @param beamwidth_deg 波束宽度门限（单位：度）。
 * @param for_index 待统计的目标元素索引。
 * @return 与 for_index 同方向（含自身）的参与元素数量。
 */
template <typename MemberPred, typename AzimuthGetter, typename ElevationGetter>
std::size_t CountCoherentNeighbors(std::size_t element_count, MemberPred is_member,
                                   AzimuthGetter azimuth_of, ElevationGetter elevation_of,
                                   double beamwidth_deg, std::size_t for_index) {
  const double az = azimuth_of(for_index);
  const double el = elevation_of(for_index);
  std::size_t count = 0U;
  for (std::size_t i = 0U; i < element_count; ++i) {
    if (!is_member(i)) {
      continue;
    }
    if (AreBearingsCoherent(az, el, azimuth_of(i), elevation_of(i), beamwidth_deg)) {
      ++count;
    }
  }
  return count;
}

}  // namespace geometry
}  // namespace common
}  // namespace oneq

#endif  // COMMON_GEOMETRY_BEARING_CLUSTER_H_
