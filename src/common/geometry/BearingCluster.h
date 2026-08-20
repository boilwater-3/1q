/**
 * @file BearingCluster.h
 * @brief 定义库内共享的方位聚类工具，用于按角度一致性统计同方向发射/观测数量。
 */

#ifndef COMMON_GEOMETRY_BEARING_CLUSTER_H_
#define COMMON_GEOMETRY_BEARING_CLUSTER_H_

#include <cmath>
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
 * @brief 计算两个方位角（单位：度，0–360 周期）之间的最短角距离。
 *
 * 正确处理跨 0°/360° 边界：359° 与 1° 的距离为 2°，而非 358°。方位角由接收机在
 * ECEF 切平面解析得到，合法范围覆盖整个 0–360°，跨边界场景真实存在。
 *
 * @param az_a_deg 第一个方位角（单位：度）。
 * @param az_b_deg 第二个方位角（单位：度）。
 * @return [0, 180] 范围内的最短角距离（单位：度）。
 */
inline double AzimuthShortestDifferenceDeg(double az_a_deg, double az_b_deg) {
  double diff = az_a_deg - az_b_deg;
  diff -= 360.0 * std::floor((diff + 180.0) / 360.0);
  return diff < 0.0 ? -diff : diff;
}

/**
 * @brief 判定两个方位是否落在各自波束宽度内（分轴重载）。
 *
 * 方位差与俯仰差各自与对应轴的门限比较；两个轴独立钳制到不小于
 * @p kMinimumBeamwidthDeg。单宽度版本是调用本版本并将同一宽度传入两个轴的特例。
 *
 * @param az_a_deg 第一个观测的方位角（单位：度，0–360）。
 * @param el_a_deg 第一个观测的俯仰角（单位：度）。
 * @param az_b_deg 第二个观测的方位角（单位：度，0–360）。
 * @param el_b_deg 第二个观测的俯仰角（单位：度）。
 * @param az_beamwidth_deg 方位轴波束宽度门限（单位：度）。
 * @param el_beamwidth_deg 俯仰轴波束宽度门限（单位：度）。
 * @return 两轴均落在各自门限内时返回 true。
 */
inline bool AreBearingsCoherent(double az_a_deg, double el_a_deg, double az_b_deg,
                                double el_b_deg, double az_beamwidth_deg, double el_beamwidth_deg) {
  const double effective_az = az_beamwidth_deg < kMinimumBeamwidthDeg
                                  ? kMinimumBeamwidthDeg
                                  : az_beamwidth_deg;
  const double effective_el = el_beamwidth_deg < kMinimumBeamwidthDeg
                                  ? kMinimumBeamwidthDeg
                                  : el_beamwidth_deg;
  const double az_diff = AzimuthShortestDifferenceDeg(az_a_deg, az_b_deg);
  const double el_diff = std::abs(el_a_deg - el_b_deg);
  return az_diff < effective_az && el_diff < effective_el;
}

/**
 * @brief 判定两个方位是否落在同一波束宽度内（单宽度版本）。
 *
 * 委托到分轴版本，将同一波束宽度同时用于方位和俯仰轴。
 *
 * @param az_a_deg 第一个观测的方位角（单位：度，0–360）。
 * @param el_a_deg 第一个观测的俯仰角（单位：度）。
 * @param az_b_deg 第二个观测的方位角（单位：度，0–360）。
 * @param el_b_deg 第二个观测的俯仰角（单位：度）。
 * @param beamwidth_deg 波束宽度门限（单位：度）。
 * @return 同方向返回 true。
 */
inline bool AreBearingsCoherent(double az_a_deg, double el_a_deg, double az_b_deg,
                                double el_b_deg, double beamwidth_deg) {
  return AreBearingsCoherent(az_a_deg, el_a_deg, az_b_deg, el_b_deg,
                             beamwidth_deg, beamwidth_deg);
}

/**
 * @brief 统计同方向（分轴波束宽度内）的元素数量（含自身）。
 *
 * 方位和俯仰使用独立的波束宽度门限。单宽度版本是调用本版本并将同一宽度
 * 传入两个轴的特例。
 *
 * 该函数是纯几何聚类，不携带任何欺骗语义；"计数 ≥ 阈值即疑似假目标" 的判定
 * 由调用方负责。
 *
 * @par 复杂度
 * O(n²) 逐对比较，n 为元素总数。设计用于单周期接收机观测列表（典型数十条）；
 * 不适用于数百以上元素的批量场景。
 *
 * @tparam MemberPred bool(std::size_t) 谓词。
 * @tparam AzimuthGetter double(std::size_t) 方位角取角函数。
 * @tparam ElevationGetter double(std::size_t) 俯仰角取角函数。
 * @param element_count 元素总数。
 * @param is_member 返回该索引是否参与聚类。
 * @param azimuth_of 返回指定索引的方位角（度）。
 * @param elevation_of 返回指定索引的俯仰角（度）。
 * @param az_beamwidth_deg 方位轴波束宽度门限（单位：度）。
 * @param el_beamwidth_deg 俯仰轴波束宽度门限（单位：度）。
 * @param for_index 待统计的目标元素索引。
 * @return 与 for_index 同方向（含自身）的参与元素数量。
 */
template <typename MemberPred, typename AzimuthGetter, typename ElevationGetter>
std::size_t CountCoherentNeighbors(std::size_t element_count, MemberPred is_member,
                                   AzimuthGetter azimuth_of, ElevationGetter elevation_of,
                                   double az_beamwidth_deg, double el_beamwidth_deg,
                                   std::size_t for_index) {
  const double az = azimuth_of(for_index);
  const double el = elevation_of(for_index);
  std::size_t count = 0U;
  for (std::size_t i = 0U; i < element_count; ++i) {
    if (!is_member(i)) {
      continue;
    }
    if (AreBearingsCoherent(az, el, azimuth_of(i), elevation_of(i),
                            az_beamwidth_deg, el_beamwidth_deg)) {
      ++count;
    }
  }
  return count;
}

/**
 * @brief 统计同方向（单宽度波束内）的元素数量（含自身）。
 *
 * 委托到分轴版本，将同一波束宽度同时用于方位和俯仰轴。
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
  return CountCoherentNeighbors(element_count, is_member, azimuth_of, elevation_of,
                                beamwidth_deg, beamwidth_deg, for_index);
}

}  // namespace geometry
}  // namespace common
}  // namespace oneq

#endif  // COMMON_GEOMETRY_BEARING_CLUSTER_H_
