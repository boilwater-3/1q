/**
 * @file area_division.h
 * @brief 编队区域切分：把单个覆盖区域按编队飞机数切分为互不重叠的子区域。
 *
 * 业务语义：主机收到一个覆盖区域（多边形 / 圆形），由切分逻辑自动把区域
 * 分为每架飞机一份（分工覆盖），合起来恰好覆盖整个区域：
 * - 多边形：沿扫描航向切成等宽条带（保持牛耕式扫描结构，逐条带仍可直接
 *   交给 navigation::AreaCoveragePlanner 规划，航向/间距等参数原样透传）；
 * - 圆形：切成同心环（每架飞机沿一个环盘旋，环半径外 → 内算术均匀，与
 *   AreaCoveragePlanner 多环取点惯例一致；每机强制单环）。
 * 属 example 业务层算法面（docs/navigation/boundaries.md 非目标 #2：
 * 编队概念或任务语义属 example 业务层），不进入 include/1q。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_AREA_DIVISION_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_AREA_DIVISION_H_

#include <cstddef>
#include <string>
#include <vector>

#include "1q/navigation/CoverageArea.h"
#include "1q/navigation/CoveragePlanConfig.h"

namespace component_attachment {
namespace demo {

/**
 * @brief 编队切分结果：每架飞机的子区域与规划参数。
 * 下标 0 = 主机，1..N-1 = platforms[] 从机（按数组序）。
 */
struct FormationDivisionResult {
  std::vector<navigation::CoverageArea> sub_areas{};         /**< 每架飞机的子区域 */
  std::vector<navigation::CoveragePlanConfig> sub_configs{}; /**< 每架飞机的规划参数
                                                                （圆形切分时 orbit_rings
                                                                强制 1 = 每机单环） */
  bool ok{false};                                            /**< 切分成功 */
  std::string error{};                                       /**< 失败原因（ok = false 时） */
};

/**
 * @brief 把单个覆盖区域切分为 aircraft_count 个子区域（分工覆盖）。
 * @param area 主机收到的覆盖区域（多边形 / 圆形；模式须与形态匹配）。
 * @param config 覆盖规划参数（kScan ↔ 多边形、kOrbit ↔ 圆形，不匹配报错）。
 * @param aircraft_count 编队飞机数（>= 1；1 = 整区原样返回，等价于不切分）。
 * @return 切分结果；失败时 ok = false 并置 error（顶点不足、间距/半径非正、
 *         坐标非法、条带裁剪退化等）。
 */
FormationDivisionResult DivideArea(const navigation::CoverageArea& area,
                                   const navigation::CoveragePlanConfig& config,
                                   std::size_t aircraft_count);

}  // namespace demo
}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_AREA_DIVISION_H_
