/**
 * @file BoundarySearchSolver.h
 * @brief 定义电子侦察可截获边界二分搜索求解器。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_INTERCEPT_BOUNDARY_SEARCH_SOLVER_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_INTERCEPT_BOUNDARY_SEARCH_SOLVER_H_

#include <cmath>

namespace electronic_surveillance_radar {
namespace intercept {

/**
 * @brief BoundarySearchResult 描述边界搜索输出。
 */
struct BoundarySearchResult {
  bool converged{false}; /**< 是否收敛 */
  float boundary_range_m{0.0f}; /**< 边界距离（单位：m） */
  int iterations{0}; /**< 实际迭代次数 */
};

/**
 * @brief BoundarySearchSolver 使用二分法求解单调边界。
 */
class BoundarySearchSolver final {
 public:
  /**
   * @brief 在指定区间内搜索满足谓词的最大边界点。
   * @tparam Predicate 单调谓词类型，返回 `true` 表示当前距离可截获。
   * @param[in] min_range_m 搜索最小距离（单位：m）。
   * @param[in] max_range_m 搜索最大距离（单位：m）。
   * @param[in] resolution_m 收敛分辨率（单位：m）。
   * @param[in] max_iterations 最大迭代次数。
   * @param[in] predicate 单调谓词。
   * @return 边界搜索结果。
   */
  template <typename Predicate>
  static BoundarySearchResult Solve(float min_range_m, float max_range_m,
                                    float resolution_m, int max_iterations,
                                    const Predicate& predicate) {
    BoundarySearchResult result;
    if (max_range_m <= min_range_m || resolution_m <= 0.0f ||
        max_iterations <= 0) {
      return result;
    }

    float low = min_range_m;
    float high = max_range_m;
    float best = min_range_m;
    const bool low_ok = predicate(low);
    if (!low_ok) {
      result.boundary_range_m = min_range_m;
      return result;
    }
    best = low;

    for (int i = 0; i < max_iterations; ++i) {
      ++result.iterations;
      const float mid = 0.5f * (low + high);
      if (predicate(mid)) {
        best = mid;
        low = mid;
      } else {
        high = mid;
      }
      if (std::fabs(high - low) <= resolution_m) {
        result.converged = true;
        break;
      }
    }
    result.boundary_range_m = best;
    return result;
  }
};

}  // namespace intercept
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_INTERCEPT_BOUNDARY_SEARCH_SOLVER_H_
