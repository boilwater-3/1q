/**
 * @file LapjvSolver.h
 * @brief 定义基于 LAPJV 最短增广路算法的全局最优指派求解器（common 单源）。
 */

#ifndef COMMON_OPTIMIZATION_LAPJV_SOLVER_H_
#define COMMON_OPTIMIZATION_LAPJV_SOLVER_H_

#include <Eigen/Core>
#include <vector>

namespace oneq {
namespace common {
namespace optimization {

/**
 * @brief LAPJV 指派求解器。
 * @details 使用 LAPJV 风格的最短增广路算法求解稠密方阵代价矩阵上的线性指派问题。
 */
class LapjvSolver final {
 public:
  /**
   * @brief 求解方阵代价矩阵上的指派关系。
   * @param cost_matrix 稠密方阵代价矩阵。
   * @return 行索引到列索引的映射；非方阵或空矩阵返回空解。
   */
  std::vector<int> Solve(const Eigen::Ref<const Eigen::MatrixXf>& cost_matrix) const;
};

}  // namespace optimization
}  // namespace common
}  // namespace oneq

#endif  // COMMON_OPTIMIZATION_LAPJV_SOLVER_H_
