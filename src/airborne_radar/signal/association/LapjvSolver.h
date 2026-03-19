/**
 * @file LapjvSolver.h
 * @brief 定义基于 LAPJV 最短增广路算法的指派求解器。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_ASSOCIATION_LAPJV_SOLVER_H_
#define AIRBORNE_RADAR_SIGNAL_ASSOCIATION_LAPJV_SOLVER_H_

#include "airborne_radar/signal/association/AssignmentSolver.h"

namespace airborne_radar {
namespace signal {
namespace association {
/**
 * @brief LAPJV 指派求解器。
 * @details 使用 LAPJV 风格的最短增广路算法求解稠密代价矩阵上的线性指派问题。
 */
class LapjvSolver final : public IAssignmentSolver {
public:
/**
 * @brief 求解方阵代价矩阵上的指派关系。
 * @param cost_matrix 稠密方阵代价矩阵。
 * @return 行索引到列索引的映射。
 */
  std::vector<int> Solve(
      const Eigen::Ref<const Eigen::MatrixXf> &cost_matrix) const override;
};

} // namespace association
} // namespace signal
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_SIGNAL_ASSOCIATION_LAPJV_SOLVER_H_
