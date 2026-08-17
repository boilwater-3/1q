/**
 * @file LapjvSolver.h
 * @brief AR LAPJV 指派求解器薄适配层（common 单源）。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_ASSOCIATION_LAPJV_SOLVER_H_
#define AIRBORNE_RADAR_SIGNAL_ASSOCIATION_LAPJV_SOLVER_H_

#include <Eigen/Core>
#include <vector>

namespace airborne_radar {
namespace signal {
namespace association {
/**
 * @brief LAPJV 指派求解器。
 * @details 转发 `oneq::common::optimization::LapjvSolver`，供既有 AR 内部引用。
 */
class LapjvSolver final {
 public:
  /**
   * @brief 求解方阵代价矩阵上的指派关系。
   * @param cost_matrix 稠密方阵代价矩阵。
   * @return 行索引到列索引的映射。
   */
  std::vector<int> Solve(const Eigen::Ref<const Eigen::MatrixXf>& cost_matrix) const;
};

}  // namespace association
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_ASSOCIATION_LAPJV_SOLVER_H_
