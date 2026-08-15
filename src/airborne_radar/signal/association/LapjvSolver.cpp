/**
 * @file LapjvSolver.cpp
 * @brief AR LAPJV 指派求解器薄适配层（common 单源）。
 */

#include "airborne_radar/signal/association/LapjvSolver.h"

#include "common/optimization/LapjvSolver.h"

namespace airborne_radar {
namespace signal {
namespace association {

std::vector<int> LapjvSolver::Solve(const Eigen::Ref<const Eigen::MatrixXf>& cost_matrix) const {
  return oneq::common::optimization::LapjvSolver().Solve(cost_matrix);
}

}  // namespace association
}  // namespace signal
}  // namespace airborne_radar
