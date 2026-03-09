// Copyright 2026. All Rights Reserved.
//
// Description: LAPJV assignment solver interface for dense cost matrices.

#ifndef AIRBORNE_RADAR_SIGNAL_ASSOCIATION_LAPJV_SOLVER_H_
#define AIRBORNE_RADAR_SIGNAL_ASSOCIATION_LAPJV_SOLVER_H_

#include <vector>

#include <Eigen/Core>

namespace airborne_radar {
namespace signal {
namespace association {

/// @brief LapjvSolver solves a linear assignment problem using
/// a LAPJV-style shortest augmenting path algorithm.
class LapjvSolver {
public:
  /// @brief Solve assignment on a square cost matrix.
  /// @param cost_matrix Dense square cost matrix.
  /// @return row -> assigned column index.
  std::vector<int> Solve(const Eigen::Ref<const Eigen::MatrixXf> &cost_matrix) const;
};

} // namespace association
} // namespace signal
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_SIGNAL_ASSOCIATION_LAPJV_SOLVER_H_
