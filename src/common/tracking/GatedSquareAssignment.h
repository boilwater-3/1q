/**
 * @file GatedSquareAssignment.h
 * @brief 门限内代价 → 方阵增广 → LAPJV 指派（关联核单源）。
 */

#ifndef COMMON_TRACKING_GATED_SQUARE_ASSIGNMENT_H_
#define COMMON_TRACKING_GATED_SQUARE_ASSIGNMENT_H_

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace oneq {
namespace common {
namespace tracking {

/**
 * @brief 由矩形门限代价构造方阵代价矩阵。
 *
 * @param gated_costs 行=航迹、列=量测；> gate_threshold 或非有限视为拒绝。
 * @param gate_threshold 波门阈值（同时作为未分配代价）。
 * @return 方阵；拒绝格填 nextafter(gate)；哑行/哑列填 gate。
 */
inline Eigen::MatrixXf BuildAugmentedSquareCostMatrix(const Eigen::MatrixXf& gated_costs,
                                                      float gate_threshold) {
  const std::size_t rows = static_cast<std::size_t>(gated_costs.rows());
  const std::size_t cols = static_cast<std::size_t>(gated_costs.cols());
  const std::size_t dim = std::max(rows, cols);
  const float unassigned_cost = gate_threshold;
  const float rejected_cost =
      std::nextafter(unassigned_cost, std::numeric_limits<float>::infinity());

  Eigen::MatrixXf cost_matrix(static_cast<Eigen::Index>(dim), static_cast<Eigen::Index>(dim));
  cost_matrix.setConstant(rejected_cost);
  if (dim > cols) {
    cost_matrix.rightCols(static_cast<Eigen::Index>(dim - cols)).setConstant(unassigned_cost);
  }
  if (dim > rows) {
    cost_matrix.bottomRows(static_cast<Eigen::Index>(dim - rows)).setConstant(unassigned_cost);
  }

  for (std::size_t r = 0U; r < rows; ++r) {
    for (std::size_t c = 0U; c < cols; ++c) {
      const float cost = gated_costs(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c));
      if (std::isfinite(cost) && cost <= gate_threshold) {
        cost_matrix(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) = cost;
      }
    }
  }
  return cost_matrix;
}

/**
 * @brief 对已增广方阵求解 LAPJV，并过滤出真实匹配（代价 ≤ unassigned_cost）。
 * @tparam Solver 需提供 `std::vector<int> Solve(const Eigen::MatrixXf&) const`。
 * @return assignment[r] = 匹配列（若 r 有有效匹配），否则 -1；长度 = rows。
 */
template <typename Solver>
inline std::vector<int> SolveAugmentedSquareAssignment(const Eigen::MatrixXf& square_cost_matrix,
                                                       std::size_t rows, std::size_t cols,
                                                       float unassigned_cost,
                                                       const Solver& solver) {
  std::vector<int> row_to_col(rows, -1);
  if (rows == 0U || cols == 0U || square_cost_matrix.rows() == 0) {
    return row_to_col;
  }
  const std::vector<int> assignment = solver.Solve(square_cost_matrix);
  for (std::size_t r = 0U; r < rows && r < assignment.size(); ++r) {
    const int assigned_col = assignment[r];
    if (assigned_col < 0 || static_cast<std::size_t>(assigned_col) >= cols) {
      continue;
    }
    const float matched_cost = square_cost_matrix(static_cast<Eigen::Index>(r),
                                                  static_cast<Eigen::Index>(assigned_col));
    if (matched_cost <= unassigned_cost) {
      row_to_col[r] = assigned_col;
    }
  }
  return row_to_col;
}

/**
 * @brief 门限矩形代价 → 方阵增广 → LAPJV → 行到列匹配。
 */
template <typename Solver>
inline std::vector<int> SolveGatedSquareAssignment(const Eigen::MatrixXf& gated_costs,
                                                   float gate_threshold, const Solver& solver) {
  const std::size_t rows = static_cast<std::size_t>(gated_costs.rows());
  const std::size_t cols = static_cast<std::size_t>(gated_costs.cols());
  const Eigen::MatrixXf square = BuildAugmentedSquareCostMatrix(gated_costs, gate_threshold);
  return SolveAugmentedSquareAssignment(square, rows, cols, gate_threshold, solver);
}

}  // namespace tracking
}  // namespace common
}  // namespace oneq

#endif  // COMMON_TRACKING_GATED_SQUARE_ASSIGNMENT_H_
