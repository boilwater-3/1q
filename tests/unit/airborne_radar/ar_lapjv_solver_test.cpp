// Copyright 2026. All Rights Reserved.
//
// @file ar_lapjv_solver_test.cpp
// @brief 验证 LAPJV 指派求解器的正确性和边界行为。

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <algorithm>
#include <vector>

#include "airborne_radar/signal/association/LapjvSolver.h"

namespace airborne_radar {
namespace tests {

using signal::association::LapjvSolver;

TEST(LapjvSolverTest, EmptyMatrixReturnsEmpty) {
  LapjvSolver solver;
  Eigen::MatrixXf cost(0, 0);
  const std::vector<int> result = solver.Solve(cost);
  EXPECT_TRUE(result.empty());
}

TEST(LapjvSolverTest, SingleElementReturnsZero) {
  LapjvSolver solver;
  Eigen::MatrixXf cost(1, 1);
  cost(0, 0) = 5.0f;
  const std::vector<int> result = solver.Solve(cost);
  ASSERT_EQ(result.size(), 1U);
  EXPECT_EQ(result[0], 0);
}

TEST(LapjvSolverTest, TwoByTwoOptimalAssignment) {
  LapjvSolver solver;
  Eigen::MatrixXf cost(2, 2);
  // cost: [1, 10; 10, 1] — optimal assignment is 0→0, 1→1
  cost(0, 0) = 1.0f;
  cost(0, 1) = 10.0f;
  cost(1, 0) = 10.0f;
  cost(1, 1) = 1.0f;
  const std::vector<int> result = solver.Solve(cost);
  ASSERT_EQ(result.size(), 2U);
  EXPECT_EQ(result[0], 0);
  EXPECT_EQ(result[1], 1);
}

TEST(LapjvSolverTest, TwoByTwoAntidiagonalWins) {
  LapjvSolver solver;
  Eigen::MatrixXf cost(2, 2);
  // cost: [10, 1; 1, 10] — optimal assignment is 0→1, 1→0
  cost(0, 0) = 10.0f;
  cost(0, 1) = 1.0f;
  cost(1, 0) = 1.0f;
  cost(1, 1) = 10.0f;
  const std::vector<int> result = solver.Solve(cost);
  ASSERT_EQ(result.size(), 2U);
  EXPECT_EQ(result[0], 1);
  EXPECT_EQ(result[1], 0);
}

TEST(LapjvSolverTest, EqualCostsProducesValidPermutation) {
  LapjvSolver solver;
  Eigen::MatrixXf cost = Eigen::MatrixXf::Constant(4, 4, 1.0f);
  const std::vector<int> result = solver.Solve(cost);
  ASSERT_EQ(result.size(), 4U);

  // 结果应为有效排列：每列恰好分配一次
  std::vector<bool> column_used(4, false);
  for (std::size_t r = 0; r < result.size(); ++r) {
    const int col = result[r];
    EXPECT_GE(col, 0);
    EXPECT_LT(col, 4);
    EXPECT_FALSE(column_used[static_cast<std::size_t>(col)]) << "column " << col << " reused";
    column_used[static_cast<std::size_t>(col)] = true;
  }
}

TEST(LapjvSolverTest, LargeValuesHandledStably) {
  // 极大代价值不应导致数值溢出
  LapjvSolver solver;
  Eigen::MatrixXf cost(3, 3);
  const float huge = 1.0e10f;
  // 构造对角占优：行 i 的最优列为 i
  cost << 1.0f, huge, huge,
           huge, 2.0f, huge,
           huge, huge, 3.0f;
  const std::vector<int> result = solver.Solve(cost);
  ASSERT_EQ(result.size(), 3U);
  EXPECT_EQ(result[0], 0);
  EXPECT_EQ(result[1], 1);
  EXPECT_EQ(result[2], 2);
}

TEST(LapjvSolverTest, NonSquareMatrixRejected) {
  // 非方阵输入应被拒绝并返回空结果
  LapjvSolver solver;
  Eigen::MatrixXf wide(2, 4);
  wide.setRandom();
  wide = wide.cwiseAbs();
  EXPECT_TRUE(solver.Solve(wide).empty());

  Eigen::MatrixXf tall(4, 2);
  tall.setRandom();
  tall = tall.cwiseAbs();
  EXPECT_TRUE(solver.Solve(tall).empty());
}

}  // namespace tests
}  // namespace airborne_radar
