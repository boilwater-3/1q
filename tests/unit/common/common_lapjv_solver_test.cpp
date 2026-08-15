// Copyright 2026. All Rights Reserved.
//
// @file common_lapjv_solver_test.cpp
// @brief 验证 common LAPJV 指派求解器的正确性和边界行为。

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <algorithm>
#include <vector>

#include "common/optimization/LapjvSolver.h"

namespace oneq {
namespace common {
namespace optimization {
namespace {

TEST(CommonLapjvSolverTest, EmptyMatrixReturnsEmpty) {
  LapjvSolver solver;
  Eigen::MatrixXf cost(0, 0);
  EXPECT_TRUE(solver.Solve(cost).empty());
}

TEST(CommonLapjvSolverTest, SingleElementReturnsZero) {
  LapjvSolver solver;
  Eigen::MatrixXf cost(1, 1);
  cost(0, 0) = 5.0f;
  const std::vector<int> result = solver.Solve(cost);
  ASSERT_EQ(result.size(), 1U);
  EXPECT_EQ(result[0], 0);
}

TEST(CommonLapjvSolverTest, TwoByTwoOptimalAssignment) {
  LapjvSolver solver;
  Eigen::MatrixXf cost(2, 2);
  cost(0, 0) = 1.0f;
  cost(0, 1) = 10.0f;
  cost(1, 0) = 10.0f;
  cost(1, 1) = 1.0f;
  const std::vector<int> result = solver.Solve(cost);
  ASSERT_EQ(result.size(), 2U);
  EXPECT_EQ(result[0], 0);
  EXPECT_EQ(result[1], 1);
}

TEST(CommonLapjvSolverTest, NonSquareMatrixRejected) {
  LapjvSolver solver;
  Eigen::MatrixXf wide(2, 4);
  wide.setRandom();
  EXPECT_TRUE(solver.Solve(wide).empty());
}

}  // namespace
}  // namespace optimization
}  // namespace common
}  // namespace oneq
