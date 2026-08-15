// Copyright 2026. All Rights Reserved.
//
// @file rir_track_associator_test.cpp
// @brief 验证 RIR 轻量跟踪子集门限 + LAPJV 全局最优关联（阶段 2-T T2，N1/N2）。

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <limits>
#include <vector>

#include "remote_identification_radar/tracking/RirLapjvSolver.h"
#include "remote_identification_radar/tracking/RirTrackAssociator.h"
#include "remote_identification_radar/tracking/RirTrackTypes.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using tracking::RirAssociationConfig;
using tracking::RirMeasurementCovariance;
using tracking::RirStateCovariance;
using tracking::RirStateVector;
using tracking::RirTrackAssociator;
using tracking::RirTrackMeasurement;
using tracking::RirTrackSeed;

RirTrackMeasurement MakeMeasurement(std::size_t source_index, float px, float py, float pz) {
  RirTrackMeasurement measurement;
  measurement.source_index = source_index;
  measurement.position = Eigen::Vector3f(px, py, pz);
  measurement.measurement_covariance = RirMeasurementCovariance::Identity();
  return measurement;
}

RirTrackSeed MakeSeed(std::uint64_t key, float px, float py, float pz,
                      const RirStateCovariance& covariance) {
  RirTrackSeed seed;
  seed.association_key = key;
  seed.has_position = true;
  seed.position = Eigen::Vector3f(px, py, pz);
  seed.has_gaussian_state = true;
  RirStateVector mean = RirStateVector::Zero();
  mean(0) = px;
  mean(2) = py;
  mean(4) = pz;
  seed.gaussian_state = tracking::RirGaussianState(mean, covariance);
  return seed;
}

/// @brief 无既有航迹：全部量测分配单调递增新键，0 保留。
TEST(RirTrackAssociatorTest, NoSeedsAllocatesMonotonicKeys) {
  RirTrackAssociator associator;
  std::vector<RirTrackMeasurement> measurements;
  measurements.push_back(MakeMeasurement(3U, 100.0f, 0.0f, 0.0f));
  measurements.push_back(MakeMeasurement(7U, 200.0f, 0.0f, 0.0f));

  const auto result = associator.Associate(measurements, {}, 1.0f);

  ASSERT_EQ(result.measurements.size(), 2U);
  EXPECT_EQ(result.measurements[0].association_key, 1U);
  EXPECT_FALSE(result.measurements[0].matched_existing_track);
  EXPECT_EQ(result.measurements[1].association_key, 2U);
  EXPECT_TRUE(result.matches.empty());
  EXPECT_TRUE(result.missed_track_keys.empty());
}

/// @brief 3σ 门（阈值 9）：门内量测命中既有键，门外量测分配新键。
TEST(RirTrackAssociatorTest, GateAndNearestNeighbourAssignments) {
  RirTrackAssociator associator;
  RirStateCovariance covariance = RirStateCovariance::Zero();
  covariance(0, 0) = 0.0f;
  covariance(2, 2) = 0.0f;
  covariance(4, 4) = 0.0f;
  std::vector<RirTrackSeed> seeds;
  seeds.push_back(MakeSeed(42U, 0.0f, 0.0f, 0.0f, covariance));

  std::vector<RirTrackMeasurement> measurements;
  measurements.push_back(MakeMeasurement(0U, 0.1f, 0.0f, 0.0f));   // d²=0.01
  measurements.push_back(MakeMeasurement(1U, 10.0f, 0.0f, 0.0f));  // d²=100

  const auto result = associator.Associate(measurements, seeds, 0.0f);

  ASSERT_EQ(result.measurements.size(), 2U);
  EXPECT_EQ(result.measurements[0].association_key, 42U);
  EXPECT_TRUE(result.measurements[0].matched_existing_track);
  EXPECT_EQ(result.measurements[1].association_key, 1U);
  EXPECT_FALSE(result.measurements[1].matched_existing_track);
  ASSERT_EQ(result.matches.size(), 1U);
  EXPECT_EQ(result.matches[0].association_key, 42U);
  EXPECT_EQ(result.matches[0].source_index, 0U);
  EXPECT_NEAR(result.matches[0].cost, 0.01f, 1.0e-5f);
  EXPECT_TRUE(result.missed_track_keys.empty());
}

/// @brief 波门按量测误差定标：同一位置差在 R 小时被拒绝、R 大时被接受。
TEST(RirTrackAssociatorTest, GateIsScaledByMeasurementCovariance) {
  RirTrackAssociator associator;
  RirStateCovariance covariance = RirStateCovariance::Zero();
  std::vector<RirTrackSeed> seeds;
  seeds.push_back(MakeSeed(7U, 0.0f, 0.0f, 0.0f, covariance));

  std::vector<RirTrackMeasurement> measurements;
  RirTrackMeasurement precise = MakeMeasurement(0U, 2.0f, 0.0f, 0.0f);
  precise.measurement_covariance = RirMeasurementCovariance::Identity() * 0.1f;  // d²=40
  RirTrackMeasurement coarse = MakeMeasurement(1U, 2.0f, 0.0f, 0.0f);
  coarse.measurement_covariance = RirMeasurementCovariance::Identity();  // d²=4
  measurements.push_back(precise);
  measurements.push_back(coarse);

  const auto result = associator.Associate(measurements, seeds, 0.0f);

  ASSERT_EQ(result.measurements.size(), 2U);
  EXPECT_EQ(result.measurements[0].association_key, 1U);  // precise 被门拒绝 → 新键
  EXPECT_FALSE(result.measurements[0].matched_existing_track);
  EXPECT_EQ(result.measurements[1].association_key, 7U);  // coarse 命中 seed
  EXPECT_TRUE(result.measurements[1].matched_existing_track);
}

/// @brief 全局最近邻唯一分配：每条航迹/量测至多命中一次。
TEST(RirTrackAssociatorTest, UniqueAssignmentUsesGlobalNearestNeighbour) {
  RirTrackAssociator associator;
  RirStateCovariance covariance = RirStateCovariance::Zero();
  std::vector<RirTrackSeed> seeds;
  seeds.push_back(MakeSeed(10U, 0.0f, 0.0f, 0.0f, covariance));
  seeds.push_back(MakeSeed(11U, 2.5f, 0.0f, 0.0f, covariance));

  std::vector<RirTrackMeasurement> measurements;
  measurements.push_back(MakeMeasurement(0U, 0.6f, 0.0f, 0.0f));
  measurements.push_back(MakeMeasurement(1U, 0.9f, 0.0f, 0.0f));

  const auto result = associator.Associate(measurements, seeds, 0.0f);

  ASSERT_EQ(result.matches.size(), 2U);
  EXPECT_EQ(result.measurements[0].association_key, 10U);
  EXPECT_EQ(result.measurements[1].association_key, 11U);
  EXPECT_TRUE(result.missed_track_keys.empty());
}

/// @brief 全局最优指派（N2）：贪心最近邻会被全局最小边带偏的冲突场景，
///        LAPJV 必须选择总代价更小的换位配对。
/// 场景（零过程噪声 + R=I，代价 = 欧氏距离平方，门 9）：
///   seed A(0) - 量测 a(0.3)：0.09（全局最小边）；seed A - 量测 b(-1.5)：2.25；
///   seed B(2.0) - 量测 a：2.89；seed B - 量测 b：12.25（门外）。
/// 贪心取 A-a 后 B 失配、b 新键（总代价 0.09+9+9）；全局最优为 A-b + B-a（5.14）。
TEST(RirTrackAssociatorTest, GlobalOptimumBeatsGreedyNearestNeighbour) {
  RirTrackAssociator associator;
  RirStateCovariance covariance = RirStateCovariance::Zero();
  std::vector<RirTrackSeed> seeds;
  seeds.push_back(MakeSeed(100U, 0.0f, 0.0f, 0.0f, covariance));   // A
  seeds.push_back(MakeSeed(200U, 2.0f, 0.0f, 0.0f, covariance));   // B

  std::vector<RirTrackMeasurement> measurements;
  measurements.push_back(MakeMeasurement(0U, 0.3f, 0.0f, 0.0f));   // a
  measurements.push_back(MakeMeasurement(1U, -1.5f, 0.0f, 0.0f));  // b

  const auto result = associator.Associate(measurements, seeds, 0.0f);

  ASSERT_EQ(result.measurements.size(), 2U);
  // 全局最优换位：a→B、b→A，两条既有航迹全部命中。
  EXPECT_EQ(result.measurements[0].association_key, 200U);
  EXPECT_TRUE(result.measurements[0].matched_existing_track);
  EXPECT_EQ(result.measurements[1].association_key, 100U);
  EXPECT_TRUE(result.measurements[1].matched_existing_track);
  ASSERT_EQ(result.matches.size(), 2U);
  EXPECT_NEAR(result.matches[0].cost, 2.89f, 1.0e-4f);
  EXPECT_NEAR(result.matches[1].cost, 2.25f, 1.0e-4f);
  EXPECT_TRUE(result.missed_track_keys.empty());
}

/// @brief 量测多于航迹：增广哑行吸收多余量测，多余量测分配新键。
TEST(RirTrackAssociatorTest, MoreMeasurementsThanSeedsAssignsSurplusToNewKeys) {
  RirTrackAssociator associator;
  RirStateCovariance covariance = RirStateCovariance::Zero();
  std::vector<RirTrackSeed> seeds;
  seeds.push_back(MakeSeed(5U, 0.0f, 0.0f, 0.0f, covariance));

  std::vector<RirTrackMeasurement> measurements;
  measurements.push_back(MakeMeasurement(0U, 0.1f, 0.0f, 0.0f));
  measurements.push_back(MakeMeasurement(1U, 0.2f, 0.0f, 0.0f));
  measurements.push_back(MakeMeasurement(2U, 50.0f, 0.0f, 0.0f));

  const auto result = associator.Associate(measurements, seeds, 0.0f);

  ASSERT_EQ(result.measurements.size(), 3U);
  ASSERT_EQ(result.matches.size(), 1U);
  EXPECT_EQ(result.matches[0].association_key, 5U);
  // 哑行吸收的两个量测按输入顺序分配单调新键。
  EXPECT_EQ(result.measurements[1].association_key, 1U);
  EXPECT_FALSE(result.measurements[1].matched_existing_track);
  EXPECT_EQ(result.measurements[2].association_key, 2U);
  EXPECT_FALSE(result.measurements[2].matched_existing_track);
  EXPECT_TRUE(result.missed_track_keys.empty());
}

/// @brief LAPJV 求解器（N1）直接契约：非方阵拒绝；2×2 方阵给出全局最优行→列映射。
TEST(RirLapjvSolverTest, SolvesSquareMatrixAndRejectsNonSquare) {
  tracking::RirLapjvSolver solver;

  Eigen::MatrixXf non_square(2, 3);
  non_square.setZero();
  EXPECT_TRUE(solver.Solve(non_square).empty());

  Eigen::MatrixXf cost(2, 2);
  // 贪心取全局最小边 (0,0)=1.0 会把行 1 逼到 100.0（总 101.0）；
  // 全局最优是 (0,1)=1.1 + (1,0)=1.05（总 2.15）。
  cost << 1.0f, 1.1f, 1.05f, 100.0f;
  const std::vector<int> assignment = solver.Solve(cost);
  ASSERT_EQ(assignment.size(), 2U);
  EXPECT_EQ(assignment[0], 1);
  EXPECT_EQ(assignment[1], 0);
}

/// @brief 非有限量测被剔除，不消耗新键。
TEST(RirTrackAssociatorTest, NonFiniteMeasurementIsDropped) {
  RirTrackAssociator associator;
  std::vector<RirTrackMeasurement> measurements;
  measurements.push_back(MakeMeasurement(0U, 1.0f, 0.0f, 0.0f));
  RirTrackMeasurement invalid = MakeMeasurement(1U, 0.0f, 0.0f, 0.0f);
  invalid.position.x() = std::numeric_limits<float>::infinity();
  measurements.push_back(invalid);

  const auto result = associator.Associate(measurements, {}, 1.0f);

  ASSERT_EQ(result.measurements.size(), 1U);
  EXPECT_EQ(result.measurements[0].association_key, 1U);
}

/// @brief 运行态捕获/恢复：next_key 精确回滚。
TEST(RirTrackAssociatorTest, RuntimeStateRoundTrip) {
  RirTrackAssociator source;
  source.Associate({MakeMeasurement(0U, 0.0f, 0.0f, 0.0f)}, {}, 1.0f);
  source.Associate({MakeMeasurement(1U, 1.0f, 0.0f, 0.0f)}, {}, 1.0f);
  const auto state = source.CaptureRuntimeState();

  RirTrackAssociator restored;
  restored.RestoreRuntimeState(state);
  const auto result = restored.Associate({MakeMeasurement(2U, 2.0f, 0.0f, 0.0f)}, {}, 1.0f);
  ASSERT_EQ(result.measurements.size(), 1U);
  EXPECT_EQ(result.measurements[0].association_key, 3U);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
