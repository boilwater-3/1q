// 极化散射矩阵统计提取器：单行黄金值、双行算术统计、圆统计缠绕反例、fail-closed。

#include "remote_identification_radar/recognition/PolarizationStatsExtractor.h"

#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"

namespace remote_identification_radar {
namespace recognition {
namespace {

/** 幅度 dBsm + 相位 deg 全字段行（其余字段 0）。 */
session::RirPolSMatrixSample MakeRow(float hh_db, float hv_db, float vh_db, float vv_db,
                                     float vv_phase_deg, float hv_phase_deg) {
  session::RirPolSMatrixSample row;
  row.aspect_az_deg = 0.0f;
  row.aspect_el_deg = 10.0f;
  row.hh_amp_db = hh_db;
  row.hv_amp_db = hv_db;
  row.vh_amp_db = vh_db;
  row.hv_phase_deg = hv_phase_deg;
  row.vv_amp_db = vv_db;
  row.vv_phase_deg = vv_phase_deg;
  return row;
}

}  // namespace

TEST(RirPolarizationStatsTest, SingleDiagonalRowGoldenValues) {
  // HH=10 dBsm（σ=10）、VV=0 dBsm（σ=1）、交叉 −200 dBsm（数值零）：
  // 对角占优 → ψ=0、τ=0；span=11、|det|=√10、去极化≈0。
  const std::vector<session::RirPolSMatrixSample> window = {
      MakeRow(10.0f, -200.0f, -200.0f, 0.0f, 0.0f, 0.0f)};
  const RirPolarizationStatsObservation stats = RirPolarizationStatsExtractor::Extract(window);
  ASSERT_TRUE(stats.valid);
  EXPECT_NEAR(stats.span.mean, 11.0, 1.0e-12);
  EXPECT_NEAR(stats.determinant.mean, std::sqrt(10.0), 1.0e-12);
  EXPECT_NEAR(stats.depolarization.mean, 0.0, 1.0e-15);
  EXPECT_NEAR(stats.psi_deg.mean, 0.0, 1.0e-12);
  EXPECT_NEAR(stats.tau_deg.mean, 0.0, 1.0e-12);
  EXPECT_DOUBLE_EQ(stats.span.std, 0.0);
  EXPECT_DOUBLE_EQ(stats.psi_deg.std, 0.0);
}

TEST(RirPolarizationStatsTest, SwappedDiagonalPicksPsiNinety) {
  // VV 强于 HH（b>a 且非对角为零）→ ψ=90°。
  const std::vector<session::RirPolSMatrixSample> window = {
      MakeRow(0.0f, -200.0f, -200.0f, 10.0f, 0.0f, 0.0f)};
  const RirPolarizationStatsObservation stats = RirPolarizationStatsExtractor::Extract(window);
  ASSERT_TRUE(stats.valid);
  // 交叉残余 1e-20 走 ρ 数值路径：ψ=90−2.7e-9°（物理等价 90°）。
  EXPECT_NEAR(stats.psi_deg.mean, 90.0, 1.0e-6);
}

TEST(RirPolarizationStatsTest, EqualFourChannelRowGoldenValues) {
  // 四路同强（σ=1、相位 0）：span=4、去极化=0.5、|det|=|1-1|=0；
  // Graves 非对角 c=2 → ρ=1 → ψ=45°、τ=0。
  const std::vector<session::RirPolSMatrixSample> window = {
      MakeRow(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f)};
  const RirPolarizationStatsObservation stats = RirPolarizationStatsExtractor::Extract(window);
  ASSERT_TRUE(stats.valid);
  EXPECT_NEAR(stats.span.mean, 4.0, 1.0e-12);
  EXPECT_NEAR(stats.depolarization.mean, 0.5, 1.0e-12);
  EXPECT_NEAR(stats.determinant.mean, 0.0, 1.0e-15);
  EXPECT_NEAR(stats.psi_deg.mean, 45.0, 1.0e-12);
  EXPECT_NEAR(stats.tau_deg.mean, 0.0, 1.0e-12);
}

TEST(RirPolarizationStatsTest, TwoRowArithmeticMeanAndSampleStd) {
  // 对角行（|det|=√10、span=11、去极化 0）+ 全同行（|det|=0、span=4、去极化 0.5）：
  // 均值=中点、样本标准差=半差（n=2）。
  const std::vector<session::RirPolSMatrixSample> window = {
      MakeRow(10.0f, -200.0f, -200.0f, 0.0f, 0.0f, 0.0f),
      MakeRow(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f)};
  const RirPolarizationStatsObservation stats = RirPolarizationStatsExtractor::Extract(window);
  ASSERT_TRUE(stats.valid);
  // n=2 样本标准差（分母 n−1=1）＝半差·√2：7·√2/2、√10·√2/2、0.5·√2/2。
  EXPECT_NEAR(stats.span.mean, 7.5, 1.0e-12);
  EXPECT_NEAR(stats.span.std, 7.0 / std::sqrt(2.0), 1.0e-12);
  EXPECT_NEAR(stats.determinant.mean, std::sqrt(10.0) / 2.0, 1.0e-12);
  EXPECT_NEAR(stats.determinant.std, std::sqrt(5.0), 1.0e-12);
  EXPECT_NEAR(stats.depolarization.mean, 0.25, 1.0e-12);
  EXPECT_NEAR(stats.depolarization.std, 0.5 / std::sqrt(2.0), 1.0e-12);
}

TEST(RirPolarizationStatsTest, CircularStatsWraparoundGoldenValues) {
  // 缠绕反例（Stage A 探针转正）：+89° 与 −89° 物理几乎同向（差 2°），
  // 算术平均会假得 0°；圆统计均值=90°、散布≈1°。
  const RirPolarizationQuantityStats near_wrap =
      CircularMeanStdDeg({89.0, -89.0});
  EXPECT_NEAR(std::fabs(near_wrap.mean), 90.0, 1.0e-9);
  EXPECT_NEAR(near_wrap.std, 1.0002, 1.0e-3);

  // ±60° 均值=90°、散布=½·√(−2·ln 0.5)·180/π ≈ 33.731°。
  const RirPolarizationQuantityStats spread =
      CircularMeanStdDeg({60.0, -60.0});
  EXPECT_NEAR(std::fabs(spread.mean), 90.0, 1.0e-9);
  EXPECT_NEAR(spread.std, 33.7309, 1.0e-2);

  // 完全同向：散布 0、均值即样本值。
  const RirPolarizationQuantityStats aligned = CircularMeanStdDeg({30.0, 30.0});
  EXPECT_NEAR(aligned.mean, 30.0, 1.0e-12);
  EXPECT_DOUBLE_EQ(aligned.std, 0.0);
}

TEST(RirPolarizationStatsTest, InvalidRowsAreSkippedAndEmptyWindowInvalid) {
  // 非有限行（相位 NaN）被整行跳过；仅剩一行有效 → std=0。
  std::vector<session::RirPolSMatrixSample> window = {
      MakeRow(0.0f, -200.0f, -200.0f, 0.0f, 0.0f, 0.0f),
      MakeRow(10.0f, -200.0f, -200.0f, 0.0f, std::numeric_limits<float>::quiet_NaN(), 0.0f)};
  RirPolarizationStatsObservation stats = RirPolarizationStatsExtractor::Extract(window);
  ASSERT_TRUE(stats.valid);
  EXPECT_NEAR(stats.span.mean, 2.0, 1.0e-12);
  EXPECT_DOUBLE_EQ(stats.span.std, 0.0);

  // 空窗口 / 全非法窗口：valid=false（fail-closed，不冒充）。
  stats = RirPolarizationStatsExtractor::Extract({});
  EXPECT_FALSE(stats.valid);
  const std::vector<session::RirPolSMatrixSample> garbage = {
      MakeRow(std::numeric_limits<float>::infinity(), 0.0f, 0.0f, 0.0f, 0.0f, 0.0f)};
  EXPECT_FALSE(RirPolarizationStatsExtractor::Extract(garbage).valid);
}

}  // namespace recognition
}  // namespace remote_identification_radar
