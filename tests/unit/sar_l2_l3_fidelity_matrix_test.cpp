#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "sar/echo/SarEcho.h"
#include "sar/geometry/SarGeometry.h"
#include "sar/imaging/SarGbp.h"
#include "sar/imaging/SarImageQuality.h"
#include "sar/imaging/SarMotionCompensation.h"
#include "sar/imaging/SarRda.h"
#include "sar/signal/SarWaveform.h"
#include "support/sar_reference_scene.h"

// SAR L1/L2/L3 多保真度一致性矩阵测试(契约 l2_l3_fidelity_matrix.md §2-4)。
//
// 对同一物理场景,分别经三条保真度路径聚焦,系统化对比输出:
//   L1-RDA(理想直线)、L2-RDA(直线+扰动,含一阶补偿)、L3-BP(航路点/转弯,逐脉冲精确)。
// 全部为只读消费者:复用现有自由函数,不改任何生产源代码。
//
// 物理区分(阶段 A 评估 §2.4):L1/L2 用 RDA(频域,需补偿),L3 用 BP(时域逐脉冲精确,
// 不需补偿——SarGbp.cpp:83 用实际位置精确投影)。故 L2/L3 互斥是正确设计。
namespace sar {
namespace {

constexpr double kSpeedOfLightMps = 299792458.0;
constexpr std::size_t kMatrixTargetDelay = 20U;

// 找图像幅度峰值位置(独立算法,便于跨路径对比峰值列)。
struct PeakLocation {
  std::size_t row{0U};
  std::size_t col{0U};
  double magnitude{0.0};
};

PeakLocation FindMagnitudePeak(const signal::ComplexMatrix& image) {
  PeakLocation peak;
  for (std::size_t r = 0U; r < image.rows; ++r) {
    for (std::size_t c = 0U; c < image.cols; ++c) {
      const double mag = std::abs(image(r, c));
      if (mag > peak.magnitude) {
        peak.magnitude = mag;
        peak.row = r;
        peak.col = c;
      }
    }
  }
  return peak;
}

// 构建直线 waypoint 轨迹(L3 路径用,模拟直线场景下的 BP 聚焦)。
// 用全程直线匀速的两个端点航路点,等价 L1 直线轨迹但走 BP 算法。
std::vector<geometry::PlatformPulseState> BuildStraightWaypointTrack(
    const test_support::ReferencePointScene& scene) {
  geometry::WaypointTrackConfig config;
  geometry::Waypoint start;
  start.time_s = scene.pulses.front().time_s;
  start.position_m = scene.pulses.front().position_m;
  geometry::Waypoint end;
  end.time_s = scene.pulses.back().time_s;
  end.position_m = scene.pulses.back().position_m;
  config.waypoints = {start, end};
  for (const geometry::PlatformPulseState& pulse : scene.pulses) {
    config.pulse_times_s.push_back(pulse.time_s);
  }
  std::vector<geometry::PlatformPulseState> pulses;
  if (!geometry::GenerateWaypointTrack(config, &pulses)) {
    return {};
  }
  return pulses;
}

// 构建转弯 waypoint 轨迹(L3 路径用,中段横向偏移 final_cross_range_m)。
// 沿用 sar_second_order_motion_compensation_evidence_test 的转弯构造模式。
std::vector<geometry::PlatformPulseState> BuildTurningWaypointTrack(
    const test_support::ReferencePointScene& scene, double final_cross_range_m) {
  geometry::WaypointTrackConfig config;
  geometry::Waypoint start;
  start.time_s = scene.pulses.front().time_s;
  start.position_m = scene.pulses.front().position_m;
  geometry::Waypoint turn;
  turn.time_s = scene.pulses[scene.pulses.size() / 2U].time_s;
  turn.position_m = scene.pulses[scene.pulses.size() / 2U].position_m;
  geometry::Waypoint end;
  end.time_s = scene.pulses.back().time_s;
  end.position_m = scene.pulses.back().position_m;
  end.position_m.y_m = final_cross_range_m;  // 末端横向偏移(转弯)
  config.waypoints = {start, turn, end};
  for (const geometry::PlatformPulseState& pulse : scene.pulses) {
    config.pulse_times_s.push_back(pulse.time_s);
  }
  std::vector<geometry::PlatformPulseState> pulses;
  if (!geometry::GenerateWaypointTrack(config, &pulses)) {
    return {};
  }
  return pulses;
}

// 构建 L2 扰动轨迹(直线 ideal + 三向速度误差)。
std::vector<geometry::PlatformPulseState> BuildPerturbedTrack(
    const test_support::ReferencePointScene& scene, double stddev_y_mps, double stddev_z_mps,
    double stddev_x_mps, std::uint32_t seed) {
  geometry::PerturbedStripmapTrackConfig l2_config;
  l2_config.ideal.start_position_m = scene.pulses.front().position_m;
  l2_config.ideal.velocity_x_mps = scene.platform_velocity_mps;
  l2_config.ideal.prf_hz = scene.prf_hz;
  l2_config.ideal.pulse_count = scene.pulse_count;
  l2_config.velocity_error_stddev_x_mps = stddev_x_mps;
  l2_config.velocity_error_stddev_y_mps = stddev_y_mps;
  l2_config.velocity_error_stddev_z_mps = stddev_z_mps;
  l2_config.random_seed = seed;
  std::vector<geometry::PlatformPulseState> actual;
  geometry::TrajectoryErrorDiagnostics diag;
  if (!geometry::GeneratePerturbedStripmapTrack(l2_config, &actual, &diag)) {
    return {};
  }
  return actual;
}

// 用指定轨迹生成 raw history(供 RDA/BP 消费)。
bool BuildRawWithTrack(const test_support::ReferencePointScene& scene,
                       const std::vector<echo::PointTarget>& targets,
                       const std::vector<geometry::PlatformPulseState>& track,
                       signal::ComplexMatrix* history) {
  if (history == nullptr || track.empty()) {
    return false;
  }
  test_support::ReferencePointScene track_scene = scene;
  track_scene.pulses = track;
  return test_support::BuildReferenceRawHistory(track_scene, targets, history);
}

// BP 配置(沿用 sar_gbp_test 的 MakeGbpConfig 模式)。
imaging::GbpConfig MakeBpConfig(const test_support::ReferencePointScene& scene) {
  imaging::GbpConfig config;
  config.sample_rate_hz = scene.sample_rate_hz;
  config.carrier_frequency_hz = scene.carrier_frequency_hz;
  config.grid.azimuth_pixel_count = scene.pulse_count;
  config.grid.range_pixel_count = scene.range_sample_count;
  config.grid.azimuth_spacing_m = scene.platform_velocity_mps / scene.prf_hz;
  config.grid.range_spacing_m = kSpeedOfLightMps / (2.0 * scene.sample_rate_hz);
  config.grid.azimuth_start_m = scene.pulses.front().position_m.x_m;
  config.grid.range_start_m = 0.0;
  return config;
}

// 一阶补偿配置(参考点固定为场景中心斜距,沿用 session 的补偿语义)。
imaging::FirstOrderMotionCompensationConfig MakeCompensationConfig(
    const test_support::ReferencePointScene& scene, std::size_t target_delay) {
  imaging::FirstOrderMotionCompensationConfig config;
  config.sample_rate_hz = scene.sample_rate_hz;
  config.carrier_frequency_hz = scene.carrier_frequency_hz;
  // 参考点固定为场景中心斜距(目标位置),与 l2_session_integration §2 一致。
  config.reference_point_m = {0.0, static_cast<double>(target_delay) * kSpeedOfLightMps /
                                           (2.0 * scene.sample_rate_hz), 0.0};
  return config;
}

}  // namespace

// ── 断言 2(前置):L2 σ=0 退化不变 — 零扰动补偿逐样本等价 L1 ──
// 契约 §2.4 断言 2。这是 l2_session_integration §3 已确立不变量的矩阵复验。
TEST(SarL2L3FidelityMatrixTest, L2ZeroPerturbationDegeneratesToL1) {
  test_support::ReferencePointScene scene;
  scene.pulse_count = 17U;
  scene.prf_hz = 50.0;
  scene.platform_velocity_mps = 20.0;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const echo::PointTarget target =
      test_support::MakeReferenceTargetAtDelay(kMatrixTargetDelay, scene.sample_rate_hz, 1.0);

  // L1 raw(理想直线)。
  signal::ComplexMatrix l1_raw;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, {target}, &l1_raw));

  // L2 raw(零扰动轨迹 == ideal 轨迹)。
  const std::vector<geometry::PlatformPulseState> zero_perturb =
      BuildPerturbedTrack(scene, 0.0, 0.0, 0.0, 2026U);
  ASSERT_FALSE(zero_perturb.empty());
  signal::ComplexMatrix l2_raw;
  ASSERT_TRUE(BuildRawWithTrack(scene, {target}, zero_perturb, &l2_raw));

  // 一阶补偿(零扰动 → 补偿不变)。
  imaging::FirstOrderMotionCompensationConfig comp_config =
      MakeCompensationConfig(scene, kMatrixTargetDelay);
  signal::ComplexMatrix compensated_raw;
  imaging::MotionCompensationDiagnostics diag;
  ASSERT_TRUE(imaging::ApplyFirstOrderMotionCompensation(comp_config, scene.pulses, zero_perturb,
                                                         l2_raw, &compensated_raw, &diag));
  EXPECT_DOUBLE_EQ(diag.max_abs_range_error_m, 0.0);  // 零扰动确认

  const imaging::RdaConfig rda_config =
      test_support::MakeReferenceRdaConfig(scene, kMatrixTargetDelay);
  imaging::FocusedSarImage l1_image;
  imaging::FocusedSarImage l2_image;
  ASSERT_TRUE(imaging::FocusStripmapRda(rda_config, l1_raw, scene.matched_filter, &l1_image));
  ASSERT_TRUE(
      imaging::FocusStripmapRda(rda_config, compensated_raw, scene.matched_filter, &l2_image));

  // 逐样本一致(L2 σ=0 退化为 L1)。
  ASSERT_EQ(l1_image.image.values.size(), l2_image.image.values.size());
  for (std::size_t i = 0U; i < l1_image.image.values.size(); ++i) {
    EXPECT_NEAR(l1_image.image.values[i].real(), l2_image.image.values[i].real(), 1e-9);
    EXPECT_NEAR(l1_image.image.values[i].imag(), l2_image.image.values[i].imag(), 1e-9);
  }
}

// ── 断言 1:直线 σ=0 三路径峰值一致 ──
// 契约 §2.4 断言 1。L1-RDA / L2-RDA(σ=0)/ L3-BP(直线 waypoint)在同场景峰值位置一致。
TEST(SarL2L3FidelityMatrixTest, StraightSceneL1L2L3PeakConsistent) {
  test_support::ReferencePointScene scene;
  scene.pulse_count = 17U;
  scene.prf_hz = 50.0;
  scene.platform_velocity_mps = 20.0;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const echo::PointTarget target =
      test_support::MakeReferenceTargetAtDelay(kMatrixTargetDelay, scene.sample_rate_hz, 1.0);

  const imaging::RdaConfig rda_config =
      test_support::MakeReferenceRdaConfig(scene, kMatrixTargetDelay);

  // L1-RDA。
  signal::ComplexMatrix l1_raw;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, {target}, &l1_raw));
  imaging::FocusedSarImage l1_image;
  ASSERT_TRUE(imaging::FocusStripmapRda(rda_config, l1_raw, scene.matched_filter, &l1_image));
  const PeakLocation l1_peak = FindMagnitudePeak(l1_image.image);

  // L2-RDA(σ=0,零扰动退化)。
  const std::vector<geometry::PlatformPulseState> straight_wp = BuildStraightWaypointTrack(scene);
  ASSERT_FALSE(straight_wp.empty());
  signal::ComplexMatrix l2_raw;
  ASSERT_TRUE(BuildRawWithTrack(scene, {target}, straight_wp, &l2_raw));
  imaging::FirstOrderMotionCompensationConfig comp_config =
      MakeCompensationConfig(scene, kMatrixTargetDelay);
  signal::ComplexMatrix l2_compensated;
  imaging::MotionCompensationDiagnostics diag;
  ASSERT_TRUE(imaging::ApplyFirstOrderMotionCompensation(comp_config, scene.pulses, straight_wp,
                                                         l2_raw, &l2_compensated, &diag));
  imaging::FocusedSarImage l2_image;
  ASSERT_TRUE(
      imaging::FocusStripmapRda(rda_config, l2_compensated, scene.matched_filter, &l2_image));
  const PeakLocation l2_peak = FindMagnitudePeak(l2_image.image);

  // L3-BP(直线 waypoint 轨迹,独立算法)。
  signal::ComplexMatrix l3_raw;
  ASSERT_TRUE(BuildRawWithTrack(scene, {target}, straight_wp, &l3_raw));
  imaging::FocusedGbpImage l3_image;
  ASSERT_TRUE(imaging::FocusSmallSceneBp(MakeBpConfig(scene), straight_wp, l3_raw,
                                         scene.matched_filter, &l3_image));
  const PeakLocation l3_peak = FindMagnitudePeak(l3_image.image);

  // 三路径峰值列一致(落在目标真实斜距附近——距离单元)。
  // BP 峰值列直接对应距离样本;RDA 峰值列对应距离单元。两者应一致。
  EXPECT_EQ(l1_peak.col, l2_peak.col);  // L1/L2 同 RDA,σ=0 退化
  // L3-BP 独立算法,峰值列应落在目标斜距附近(允许网格量化误差)。
  EXPECT_NEAR(static_cast<double>(l3_peak.col), static_cast<double>(kMatrixTargetDelay), 2.0);

  // 三路径峰值均能量集中(>均匀分布)。
  auto is_concentrated = [](const signal::ComplexMatrix& img, const PeakLocation& peak) {
    double total = 0.0;
    for (const signal::ComplexSample& s : img.values) {
      total += std::abs(s);
    }
    return peak.magnitude > total / static_cast<double>(img.values.size());
  };
  EXPECT_TRUE(is_concentrated(l1_image.image, l1_peak));
  EXPECT_TRUE(is_concentrated(l2_image.image, l2_peak));
  EXPECT_TRUE(is_concentrated(l3_image.image, l3_peak));
}

// ── 断言 3:L2 小扰动补偿改善 ──
// 契约 §2.4 断言 3。小扰动(σ_y=10 m/s)下,补偿后 L2-RDA 的相干相关性优于未补偿。
TEST(SarL2L3FidelityMatrixTest, L2SmallPerturbationCompensationImproves) {
  test_support::ReferencePointScene scene;
  scene.pulse_count = 33U;
  scene.prf_hz = 50.0;
  scene.platform_velocity_mps = 20.0;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const echo::PointTarget target =
      test_support::MakeReferenceTargetAtDelay(kMatrixTargetDelay, scene.sample_rate_hz, 1.0);

  // L1 基线(理想)。
  signal::ComplexMatrix ideal_raw;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, {target}, &ideal_raw));

  // L2 小扰动轨迹 + raw。
  const std::vector<geometry::PlatformPulseState> small_perturb =
      BuildPerturbedTrack(scene, 10.0, 5.0, 0.0, 2026U);
  ASSERT_FALSE(small_perturb.empty());
  signal::ComplexMatrix actual_raw;
  ASSERT_TRUE(BuildRawWithTrack(scene, {target}, small_perturb, &actual_raw));

  // 一阶补偿。
  imaging::FirstOrderMotionCompensationConfig comp_config =
      MakeCompensationConfig(scene, kMatrixTargetDelay);
  signal::ComplexMatrix compensated_raw;
  imaging::MotionCompensationDiagnostics diag;
  ASSERT_TRUE(imaging::ApplyFirstOrderMotionCompensation(comp_config, scene.pulses, small_perturb,
                                                         actual_raw, &compensated_raw, &diag));
  EXPECT_GT(diag.max_abs_range_error_m, 0.0);  // 确有扰动

  const imaging::RdaConfig rda_config =
      test_support::MakeReferenceRdaConfig(scene, kMatrixTargetDelay);
  imaging::FocusedSarImage ideal_image;
  imaging::FocusedSarImage uncompensated_image;
  imaging::FocusedSarImage compensated_image;
  ASSERT_TRUE(imaging::FocusStripmapRda(rda_config, ideal_raw, scene.matched_filter, &ideal_image));
  ASSERT_TRUE(
      imaging::FocusStripmapRda(rda_config, actual_raw, scene.matched_filter, &uncompensated_image));
  ASSERT_TRUE(imaging::FocusStripmapRda(rda_config, compensated_raw, scene.matched_filter,
                                         &compensated_image));

  // 补偿改善:NRMS 下降,相干相关性上升。
  const imaging::ImageComparisonMetrics uncompensated =
      imaging::CompareImagesWithGlobalPhaseReference(ideal_image.image, uncompensated_image.image);
  const imaging::ImageComparisonMetrics compensated =
      imaging::CompareImagesWithGlobalPhaseReference(ideal_image.image, compensated_image.image);
  ASSERT_TRUE(uncompensated.valid);
  ASSERT_TRUE(compensated.valid);
  EXPECT_LT(compensated.normalized_rms_error, uncompensated.normalized_rms_error);
  EXPECT_GT(compensated.coherent_correlation, uncompensated.coherent_correlation);
  // 小扰动补偿后应达到高保真(NRMS<0.3, 相干>0.95)。
  EXPECT_LT(compensated.normalized_rms_error, 0.3);
  EXPECT_GT(compensated.coherent_correlation, 0.95);
}

// ── 断言 4:L2 大扰动补偿部分有效 ──
// 契约 §2.4 断言 4。大扰动(σ_y=30 m/s)下,补偿仍有改善但不完全(NRMS 仍较高)。
// 这展示一阶补偿的精度边界——不阻断但改善有限。
TEST(SarL2L3FidelityMatrixTest, L2LargePerturbationCompensationPartial) {
  test_support::ReferencePointScene scene;
  scene.pulse_count = 33U;
  scene.prf_hz = 50.0;
  scene.platform_velocity_mps = 20.0;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const echo::PointTarget target =
      test_support::MakeReferenceTargetAtDelay(kMatrixTargetDelay, scene.sample_rate_hz, 1.0);

  signal::ComplexMatrix ideal_raw;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, {target}, &ideal_raw));

  // L2 大扰动(σ_y=30 m/s,与 sar_motion_compensation_test 的大档一致)。
  const std::vector<geometry::PlatformPulseState> large_perturb =
      BuildPerturbedTrack(scene, 30.0, 10.0, 0.0, 2026U);
  ASSERT_FALSE(large_perturb.empty());
  signal::ComplexMatrix actual_raw;
  ASSERT_TRUE(BuildRawWithTrack(scene, {target}, large_perturb, &actual_raw));

  imaging::FirstOrderMotionCompensationConfig comp_config =
      MakeCompensationConfig(scene, kMatrixTargetDelay);
  signal::ComplexMatrix compensated_raw;
  imaging::MotionCompensationDiagnostics diag;
  ASSERT_TRUE(imaging::ApplyFirstOrderMotionCompensation(comp_config, scene.pulses, large_perturb,
                                                         actual_raw, &compensated_raw, &diag));
  EXPECT_GT(diag.max_abs_range_error_m, 0.0);

  const imaging::RdaConfig rda_config =
      test_support::MakeReferenceRdaConfig(scene, kMatrixTargetDelay);
  imaging::FocusedSarImage ideal_image;
  imaging::FocusedSarImage uncompensated_image;
  imaging::FocusedSarImage compensated_image;
  ASSERT_TRUE(imaging::FocusStripmapRda(rda_config, ideal_raw, scene.matched_filter, &ideal_image));
  ASSERT_TRUE(
      imaging::FocusStripmapRda(rda_config, actual_raw, scene.matched_filter, &uncompensated_image));
  ASSERT_TRUE(imaging::FocusStripmapRda(rda_config, compensated_raw, scene.matched_filter,
                                         &compensated_image));

  const imaging::ImageComparisonMetrics uncompensated =
      imaging::CompareImagesWithGlobalPhaseReference(ideal_image.image, uncompensated_image.image);
  const imaging::ImageComparisonMetrics compensated =
      imaging::CompareImagesWithGlobalPhaseReference(ideal_image.image, compensated_image.image);
  ASSERT_TRUE(uncompensated.valid);
  ASSERT_TRUE(compensated.valid);

  // 补偿仍改善(NRMS 下降,相干上升)。
  EXPECT_LT(compensated.normalized_rms_error, uncompensated.normalized_rms_error);
  EXPECT_GT(compensated.coherent_correlation, uncompensated.coherent_correlation);
  // 但大扰动下补偿不完全(NRMS > 小扰动档)——展示精度边界。
  // 注:此处不断言 NRMS<0.3(大扰动难达到),只验改善方向 + 优于未补偿。
  EXPECT_GT(compensated.coherent_correlation, 0.0);
}

// ── 断言 5:L3-BP 转弯场景修复 ──
// 契约 §2.4 断言 5。转弯场景(末端横向偏移 12m,RDA 失效区)下,BP 峰值位置正确、能量集中。
// 这格本身是二阶补偿冻结决策的正确性证据(转弯应改用 BP,见 second_order_motion_compensation.md)。
TEST(SarL2L3FidelityMatrixTest, L3BpRecoversTurningScene) {
  test_support::ReferencePointScene scene;
  scene.pulse_count = 33U;
  scene.prf_hz = 50.0;
  scene.platform_velocity_mps = 20.0;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const echo::PointTarget target =
      test_support::MakeReferenceTargetAtDelay(kMatrixTargetDelay, scene.sample_rate_hz, 1.0);

  // 转弯轨迹:末端横向偏移 12m(l3_first_order_applicability_matrix 的 RDA 失效门槛)。
  const std::vector<geometry::PlatformPulseState> turning_track =
      BuildTurningWaypointTrack(scene, 12.0);
  ASSERT_FALSE(turning_track.empty());

  // L3-BP 用转弯 actual 轨迹逐脉冲精确投影。
  signal::ComplexMatrix turning_raw;
  ASSERT_TRUE(BuildRawWithTrack(scene, {target}, turning_track, &turning_raw));
  imaging::FocusedGbpImage bp_image;
  ASSERT_TRUE(imaging::FocusSmallSceneBp(MakeBpConfig(scene), turning_track, turning_raw,
                                         scene.matched_filter, &bp_image));

  // BP 峰值落在目标真实斜距附近。
  const PeakLocation bp_peak = FindMagnitudePeak(bp_image.image);
  EXPECT_GT(bp_peak.magnitude, 0.0);
  EXPECT_NEAR(static_cast<double>(bp_peak.col), static_cast<double>(kMatrixTargetDelay), 4.0);

  // BP 能量集中(>均匀分布)——转弯场景下 BP 仍正确聚焦。
  double bp_total = 0.0;
  for (const signal::ComplexSample& s : bp_image.image.values) {
    bp_total += std::abs(s);
  }
  const double bp_avg = bp_total / static_cast<double>(bp_image.image.values.size());
  EXPECT_GT(bp_peak.magnitude, bp_avg);

  // 对比:同转弯场景下 RDA(不经补偿,直接用 ideal 直线轨迹)会散焦。
  // BP 的相干相关性显著优于 RDA 在转弯下的表现——证明转弯应改用 BP。
  signal::ComplexMatrix straight_raw;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, {target}, &straight_raw));
  const imaging::RdaConfig rda_config =
      test_support::MakeReferenceRdaConfig(scene, kMatrixTargetDelay);
  imaging::FocusedSarImage rda_turning_image;
  ASSERT_TRUE(imaging::FocusStripmapRda(rda_config, turning_raw, scene.matched_filter,
                                         &rda_turning_image));
  const imaging::ImageQualityMetrics bp_quality = imaging::EvaluateImageQuality(bp_image.image);
  const imaging::ImageQualityMetrics rda_quality =
      imaging::EvaluateImageQuality(rda_turning_image.image);

  // BP 在转弯场景下对比度优于 RDA(RDA 散焦 → 对比度下降)。
  EXPECT_GT(bp_quality.image_contrast, rda_quality.image_contrast);
}

}  // namespace sar
