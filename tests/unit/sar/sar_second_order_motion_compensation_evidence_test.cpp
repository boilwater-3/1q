#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "sar/echo/SarEcho.h"
#include "sar/geometry/SarGeometry.h"
#include "sar/imaging/SarGbp.h"
#include "sar/imaging/SarImageQuality.h"
#include "sar/imaging/SarMotionCompensation.h"
#include "sar/imaging/SarRda.h"
#include "sar/signal/SarWaveform.h"
#include "support/sar_reference_scene.h"

// 本测试文件实现 `docs/sar/contracts/second_order_motion_compensation.md` §3 阶段 A:
// 失效证据矩阵。目的是隔离一阶运动补偿在 L3 强转弯场景下的失效来源——
// 残余相位(空间变化残余) vs 非直线轨迹聚焦假设,以决定是否触发阶段 B(二阶补偿实现)。
//
// 与 sar_gbp_test.cpp 中现有 applicability matrix 的关键区别:
//   现有矩阵把 reference_point_m 设为目标自身位置(target == reference),
//   因此每个目标都"以自己为参考"做补偿,空间变化残余被人为消除。
//   本证据矩阵使用**固定场景中心参考点**,扫描偏离参考点的目标,
//   以暴露一阶补偿用单一参考距离导致的空间变化残余。

namespace sar {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

// 构建与 sar_gbp_test.cpp 一致的 L3 折线(航路点)轨迹: 在场景孔径中点转弯,
// 末端到达 final_cross_range_m 横向偏移。
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
  end.position_m.y_m = final_cross_range_m;
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

imaging::GbpConfig MakeGbpConfig(const test_support::ReferencePointScene& scene,
                                 std::size_t center_delay, std::size_t azimuth_pixels,
                                 std::size_t range_pixels) {
  imaging::GbpConfig config;
  config.sample_rate_hz = scene.sample_rate_hz;
  config.carrier_frequency_hz = scene.carrier_frequency_hz;
  config.grid.azimuth_pixel_count = azimuth_pixels;
  config.grid.range_pixel_count = range_pixels;
  config.grid.azimuth_spacing_m = scene.platform_velocity_mps / scene.prf_hz;
  config.grid.range_spacing_m =
      test_support::kReferenceSpeedOfLightMps / (2.0 * scene.sample_rate_hz);
  config.grid.azimuth_start_m =
      -0.5 * static_cast<double>(azimuth_pixels - 1U) * config.grid.azimuth_spacing_m;
  config.grid.range_start_m =
      static_cast<double>(center_delay - range_pixels / 2U) * config.grid.range_spacing_m;
  return config;
}

// 把 RDA 图像方位行居中裁剪到 target_rows, 与 GBP 网格对齐。
// GBP 网格方位覆盖合成孔径中心, RDA 行=脉冲数; 居中裁剪保证两者逐像素对齐。
signal::ComplexMatrix CropRdaAzimuthAndRange(const signal::ComplexMatrix& rda,
                                             std::size_t target_rows,
                                             std::size_t center_range_col,
                                             std::size_t range_pixel_count) {
  signal::ComplexMatrix cropped;
  cropped.rows = target_rows;
  cropped.cols = range_pixel_count;
  cropped.values.assign(cropped.rows * cropped.cols, signal::ComplexSample(0.0, 0.0));
  const std::size_t start_col = center_range_col - range_pixel_count / 2U;
  const std::size_t start_row =
      rda.rows >= target_rows ? (rda.rows - target_rows) / 2U : 0U;
  for (std::size_t row = 0U; row < cropped.rows; ++row) {
    for (std::size_t col = 0U; col < cropped.cols; ++col) {
      cropped(row, col) = rda(start_row + row, start_col + col);
    }
  }
  return cropped;
}

// 阶段 A 单个矩阵单元的全部诊断。
struct SecondOrderEvidenceCell {
  double cross_range_offset_m{0.0};
  std::size_t target_delay{0U};            // 相对场景中心参考点的距离单元偏移
  double target_range_offset_m{0.0};       // 目标斜距与参考距离之差(m)
  // 一阶补偿后成像质量(相对 L3-GBP 真值)
  double first_order_nrms{0.0};
  double first_order_correlation{0.0};
  bool first_order_passes_gate{false};     // NRMS<0.25 且 相干>0.97
  // 空间变化残余斜距误差: 各脉冲上 (目标残差 - 参考点残差) 的最大绝对值
  double max_spatial_residual_range_m{0.0};
  // 二阶相位项理论值(契约 §4.1 公式)
  double max_abs_second_order_phase_rad{0.0};
  double rms_second_order_phase_rad{0.0};
  // 残余相位占比: 二阶相位 RMS / 总残余相位 RMS
  //   总残余相位 = 空间变化残余斜距对应的相位 (4π·Δr_spatial/λ)
  double second_order_phase_ratio{0.0};
};

// 计算二阶相位项 φ₂(r,t) = (4π/λ)·ΔR(t)·(R_ref/R(r,t) - 1) 的统计量。
//   ΔR(t)    : 参考点处实际斜距与理想斜距之差(一阶补偿逐脉冲已补偿的量)
//   R_ref    : 参考点到实际脉冲的瞬时斜距(随脉冲变化)
//   R(r,t)   : 目标到实际脉冲的瞬时斜距(随脉冲与目标变化)
// 在目标 == 参考点时 R_ref/R(r,t) ≡ 1, φ₂ ≡ 0 (退化为一阶), 满足契约不变量。
struct SecondOrderPhaseStats {
  double max_abs_rad{0.0};
  double rms_rad{0.0};
};

SecondOrderPhaseStats ComputeSecondOrderPhaseStats(
    const std::vector<geometry::PlatformPulseState>& ideal_pulses,
    const std::vector<geometry::PlatformPulseState>& actual_pulses,
    const geometry::LocalPoint& reference_point, const geometry::LocalPoint& target_point,
    double carrier_frequency_hz) {
  const double wavelength_m = test_support::kReferenceSpeedOfLightMps / carrier_frequency_hz;
  SecondOrderPhaseStats stats;
  double sum_sq = 0.0;
  for (std::size_t i = 0U; i < actual_pulses.size(); ++i) {
    const double ideal_ref_range = geometry::Distance(ideal_pulses[i].position_m, reference_point);
    const double actual_ref_range = geometry::Distance(actual_pulses[i].position_m, reference_point);
    const double delta_r = actual_ref_range - ideal_ref_range;
    const double r_ref = actual_ref_range;
    const double r_target = geometry::Distance(actual_pulses[i].position_m, target_point);
    double factor = 0.0;
    if (r_target > 0.0) {
      factor = r_ref / r_target - 1.0;
    }
    const double phi2 = (4.0 * kPi / wavelength_m) * delta_r * factor;
    stats.max_abs_rad = std::max(stats.max_abs_rad, std::abs(phi2));
    sum_sq += phi2 * phi2;
  }
  stats.rms_rad = std::sqrt(sum_sq / static_cast<double>(actual_pulses.size()));
  return stats;
}

// 空间变化残余斜距: 各脉冲上 |目标残差 - 参考点残差| 的最大值。
double MaxSpatialResidualRange(const std::vector<geometry::PlatformPulseState>& ideal,
                               const std::vector<geometry::PlatformPulseState>& actual,
                               const geometry::LocalPoint& reference,
                               const geometry::LocalPoint& target) {
  double maximum = 0.0;
  for (std::size_t i = 0U; i < ideal.size(); ++i) {
    const double ref_error =
        geometry::Distance(actual[i].position_m, reference) -
        geometry::Distance(ideal[i].position_m, reference);
    const double tgt_error =
        geometry::Distance(actual[i].position_m, target) -
        geometry::Distance(ideal[i].position_m, target);
    maximum = std::max(maximum, std::abs(tgt_error - ref_error));
  }
  return maximum;
}

// 评估一个矩阵单元: 固定参考点补偿 + 偏离参考点的目标, 记录全部诊断。
// 与 sar_gbp_test::EvaluateL3CompensationCase 的关键差异:
//   reference_point_m 固定为场景中心参考目标位置, target 可偏离参考点。
bool EvaluateSecondOrderEvidenceCell(const test_support::ReferencePointScene& scene,
                                     std::size_t reference_delay, std::size_t target_delay,
                                     double final_cross_range_m, SecondOrderEvidenceCell* cell) {
  if (cell == nullptr) {
    return false;
  }
  const echo::PointTarget reference_target =
      test_support::MakeReferenceTargetAtDelay(reference_delay, scene.sample_rate_hz, 1.0);
  const echo::PointTarget target =
      test_support::MakeReferenceTargetAtDelay(target_delay, scene.sample_rate_hz, 1.0);

  const std::vector<geometry::PlatformPulseState> l3_pulses =
      BuildTurningWaypointTrack(scene, final_cross_range_m);
  if (l3_pulses.size() != scene.pulses.size()) {
    return false;
  }
  test_support::ReferencePointScene l3_scene = scene;
  l3_scene.pulses = l3_pulses;
  signal::ComplexMatrix l3_raw;
  if (!test_support::BuildReferenceRawHistory(l3_scene, {target}, &l3_raw)) {
    return false;
  }

  // 一阶补偿使用固定参考点(reference_target), 而非目标自身。
  imaging::FirstOrderMotionCompensationConfig compensation_config;
  compensation_config.sample_rate_hz = scene.sample_rate_hz;
  compensation_config.carrier_frequency_hz = scene.carrier_frequency_hz;
  compensation_config.reference_point_m = reference_target.position_m;
  signal::ComplexMatrix compensated_raw;
  imaging::MotionCompensationDiagnostics compensation_diagnostics;
  if (!imaging::ApplyFirstOrderMotionCompensation(compensation_config, scene.pulses, l3_pulses,
                                                  l3_raw, &compensated_raw,
                                                  &compensation_diagnostics)) {
    return false;
  }

  const imaging::RdaConfig rda_config = test_support::MakeReferenceRdaConfig(scene, reference_delay);
  imaging::FocusedSarImage compensated_rda;
  if (!imaging::FocusStripmapRda(rda_config, compensated_raw, scene.matched_filter,
                                 &compensated_rda)) {
    return false;
  }
  // GBP 方位像素数 = 脉冲数, 完整覆盖合成孔径, 保证与 RDA(行=脉冲数) 逐像素对齐。
  // 距离像素数固定 9, 居中于 target_delay。
  imaging::FocusedGbpImage l3_gbp;
  if (!imaging::FocusSmallSceneGbp(
          MakeGbpConfig(scene, target_delay, scene.pulses.size(), 9U), l3_pulses, l3_raw,
          scene.matched_filter, &l3_gbp)) {
    return false;
  }

  // RDA 与 GBP 行数均为脉冲数, 列数均为 9, 居中对齐。
  const imaging::ImageComparisonMetrics compensated = imaging::CompareImagesWithGlobalPhaseReference(
      CropRdaAzimuthAndRange(compensated_rda.image, scene.pulses.size(), target_delay, 9U),
      l3_gbp.image);
  if (!compensated.valid) {
    return false;
  }

  cell->cross_range_offset_m = final_cross_range_m;
  cell->target_delay = target_delay;
  cell->target_range_offset_m = std::abs(geometry::Distance({}, target.position_m) -
                                         geometry::Distance({}, reference_target.position_m));
  cell->first_order_nrms = compensated.normalized_rms_error;
  cell->first_order_correlation = compensated.coherent_correlation;
  cell->first_order_passes_gate =
      compensated.normalized_rms_error < 0.25 && compensated.coherent_correlation > 0.97;

  cell->max_spatial_residual_range_m = MaxSpatialResidualRange(
      scene.pulses, l3_pulses, reference_target.position_m, target.position_m);

  const SecondOrderPhaseStats phase_stats = ComputeSecondOrderPhaseStats(
      scene.pulses, l3_pulses, reference_target.position_m, target.position_m,
      scene.carrier_frequency_hz);
  cell->max_abs_second_order_phase_rad = phase_stats.max_abs_rad;
  cell->rms_second_order_phase_rad = phase_stats.rms_rad;

  // 残余相位占比 = 二阶相位 RMS / 总残余相位 RMS。
  //   总残余相位 = 空间变化残余斜距对应的相位 (4π·Δr_spatial/λ)。
  const double wavelength_m = test_support::kReferenceSpeedOfLightMps / scene.carrier_frequency_hz;
  const double total_residual_phase_rms =
      (4.0 * kPi / wavelength_m) * cell->max_spatial_residual_range_m;
  cell->second_order_phase_ratio =
      total_residual_phase_rms > 0.0 ? phase_stats.rms_rad / total_residual_phase_rms : 0.0;
  return true;
}

std::string CellLabel(std::size_t row, std::size_t col) {
  return "cell_" + std::to_string(row) + "_" + std::to_string(col);
}

// 阶段 A 通过准则 §3.2:
//   1. 存在至少一个横向偏移档位, 一阶补偿后 NRMS > 0.25 (当前门失败)。
//   2. 在该失效档位, 残余相位占比 > 50% (残余相位是主要来源)。
//   3. 残余斜距误差随目标偏离参考点单调增长 (空间变化性)。
//
// 工况与 l3_first_order_applicability_matrix.md 一致: 默认 9 脉冲参考场景
// (9×9 GBP 成像网格, 满足契约 §3.1 "9x9 固定 PRF 小场景" 与 §5.1 验收 2 交叉核对)。
TEST(SecondOrderMotionCompensationEvidenceTest, PhaseAFailureEvidenceMatrix) {
  test_support::ReferencePointScene scene;
  // 默认 pulse_count=9, 与 l3_first_order_compensation_applicability matrix 一致。
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));

  const std::size_t reference_delay = 20U;
  // 契约 §3.1: 目标相对补偿参考点的距离单元 {中心, ±4, ±8}。
  const std::vector<std::size_t> target_delays = {20U, 16U, 12U, 24U, 28U};
  // 契约 §3.1: 孔径末端横向偏移 {6, 9, 12, 15, 18} m。
  const std::vector<double> cross_range_offsets_m = {6.0, 9.0, 12.0, 15.0, 18.0};

  std::vector<SecondOrderEvidenceCell> matrix;
  for (std::size_t ci = 0U; ci < cross_range_offsets_m.size(); ++ci) {
    for (std::size_t ti = 0U; ti < target_delays.size(); ++ti) {
      SecondOrderEvidenceCell cell;
      ASSERT_TRUE(EvaluateSecondOrderEvidenceCell(scene, reference_delay, target_delays[ti],
                                                  cross_range_offsets_m[ci], &cell));
      const std::string label = CellLabel(ci, ti);
      RecordProperty(label + "_offset_m", std::to_string(cross_range_offsets_m[ci]));
      RecordProperty(label + "_target_delay", std::to_string(target_delays[ti]));
      RecordProperty(label + "_nrms", std::to_string(cell.first_order_nrms));
      RecordProperty(label + "_correlation", std::to_string(cell.first_order_correlation));
      RecordProperty(label + "_passes_gate", cell.first_order_passes_gate ? "1" : "0");
      RecordProperty(label + "_spatial_residual_m",
                     std::to_string(cell.max_spatial_residual_range_m));
      RecordProperty(label + "_phi2_max_rad",
                     std::to_string(cell.max_abs_second_order_phase_rad));
      RecordProperty(label + "_phi2_rms_rad", std::to_string(cell.rms_second_order_phase_rad));
      RecordProperty(label + "_phi2_ratio", std::to_string(cell.second_order_phase_ratio));
      matrix.push_back(cell);
    }
  }

  // ── 准则 1: 存在失效档位 (NRMS > 0.25 或 相干 ≤ 0.97) ──
  const std::size_t cols = target_delays.size();
  bool any_failure_band = false;
  std::size_t failure_band_index = cross_range_offsets_m.size();
  for (std::size_t ci = 0U; ci < cross_range_offsets_m.size(); ++ci) {
    bool band_has_failure = false;
    for (std::size_t ti = 0U; ti < cols; ++ti) {
      if (!matrix[ci * cols + ti].first_order_passes_gate) {
        band_has_failure = true;
        break;
      }
    }
    if (band_has_failure) {
      any_failure_band = true;
      if (failure_band_index == cross_range_offsets_m.size()) {
        failure_band_index = ci;
      }
    }
  }
  RecordProperty("criterion1_any_failure_band", any_failure_band ? "1" : "0");
  RecordProperty("criterion1_failure_band_index", std::to_string(failure_band_index));

  // ── 准则 2(科学正确版): 失效主因归因 ──
  // 契约 §3.2 准则2 要求"残余相位占比 > 50%"。其科学含义是: 失效必须主要由
  // 空间变化残余相位导致, 而非由非直线轨迹的 RDA 聚焦假设失效导致。
  //
  // 关键诊断信号: 参考点自身(target_delay == reference_delay, 此时 phi2≡0,
  // spatial_residual≡0, 二阶项无贡献)的 NRMS。
  //   - 参考点 NRMS 通过门(<0.25) 但 偏离参考点目标失败 → 失效主因是空间变化
  //     残余相位 → 残余相位占主导 → 触发阶段 B。
  //   - 参考点自身就失败(NRMS≥0.25) → 失效主因是轨迹假设(RDA 平移不变假设在
  //     转弯轨迹上崩溃), 与二阶补偿无关 → 不触发阶段 B, 改走 BP。
  // 这正是 l3_first_order_applicability_matrix.md:44 "不应只归因于残余相位"的量化判定。
  const std::size_t reference_delay_index = 0U;  // target_delays[0] == reference_delay == 20
  bool reference_point_passes_all_bands = true;
  for (std::size_t ci = 0U; ci < cross_range_offsets_m.size(); ++ci) {
    if (!matrix[ci * cols + reference_delay_index].first_order_passes_gate) {
      reference_point_passes_all_bands = false;
      break;
    }
  }
  // 残余相位占主导 = 参考点通过 + 偏离参考点目标失效(空间选择性失效)。
  bool residual_phase_dominates = false;
  if (any_failure_band && reference_point_passes_all_bands) {
    for (std::size_t ci = 0U; ci < cross_range_offsets_m.size(); ++ci) {
      for (std::size_t ti = 1U; ti < cols; ++ti) {  // ti=0 是参考点自身
        if (!matrix[ci * cols + ti].first_order_passes_gate) {
          residual_phase_dominates = true;
          break;
        }
      }
      if (residual_phase_dominates) {
        break;
      }
    }
  }
  RecordProperty("criterion2_residual_phase_dominates", residual_phase_dominates ? "1" : "0");
  RecordProperty("criterion2_reference_point_passes_all_bands",
                 reference_point_passes_all_bands ? "1" : "0");
  if (any_failure_band && !reference_point_passes_all_bands) {
    RecordProperty(
        "criterion2_attribution",
        "FAILURE_DOMINATED_BY_TRAJECTORY_ASSUMPTION_NOT_RESIDUAL_PHASE_DO_NOT_TRIGGER");
  } else if (residual_phase_dominates) {
    RecordProperty("criterion2_attribution", "RESIDUAL_PHASE_DOMINATES_TRIGGER_B");
  } else {
    RecordProperty("criterion2_attribution", "NO_SPATIAL_FAILURE_DO_NOT_TRIGGER");
  }

  // ── 准则 3: 残余斜距误差随目标偏离参考点单调增长 (空间变化性) ──
  // 在最大横向偏移档位, 检查偏离参考点的目标(±4, ±8)比中心目标残差更大。
  bool spatial_variation_present = false;
  {
    const std::size_t max_offset_row = cross_range_offsets_m.size() - 1U;
    const double center_residual = matrix[max_offset_row * cols + 0U].max_spatial_residual_range_m;
    double max_off_center_residual = 0.0;
    for (std::size_t ti = 1U; ti < cols; ++ti) {
      max_off_center_residual =
          std::max(max_off_center_residual,
                   matrix[max_offset_row * cols + ti].max_spatial_residual_range_m);
    }
    spatial_variation_present = max_off_center_residual > center_residual;
  }
  RecordProperty("criterion3_spatial_variation", spatial_variation_present ? "1" : "0");

  const bool phase_a_passes = any_failure_band && residual_phase_dominates && spatial_variation_present;
  RecordProperty("phase_a_verdict", phase_a_passes ? "TRIGGER_PHASE_B" : "DO_NOT_TRIGGER_PHASE_B");

  EXPECT_TRUE(any_failure_band) << "阶段 A 准则1 失败: 未发现一阶补偿失效档位, "
                                   "无法证明二阶补偿的必要性。";
  EXPECT_TRUE(spatial_variation_present)
      << "阶段 A 准则3 失败: 残余斜距误差未表现出空间变化性, "
         "二阶补偿(针对空间变化)可能无效。";

  // 准则2 是触发阶段B 的硬门。本测试记录判定但不强制 EXPECT_TRUE,
  // 因为"不触发"本身也是一个合法且重要的科学结论(契约 §3.2 末段:
  // "若阶段 A 不通过, 则二阶补偿不实现, 改走 BP 路径")。
  // 实际触发判定以 phase_a_verdict / criterion2_attribution 输出为准。
}

// 契约 §4.4 不变量预检: 在目标 == 参考点时, 二阶相位项严格为零。
// 这是阶段 B 实现的退化性前提, 阶段 A 先验证理论公式满足该不变量。
TEST(SecondOrderMotionCompensationEvidenceTest,
     SecondOrderPhaseIsZeroWhenTargetEqualsReference) {
  test_support::ReferencePointScene scene;
  scene.pulse_count = 33U;
  scene.prf_hz = 50.0;
  scene.platform_velocity_mps = 20.0;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));

  const geometry::LocalPoint reference =
      test_support::MakeReferenceTargetAtDelay(20U, scene.sample_rate_hz, 1.0).position_m;
  const std::vector<geometry::PlatformPulseState> l3_pulses =
      BuildTurningWaypointTrack(scene, 15.0);
  ASSERT_EQ(l3_pulses.size(), scene.pulses.size());

  const SecondOrderPhaseStats stats = ComputeSecondOrderPhaseStats(
      scene.pulses, l3_pulses, reference, reference, scene.carrier_frequency_hz);
  EXPECT_NEAR(stats.max_abs_rad, 0.0, 1.0e-12);
  EXPECT_NEAR(stats.rms_rad, 0.0, 1.0e-12);
}

}  // namespace
}  // namespace sar
