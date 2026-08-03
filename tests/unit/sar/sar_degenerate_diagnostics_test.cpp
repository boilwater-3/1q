// 守护 SAR 退化诊断可见性契约（见 post-mortem）：
//   D1: 采样窗口装不下脉冲宽度 → config 层 abort（sample_window_too_small_for_pulse）
//   D2: 回波 clipping → kWarning（从淹没的 kInfo 浮出）
//   D3: 斜距与标称值严重错配 → 逐目标 kWarning（slant_range_mismatch）
//   D4: 聚焦图像全零峰值 → kError abort（degenerate_image_peak）
//
// 历史背景：SarDiagnosticSeverity::kWarning 虽已定义但全代码库从未被产生过；
// clipping 只产 kInfo；peak_power<=0 时 SNR 返回 -inf 反而绕过低 SNR 门控
// （isfinite(-inf)=false 短路）。这些测试守护退化不再静默通过。

#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "1q/sar/config/SarSessionConfig.h"
#include "1q/sar/session/SarCycleResult.h"
#include "1q/sar/session/SarSession.h"
#include "sar/session/SarRuntimeConfigValidation.h"

namespace sar {
namespace {

// 与 sar_session_pipeline_test.cpp::MakeSmallRdaConfig 对齐的合法基线：
// waveform_samples = ceil(0.16e-6 * 100e6) = 16 <= 64，斜距 ~30m 匹配 nominal 29.98m。
config::SarSessionConfig MakeBaselineRdaConfig() {
  config::SarSessionConfig config;
  config.hardware.carrier_frequency_hz = 1.0e9;
  config.hardware.bandwidth_hz = 25.0e6;
  config.hardware.pulse_width_s = 0.16e-6;
  config.hardware.pulse_repetition_frequency_hz = 20.0;
  config.hardware.sample_rate_hz = 100.0e6;
  config.mission.nominal_slant_range_m = 29.9792458;
  config.mission.scene_center_latitude_deg =
      29.9792458 / 6378137.0 * 180.0 / 3.14159265358979323846;
  config.mission.platform_speed_mps = 2.0;
  config.mission.range_sample_count = 64U;
  config.mission.azimuth_pulse_count = 9U;
  config.policy.enable_l1_rda_imaging = true;
  return config;
}

// 平台在场景中心上空，目标在 ~30m 地面偏移处 → 斜距 ~30m 匹配 nominal。
session::SarCycleInput MakeMatchingGeometryInput(std::uint32_t cycle_index = 1U) {
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kEarthRadiusM = 6378137.0;
  session::SarCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = 0.1f;
  input.platform.latitude_deg = 0.0;
  input.platform.longitude_deg = 0.0;
  input.platform.altitude_m = 0.0;
  session::SarPointTarget target;
  target.latitude_deg = 29.9792458 / kEarthRadiusM * 180.0 / kPi;
  target.longitude_deg = 0.0;
  target.altitude_m = 0.0;
  target.radar_cross_section_dbsm = 80.0;
  input.point_targets.push_back(target);
  return input;
}

bool HasDiagnosticWithSeverity(const session::SarCycleResult& result,
                               session::SarDiagnosticSeverity severity,
                               const std::string& code_prefix) {
  for (const session::SarDiagnosticIssue& issue : result.diagnostics) {
    if (issue.severity == severity &&
        issue.code.find(code_prefix) != std::string::npos) {
      return true;
    }
  }
  return false;
}

bool HasAbortReason(const session::SarCycleResult& result, const std::string& reason) {
  return result.has_error && result.abort_reason == reason;
}

// =========================================================================
// D1: 采样窗口过小 → config 层 abort
// =========================================================================
TEST(SarDegenerateDiagnosticsTest, SampleWindowTooSmallForPulseAbortsAtConfigLayer) {
  config::SarSessionConfig config = MakeBaselineRdaConfig();
  // 让波形样本数超过 range_sample_count：ceil(pulse_width * sample_rate) = ceil(1e-6*100e6)=100 > 64
  config.hardware.pulse_width_s = 1.0e-6;

  session::SarCycleResult result;
  const bool ok = session::ValidateRuntimeConfigForStep(config, /*has_external_raw_iq=*/false,
                                                       &result);

  EXPECT_FALSE(ok);
  EXPECT_TRUE(HasAbortReason(result, "sample_window_too_small_for_pulse"));
}

TEST(SarDegenerateDiagnosticsTest, SampleWindowExactlyFitsPulseDoesNotAbort) {
  config::SarSessionConfig config = MakeBaselineRdaConfig();
  // 波形样本数恰好等于窗口：ceil(0.64e-6 * 100e6) = 64 == 64 → 不应 abort
  config.hardware.pulse_width_s = 0.64e-6;

  session::SarCycleResult result;
  const bool ok = session::ValidateRuntimeConfigForStep(config, /*has_external_raw_iq=*/false,
                                                       &result);

  EXPECT_TRUE(ok);
  EXPECT_FALSE(HasAbortReason(result, "sample_window_too_small_for_pulse"));
}

TEST(SarDegenerateDiagnosticsTest, SampleWindowTooSmallRejectedByCreateWithDiagnostics) {
  config::SarSessionConfig config = MakeBaselineRdaConfig();
  config.hardware.pulse_width_s = 1.0e-6;  // 波形 100 样本 > 64 窗口

  config::ValidationIssueList issues;
  (void)session::SarSession::CreateWithDiagnostics(config, &issues);

  bool found = false;
  for (const config::ConfigValidationIssue& issue : issues) {
    if (issue.code == config::ConfigValidationCode::kSampleWindowTooSmallForPulse) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found);
}

// =========================================================================
// D2: 回波 clipping → kWarning
// =========================================================================
TEST(SarDegenerateDiagnosticsTest, RawEchoClippingProducesWarning) {
  config::SarSessionConfig config = MakeBaselineRdaConfig();
  // 把目标斜距推到足够远，使回波尾部超出 64 样本窗口但起点仍在窗内（触发 clip 而非完全丢弃）。
  // 回波起点 delay_sample = round(2*range/c * sample_rate)。要让波形尾部（+16 样本）超出 64：
  // delay + 16 > 64 → delay > 48 → range > 48/2 * c/sample_rate = 24 * 3e8/1e8 ≈ 72m。
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kEarthRadiusM = 6378137.0;
  constexpr double kFarSlantRangeM = 80.0;
  session::SarCycleInput input = MakeMatchingGeometryInput();
  input.point_targets[0].latitude_deg = kFarSlantRangeM / kEarthRadiusM * 180.0 / kPi;

  session::SarSession session = session::SarSession::Create(config);
  const session::SarCycleResult result = session.StepWithResult(input);

  // clip 是 warning 不阻断，周期仍应执行。
  EXPECT_FALSE(HasAbortReason(result, "sample_window_too_small_for_pulse"));
  EXPECT_TRUE(HasDiagnosticWithSeverity(result, session::SarDiagnosticSeverity::kWarning,
                                        "sar.raw_echo_clipping"));
}

// =========================================================================
// D3: 斜距与标称值严重错配 → 逐目标 kWarning
// =========================================================================
TEST(SarDegenerateDiagnosticsTest, SlantRangeMismatchProducesPerTargetWarning) {
  config::SarSessionConfig config = MakeBaselineRdaConfig();
  // 目标与平台同点 → 实际斜距 ≈ 0，远小于 nominal 29.98m（相对偏差 >> 20%）。
  session::SarCycleInput input = MakeMatchingGeometryInput();
  input.point_targets[0].latitude_deg = 0.0;   // 与平台同纬度
  input.point_targets[0].longitude_deg = 0.0;  // 与平台同经度
  input.point_targets[0].altitude_m = 0.0;     // 与平台同高度

  session::SarSession session = session::SarSession::Create(config);
  const session::SarCycleResult result = session.StepWithResult(input);

  EXPECT_TRUE(HasDiagnosticWithSeverity(result, session::SarDiagnosticSeverity::kWarning,
                                        "sar.slant_range_mismatch"));
}

TEST(SarDegenerateDiagnosticsTest, MatchingSlantRangeDoesNotProduceMismatchWarning) {
  // 基线几何斜距 ~30m 匹配 nominal 29.98m（偏差 < 20%）→ 不应产生 mismatch warning。
  session::SarSession session = session::SarSession::Create(MakeBaselineRdaConfig());
  const session::SarCycleResult result = session.StepWithResult(MakeMatchingGeometryInput());

  EXPECT_FALSE(HasDiagnosticWithSeverity(result, session::SarDiagnosticSeverity::kWarning,
                                         "sar.slant_range_mismatch"));
}

// =========================================================================
// D4: 聚焦图像全零峰值 → kError abort
// =========================================================================
TEST(SarDegenerateDiagnosticsTest, DegenerateImagePeakAbortsCycle) {
  config::SarSessionConfig config = MakeBaselineRdaConfig();
  // external raw IQ 不注入内部接收机热噪声；全零完整孔径因此稳定产生全黑图。
  session::SarCycleInput input = MakeMatchingGeometryInput();
  input.raw_iq.samples_per_pulse = config.mission.range_sample_count;
  input.raw_iq.i_values.assign(
      static_cast<std::size_t>(config.mission.azimuth_pulse_count) * config.mission.range_sample_count, 0.0);
  input.raw_iq.q_values.assign(
      static_cast<std::size_t>(config.mission.azimuth_pulse_count) * config.mission.range_sample_count, 0.0);

  session::SarSession session = session::SarSession::Create(config);
  const session::SarCycleResult result = session.StepWithResult(input);

  // 全黑图应触发 degenerate_image_peak abort。
  EXPECT_TRUE(HasAbortReason(result, "degenerate_image_peak"));
  EXPECT_FALSE(result.executed_this_cycle);
}

TEST(SarDegenerateDiagnosticsTest, HealthyImageDoesNotTripDegeneratePeakGate) {
  // 基线合法几何应成功聚焦，不触发 degenerate_image_peak。
  session::SarSession session = session::SarSession::Create(MakeBaselineRdaConfig());
  const session::SarCycleResult result = session.StepWithResult(MakeMatchingGeometryInput());

  EXPECT_FALSE(HasAbortReason(result, "degenerate_image_peak"));
  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_TRUE(result.output_frame.has_l1_image);
}

}  // namespace
}  // namespace sar
