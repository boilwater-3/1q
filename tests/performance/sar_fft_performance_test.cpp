#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <string>

#include "1q/sar/session/SarSessionFactory.h"
#include "sar/echo/SarEcho.h"
#include "sar/geometry/SarGeometry.h"
#include "sar/imaging/SarGbp.h"
#include "sar/imaging/SarRda.h"
#include "sar/signal/SarFft.h"
#include "sar/signal/SarWaveform.h"
#include "support/sar_reference_scene.h"

namespace sar {
namespace signal {
namespace {

TEST(SarPerformanceTest, FftFacadeCompletes1024SquareForwardTransform) {
  constexpr std::size_t kSize = 1024U;
  constexpr double kCurrentPlatformLimitSeconds = 10.0;

  ComplexMatrix input;
  input.rows = kSize;
  input.cols = kSize;
  input.values.assign(kSize * kSize, ComplexSample(0.0, 0.0));
  for (std::size_t row = 0U; row < kSize; ++row) {
    input(row, (row * 17U) % kSize) = ComplexSample(1.0, -0.5);
  }

  const auto start = std::chrono::steady_clock::now();
  ComplexMatrix range_spectrum;
  ASSERT_TRUE(FftRows(input, false, &range_spectrum));
  ComplexMatrix two_dimensional_spectrum;
  ASSERT_TRUE(FftCols(range_spectrum, false, &two_dimensional_spectrum));
  const double elapsed_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

  RecordProperty("matrix_rows", static_cast<int>(kSize));
  RecordProperty("matrix_cols", static_cast<int>(kSize));
  RecordProperty("elapsed_seconds", std::to_string(elapsed_seconds));
  ASSERT_EQ(two_dimensional_spectrum.values.size(), kSize * kSize);
  EXPECT_TRUE(std::isfinite(std::abs(two_dimensional_spectrum(0U, 0U))));
  EXPECT_LT(elapsed_seconds, kCurrentPlatformLimitSeconds);
}

TEST(SarPerformanceTest, RdaCompletes1024SquareSyntheticScene) {
  constexpr std::size_t kSize = 1024U;
  constexpr double kCurrentPlatformLimitSeconds = 30.0;

  ComplexMatrix raw_history;
  raw_history.rows = kSize;
  raw_history.cols = kSize;
  raw_history.values.assign(kSize * kSize, ComplexSample(0.0, 0.0));
  for (std::size_t row = 0U; row < kSize; ++row) {
    raw_history(row, kSize / 2U) = ComplexSample(1.0, 0.0);
  }

  imaging::RdaConfig config;
  config.sample_rate_hz = 100.0e6;
  config.carrier_frequency_hz = 1.0e9;
  config.prf_hz = 1000.0;
  config.platform_velocity_mps = 150.0;
  config.reference_range_m = 10000.0;
  config.rcmc_interpolation = imaging::RcmcInterpolation::kLinear;

  const ComplexVector identity_matched_filter{ComplexSample(1.0, 0.0)};
  const auto start = std::chrono::steady_clock::now();
  imaging::FocusedSarImage focused;
  ASSERT_TRUE(imaging::FocusStripmapRda(config, raw_history, identity_matched_filter, &focused));
  const double elapsed_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

  RecordProperty("matrix_rows", static_cast<int>(kSize));
  RecordProperty("matrix_cols", static_cast<int>(kSize));
  RecordProperty("elapsed_seconds", std::to_string(elapsed_seconds));
  EXPECT_EQ(focused.image.rows, kSize);
  EXPECT_EQ(focused.image.cols, kSize);
  EXPECT_GT(focused.diagnostics.azimuth_width_3db_bins, 0.0);
  EXPECT_TRUE(std::isfinite(focused.diagnostics.image_entropy_nats));
  EXPECT_LT(elapsed_seconds, kCurrentPlatformLimitSeconds);
}

TEST(SarPerformanceTest, SincRcmcCostIsMeasuredAt1024Square) {
  constexpr std::size_t kSize = 1024U;
  constexpr double kCurrentPlatformLimitSeconds = 10.0;

  ComplexMatrix input;
  input.rows = kSize;
  input.cols = kSize;
  input.values.resize(kSize * kSize);
  for (std::size_t row = 0U; row < kSize; ++row) {
    for (std::size_t col = 0U; col < kSize; ++col) {
      const double phase = 0.01 * static_cast<double>(row) + 0.1 * static_cast<double>(col);
      input(row, col) = ComplexSample(std::cos(phase), std::sin(phase));
    }
  }
  std::vector<double> shifts(kSize, 0.35);

  ComplexMatrix linear;
  std::size_t linear_out_of_bounds = 0U;
  const auto linear_start = std::chrono::steady_clock::now();
  ASSERT_TRUE(imaging::ApplyRangeMigrationCorrection(
      input, shifts, imaging::RcmcInterpolation::kLinear, 4U, &linear, &linear_out_of_bounds));
  const double linear_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - linear_start).count();

  ComplexMatrix sinc;
  std::size_t sinc_out_of_bounds = 0U;
  const auto sinc_start = std::chrono::steady_clock::now();
  ASSERT_TRUE(imaging::ApplyRangeMigrationCorrection(
      input, shifts, imaging::RcmcInterpolation::kSinc, 4U, &sinc, &sinc_out_of_bounds));
  const double sinc_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - sinc_start).count();

  RecordProperty("matrix_rows", static_cast<int>(kSize));
  RecordProperty("matrix_cols", static_cast<int>(kSize));
  RecordProperty("linear_seconds", std::to_string(linear_seconds));
  RecordProperty("sinc_seconds", std::to_string(sinc_seconds));
  EXPECT_EQ(linear_out_of_bounds, kSize);
  EXPECT_GT(sinc_out_of_bounds, 0U);
  EXPECT_TRUE(std::isfinite(std::abs(sinc(kSize / 2U, kSize / 2U))));
  EXPECT_LT(sinc_seconds, kCurrentPlatformLimitSeconds);
}

TEST(SarPerformanceTest, GbpCompletesApproved128SquareReferenceScene) {
  constexpr std::size_t kSize = 128U;
  constexpr double kCurrentPlatformLimitSeconds = 10.0;
  test_support::ReferencePointScene scene;
  scene.range_sample_count = kSize;
  scene.pulse_count = 128U;
  scene.prf_hz = 100.0;
  scene.platform_velocity_mps = 20.0;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::size_t target_delay = 64U;
  signal::ComplexMatrix raw_history;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(
      scene, {test_support::MakeReferenceTargetAtDelay(target_delay, scene.sample_rate_hz, 1.0)},
      &raw_history));

  imaging::GbpConfig config;
  config.sample_rate_hz = scene.sample_rate_hz;
  config.carrier_frequency_hz = scene.carrier_frequency_hz;
  config.grid.azimuth_pixel_count = kSize;
  config.grid.range_pixel_count = kSize;
  config.grid.azimuth_spacing_m = scene.platform_velocity_mps / scene.prf_hz;
  config.grid.range_spacing_m =
      test_support::kReferenceSpeedOfLightMps / (2.0 * scene.sample_rate_hz);
  config.grid.azimuth_start_m =
      -0.5 * static_cast<double>(kSize - 1U) * config.grid.azimuth_spacing_m;
  config.grid.range_start_m = 0.0;

  const auto start = std::chrono::steady_clock::now();
  imaging::FocusedGbpImage focused;
  ASSERT_TRUE(imaging::FocusSmallSceneGbp(config, scene.pulses, raw_history, scene.matched_filter,
                                          &focused));
  const double elapsed_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

  RecordProperty("matrix_rows", static_cast<int>(kSize));
  RecordProperty("matrix_cols", static_cast<int>(kSize));
  RecordProperty("elapsed_seconds", std::to_string(elapsed_seconds));
  EXPECT_EQ(focused.image.rows, kSize);
  EXPECT_EQ(focused.image.cols, kSize);
  EXPECT_EQ(focused.diagnostics.evaluated_pixels, kSize * kSize);
  EXPECT_LT(elapsed_seconds, kCurrentPlatformLimitSeconds);
}

TEST(SarPerformanceTest, PointTargetPipelineCompletes1024SquareScene) {
  constexpr std::size_t kSize = 1024U;
  constexpr double kSampleRateHz = 100.0e6;
  constexpr double kCarrierFrequencyHz = 1.0e9;
  constexpr double kPrfHz = 1000.0;
  constexpr double kPlatformVelocityMps = 150.0;
  constexpr double kSpeedOfLightMps = 299792458.0;
  constexpr double kCurrentPlatformLimitSeconds = 30.0;

  LfmWaveformConfig waveform_config;
  waveform_config.bandwidth_hz = 25.0e6;
  waveform_config.time_bandwidth_product = 4.0;
  waveform_config.sample_rate_hz = kSampleRateHz;
  LfmWaveform waveform;
  ASSERT_TRUE(GenerateLfmWaveform(waveform_config, &waveform));
  ComplexVector matched_filter;
  ASSERT_TRUE(BuildMatchedFilter(waveform.samples, &matched_filter));

  geometry::StraightStripmapTrackConfig track_config;
  track_config.start_position_m.x_m =
      -0.5 * static_cast<double>(kSize - 1U) * kPlatformVelocityMps / kPrfHz;
  track_config.velocity_x_mps = kPlatformVelocityMps;
  track_config.prf_hz = kPrfHz;
  track_config.pulse_count = static_cast<std::uint32_t>(kSize);
  std::vector<geometry::PlatformPulseState> pulses;
  ASSERT_TRUE(geometry::GenerateStraightStripmapTrack(track_config, &pulses));

  const double target_range_m =
      static_cast<double>(kSize / 2U) * kSpeedOfLightMps / (2.0 * kSampleRateHz);
  echo::PointTarget target;
  target.position_m.y_m = target_range_m;
  target.rcs_m2 = std::pow(target_range_m * target_range_m, 2.0);
  echo::RawEchoConfig echo_config;
  echo_config.sample_rate_hz = kSampleRateHz;
  echo_config.carrier_frequency_hz = kCarrierFrequencyHz;
  echo_config.range_sample_count = kSize;

  const auto start = std::chrono::steady_clock::now();
  ComplexMatrix raw_history;
  raw_history.rows = kSize;
  raw_history.cols = kSize;
  raw_history.values.assign(kSize * kSize, ComplexSample(0.0, 0.0));
  for (std::size_t row = 0U; row < kSize; ++row) {
    echo::RawEchoResult raw_echo;
    ASSERT_TRUE(echo::GeneratePointTargetRawEcho(echo_config, pulses[row], {target},
                                                 waveform.samples, &raw_echo));
    ASSERT_FALSE(raw_echo.has_clipping);
    for (std::size_t col = 0U; col < kSize; ++col) {
      raw_history(row, col) = raw_echo.samples[col];
    }
  }

  imaging::RdaConfig rda_config;
  rda_config.sample_rate_hz = kSampleRateHz;
  rda_config.carrier_frequency_hz = kCarrierFrequencyHz;
  rda_config.prf_hz = kPrfHz;
  rda_config.platform_velocity_mps = kPlatformVelocityMps;
  rda_config.reference_range_m = target_range_m;
  rda_config.rcmc_interpolation = imaging::RcmcInterpolation::kLinear;
  imaging::FocusedSarImage focused;
  ASSERT_TRUE(imaging::FocusStripmapRda(rda_config, raw_history, matched_filter, &focused));
  const double elapsed_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

  const std::size_t peak_index = imaging::FindPeakIndex(focused.image);
  RecordProperty("matrix_rows", static_cast<int>(kSize));
  RecordProperty("matrix_cols", static_cast<int>(kSize));
  RecordProperty("elapsed_seconds", std::to_string(elapsed_seconds));
  EXPECT_EQ(focused.image.rows, kSize);
  EXPECT_EQ(focused.image.cols, kSize);
  EXPECT_NEAR(static_cast<double>(peak_index % focused.image.cols), static_cast<double>(kSize / 2U),
              1.0);
  EXPECT_GT(focused.diagnostics.azimuth_width_3db_bins, 0.0);
  EXPECT_LT(elapsed_seconds, kCurrentPlatformLimitSeconds);
}

TEST(SarPerformanceTest, PublicSessionCompletes1024SquarePointTargetScene) {
  constexpr std::uint32_t kSize = 1024U;
  constexpr double kSampleRateHz = 100.0e6;
  constexpr double kSpeedOfLightMps = 299792458.0;
  constexpr double kCurrentPlatformLimitSeconds = 30.0;
  const double target_range_m =
      static_cast<double>(kSize / 2U) * kSpeedOfLightMps / (2.0 * kSampleRateHz);

  config::SarSessionConfig config;
  config.hardware.carrier_frequency_hz = 1.0e9;
  config.hardware.bandwidth_hz = 25.0e6;
  config.hardware.pulse_width_s = 0.16e-6;
  config.hardware.pulse_repetition_frequency_hz = 1000.0;
  config.hardware.sample_rate_hz = kSampleRateHz;
  config.mission.nominal_slant_range_m = target_range_m;
  config.mission.platform_speed_mps = 150.0;
  config.mission.range_sample_count = kSize;
  config.mission.azimuth_pulse_count = kSize;
  config.policy.enable_l1_rda_imaging = true;

  session::SarCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 0.1;
  session::SarPointTarget target;
  target.latitude_deg = target_range_m / 6378137.0 * 180.0 / 3.14159265358979323846;
  target.radar_cross_section_dbsm = 80.0;
  input.point_targets.push_back(target);

  session::SarSession sar_session = session::SarSessionFactory::Create(config);
  const auto start = std::chrono::steady_clock::now();
  const session::SarCycleResult result = sar_session.StepWithResult(input);
  const double elapsed_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

  RecordProperty("matrix_rows", static_cast<int>(kSize));
  RecordProperty("matrix_cols", static_cast<int>(kSize));
  RecordProperty("elapsed_seconds", std::to_string(elapsed_seconds));
  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_FALSE(result.has_error);
  EXPECT_TRUE(result.output_frame.has_l1_image);
  EXPECT_EQ(result.output_frame.range_sample_count, kSize);
  EXPECT_EQ(result.output_frame.azimuth_pulse_count, kSize);
  EXPECT_LT(elapsed_seconds, kCurrentPlatformLimitSeconds);
}

}  // namespace
}  // namespace signal
}  // namespace sar
