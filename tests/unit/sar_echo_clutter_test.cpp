/**
 * @file sar_echo_clutter_test.cpp
 * @brief 噪声、杂波散射系数与面目标场景回波的单元测试。
 */

#include "sar/echo/SarEcho.h"
#include "sar/signal/SarWaveform.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

namespace sar {
namespace echo {
namespace {

signal::ComplexVector MakeSimplePulse(std::size_t length) {
  signal::ComplexVector v(length, signal::ComplexSample(1.0, 0.0));
  return v;
}

RawEchoConfig MakeConfig() {
  RawEchoConfig c;
  c.sample_rate_hz = 20.0e6;
  c.carrier_frequency_hz = 10.0e9;
  c.range_sample_count = 512U;
  return c;
}

geometry::PlatformPulseState MakePlatform() {
  geometry::PlatformPulseState p;
  p.pulse_id = 0U;
  p.time_s = 0.0;
  p.position_m = geometry::LocalPoint{0.0, 0.0, 5000.0};
  p.velocity_x_mps = 100.0;
  return p;
}

}  // namespace

// ── ApplyFractionalDelay ─────────────────────────────────────

TEST(SarEchoClutterTest, FractionalDelayRejectsNullOutput) {
  EXPECT_FALSE(ApplyFractionalDelay(MakeSimplePulse(8), 0.5, nullptr));
}

TEST(SarEchoClutterTest, FractionalDelayRejectsZeroDelay) {
  // fractional_delay == 0.0 is rejected (no-op should be handled by caller)
  signal::ComplexVector output;
  EXPECT_FALSE(ApplyFractionalDelay(MakeSimplePulse(8), 0.0, &output));
}

TEST(SarEchoClutterTest, FractionalDelayProducesSameLength) {
  signal::ComplexVector input(16, signal::ComplexSample(1.0, 0.0));
  signal::ComplexVector output;
  ASSERT_TRUE(ApplyFractionalDelay(input, 0.3, &output));
  EXPECT_EQ(output.size(), input.size());
}

// ── AddNoise ──────────────────────────────────────────────────

TEST(SarEchoClutterTest, AddNoiseIncreasesSamplePower) {
  signal::ComplexVector samples(256, signal::ComplexSample(0.1, 0.0));
  const double before = std::norm(samples[0]);

  NoiseSpec spec;
  spec.signal_to_noise_ratio_db = 10.0;  // SNR = 10 dB → 噪声功率 = 1/10 信号功率
  spec.random_seed = 2026U;
  ASSERT_TRUE(AddNoise(spec, &samples));

  const double after = std::norm(samples[0]);
  // 加噪后功率应不同
  EXPECT_NE(before, after);
}

TEST(SarEchoClutterTest, AddNoiseIsDeterministic) {
  signal::ComplexVector s1(64, signal::ComplexSample(0.5, 0.0));
  signal::ComplexVector s2 = s1;

  NoiseSpec spec;
  spec.signal_to_noise_ratio_db = 0.0;
  spec.random_seed = 42U;
  ASSERT_TRUE(AddNoise(spec, &s1));
  ASSERT_TRUE(AddNoise(spec, &s2));

  for (std::size_t i = 0; i < s1.size(); ++i) {
    EXPECT_NEAR(s1[i].real(), s2[i].real(), 1.0e-12);
    EXPECT_NEAR(s1[i].imag(), s2[i].imag(), 1.0e-12);
  }
}

TEST(SarEchoClutterTest, AddNoiseRejectsNull) {
  NoiseSpec spec;
  EXPECT_FALSE(AddNoise(spec, nullptr));
}

TEST(SarEchoClutterTest, AddNoiseRejectsEmpty) {
  signal::ComplexVector v;
  NoiseSpec spec;
  EXPECT_FALSE(AddNoise(spec, &v));
}

// ── GammaClutterRcs ───────────────────────────────────────────

TEST(SarEchoClutterTest, GammaClutterRcsIncreasesWithIncidence) {
  ClutterModel model;
  model.type = ClutterType::kGamma;
  model.gamma_constant = 0.1;
  model.resolution_cell_area_m2 = 10.0;

  model.incidence_angle_rad = 0.1;
  const double rcs_low = GammaClutterRcs(model);

  model.incidence_angle_rad = 0.5;
  const double rcs_high = GammaClutterRcs(model);

  // sin(0.5) > sin(0.1)
  EXPECT_GT(rcs_high, rcs_low);
}

TEST(SarEchoClutterTest, GammaClutterRcsIsLinearInGamma) {
  ClutterModel model;
  model.type = ClutterType::kGamma;
  model.incidence_angle_rad = 0.5236;  // 30°
  model.resolution_cell_area_m2 = 1.0;

  model.gamma_constant = 0.1;
  const double rcs1 = GammaClutterRcs(model);

  model.gamma_constant = 0.2;
  const double rcs2 = GammaClutterRcs(model);

  EXPECT_NEAR(rcs2, rcs1 * 2.0, 1.0e-12);
}

// ── SeaClutterRcs ─────────────────────────────────────────────

TEST(SarEchoClutterTest, SeaClutterRcsIncreasesWithSeaState) {
  ClutterModel model;
  model.type = ClutterType::kSea;
  model.wind_speed_mps = 5.0;
  model.incidence_angle_rad = 0.5236;  // 30°
  model.resolution_cell_area_m2 = 1.0;

  model.sea_state = 1.0;
  const double rcs_low = SeaClutterRcs(model);

  model.sea_state = 5.0;
  const double rcs_high = SeaClutterRcs(model);

  // 更高海况 → 更大 σ⁰
  EXPECT_GT(rcs_high, rcs_low);
}

TEST(SarEchoClutterTest, SeaClutterRcsIncreasesWithWind) {
  ClutterModel model;
  model.type = ClutterType::kSea;
  model.sea_state = 3.0;
  model.incidence_angle_rad = 0.5236;
  model.resolution_cell_area_m2 = 1.0;

  model.wind_speed_mps = 1.0;
  const double rcs_low = SeaClutterRcs(model);

  model.wind_speed_mps = 20.0;
  const double rcs_high = SeaClutterRcs(model);

  EXPECT_GT(rcs_high, rcs_low);
}

// ── GenerateClutterScene ─────────────────────────────────────

TEST(SarEchoClutterTest, ClutterSceneIncludesPointTargets) {
  const RawEchoConfig config = MakeConfig();
  const geometry::PlatformPulseState platform = MakePlatform();
  const signal::ComplexVector waveform = MakeSimplePulse(32);

  SceneDescription scene;
  scene.scene_center = geometry::LocalPoint{0.0, 0.0, 0.0};
  scene.scene_extent_x_m = 50.0;
  scene.scene_extent_y_m = 50.0;
  scene.clutter_grid_spacing_m = 0.0;  // 跳过杂波

  PointTarget pt;
  // 目标在平台正下方 100 m 处: R≈100 m, amplitude ≈ sqrt(10)/100² ≈ 3.16e-3
  pt.position_m = geometry::LocalPoint{0.0, 0.0, 4900.0};
  pt.rcs_m2 = 10.0;
  scene.point_targets.push_back(pt);

  RawEchoResult result;
  ASSERT_TRUE(GenerateClutterScene(config, platform, scene, waveform, &result));

  EXPECT_EQ(result.samples.size(), config.range_sample_count);
  // 点目标回波应非零
  bool has_nonzero = false;
  for (const auto& v : result.samples) {
    if (std::norm(v) > 1.0e-12) {
      has_nonzero = true;
      break;
    }
  }
  EXPECT_TRUE(has_nonzero);
}

TEST(SarEchoClutterTest, ClutterSceneWithGridProducesNonZero) {
  const RawEchoConfig config = MakeConfig();
  const geometry::PlatformPulseState platform = MakePlatform();
  const signal::ComplexVector waveform = MakeSimplePulse(32);

  SceneDescription scene;
  scene.scene_center = geometry::LocalPoint{0.0, 0.0, 0.0};
  scene.scene_extent_x_m = 20.0;
  scene.scene_extent_y_m = 20.0;
  scene.clutter_grid_spacing_m = 5.0;
  scene.clutter.type = ClutterType::kGamma;
  scene.clutter.gamma_constant = 0.01;
  scene.clutter.incidence_angle_rad = 0.5;

  RawEchoResult result;
  ASSERT_TRUE(GenerateClutterScene(config, platform, scene, waveform, &result));

  EXPECT_EQ(result.samples.size(), config.range_sample_count);
  EXPECT_FALSE(result.samples.empty());
}

TEST(SarEchoClutterTest, ClutterSceneRejectsInvalidConfig) {
  RawEchoConfig bad_config;
  // all zeros → invalid
  SceneDescription scene;
  signal::ComplexVector waveform = MakeSimplePulse(32);
  RawEchoResult result;
  EXPECT_FALSE(GenerateClutterScene(bad_config, MakePlatform(), scene, waveform, &result));

  EXPECT_FALSE(GenerateClutterScene(MakeConfig(), MakePlatform(), scene, waveform, nullptr));
}

TEST(SarEchoClutterTest, ClutterSceneTotalEnergyIsFinite) {
  const RawEchoConfig config = MakeConfig();
  const geometry::PlatformPulseState platform = MakePlatform();
  const signal::ComplexVector waveform = MakeSimplePulse(32);

  SceneDescription scene;
  scene.scene_center = geometry::LocalPoint{0.0, 0.0, 0.0};
  scene.scene_extent_x_m = 30.0;
  scene.scene_extent_y_m = 30.0;
  scene.clutter_grid_spacing_m = 10.0;
  scene.clutter.type = ClutterType::kSea;
  scene.clutter.sea_state = 3.0;
  scene.clutter.wind_speed_mps = 8.0;

  RawEchoResult result;
  ASSERT_TRUE(GenerateClutterScene(config, platform, scene, waveform, &result));

  for (const auto& v : result.samples) {
    EXPECT_TRUE(std::isfinite(v.real()));
    EXPECT_TRUE(std::isfinite(v.imag()));
  }
}

}  // namespace echo
}  // namespace sar
