#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "sar/echo/SarEcho.h"
#include "sar/geometry/SarGeometry.h"
#include "sar/geometry/SarSpotlightBeam.h"
#include "sar/imaging/SarGbp.h"
#include "sar/imaging/SarImageQuality.h"
#include "sar/imaging/SarOmegaKFocusing.h"
#include "sar/signal/SarWaveform.h"
#include "support/sar_reference_scene.h"

// 聚束 Omega-K 聚焦编排器测试(契约 spotlight_mode.md §4-5)。
// Stolt 映射天然 squint-invariant,聚束编排器与条带共享全部 8 部件。
namespace sar {
namespace imaging {
namespace {

constexpr double kSpeedOfLightMps = 299792458.0;

OmegaKConfig MakeBaseConfig(const test_support::ReferencePointScene& scene,
                            std::size_t reference_delay) {
  OmegaKConfig config;
  config.range_sample_count = scene.range_sample_count;
  config.azimuth_pulse_count = scene.pulses.size();
  config.sample_rate_hz = scene.sample_rate_hz;
  config.prf_hz = scene.prf_hz;
  config.carrier_frequency_hz = scene.carrier_frequency_hz;
  config.platform_velocity_mps = scene.platform_velocity_mps;
  config.reference_range_m =
      static_cast<double>(reference_delay) * kSpeedOfLightMps / (2.0 * scene.sample_rate_hz);
  return config;
}

// broadside 退化不变量:scene_center_azimuth_m=0 时,聚束输出 == 条带输出。
TEST(SarOmegaKSpotlightTest, BroadsideDegeneratesToStripmap) {
  test_support::ReferencePointScene scene;
  scene.pulse_count = 9U;
  scene.prf_hz = 50.0;
  scene.platform_velocity_mps = 20.0;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::size_t reference_delay = 20U;
  const echo::PointTarget target =
      test_support::MakeReferenceTargetAtDelay(reference_delay, scene.sample_rate_hz, 1.0);
  signal::ComplexMatrix raw;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, {target}, &raw));

  const OmegaKConfig base = MakeBaseConfig(scene, reference_delay);

  FocusedOmegaKImage stripmap;
  FocusedOmegaKImage spotlight;
  ASSERT_TRUE(FocusStripmapOmegaK(base, raw, &stripmap));

  SpotlightOmegaKConfig spotlight_config;
  static_cast<OmegaKConfig&>(spotlight_config) = base;
  spotlight_config.scene_center_azimuth_m = 0.0;  // broadside 退化
  ASSERT_TRUE(FocusSpotlightOmegaK(spotlight_config, raw, &spotlight));

  // 逐样本一致(broadside 退化不变量)。
  ASSERT_EQ(stripmap.image.rows, spotlight.image.rows);
  ASSERT_EQ(stripmap.image.cols, spotlight.image.cols);
  for (std::size_t i = 0U; i < stripmap.image.values.size(); ++i) {
    EXPECT_NEAR(stripmap.image.values[i].real(), spotlight.image.values[i].real(), 1e-9);
    EXPECT_NEAR(stripmap.image.values[i].imag(), spotlight.image.values[i].imag(), 1e-9);
  }
}

// 聚束编排器成功运行并产生有限图像。
TEST(SarOmegaKSpotlightTest, FocusesSpotlightScene) {
  test_support::ReferencePointScene scene;
  scene.pulse_count = 9U;
  scene.prf_hz = 50.0;
  scene.platform_velocity_mps = 20.0;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::size_t reference_delay = 20U;
  const echo::PointTarget target =
      test_support::MakeReferenceTargetAtDelay(reference_delay, scene.sample_rate_hz, 1.0);
  signal::ComplexMatrix raw;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, {target}, &raw));

  SpotlightOmegaKConfig config;
  static_cast<OmegaKConfig&>(config) = MakeBaseConfig(scene, reference_delay);
  config.scene_center_azimuth_m = 5.0;  // 非零偏移(模拟聚束场景中心偏离航迹中心)

  FocusedOmegaKImage output;
  ASSERT_TRUE(FocusSpotlightOmegaK(config, raw, &output));
  EXPECT_EQ(output.diagnostics.failure_stage, "none");

  for (const signal::ComplexSample& s : output.image.values) {
    EXPECT_TRUE(std::isfinite(s.real()));
    EXPECT_TRUE(std::isfinite(s.imag()));
  }
}

// 确定性:相同输入相同输出。
TEST(SarOmegaKSpotlightTest, Deterministic) {
  test_support::ReferencePointScene scene;
  scene.pulse_count = 9U;
  scene.prf_hz = 50.0;
  scene.platform_velocity_mps = 20.0;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const std::size_t reference_delay = 20U;
  const echo::PointTarget target =
      test_support::MakeReferenceTargetAtDelay(reference_delay, scene.sample_rate_hz, 1.0);
  signal::ComplexMatrix raw;
  ASSERT_TRUE(test_support::BuildReferenceRawHistory(scene, {target}, &raw));

  SpotlightOmegaKConfig config;
  static_cast<OmegaKConfig&>(config) = MakeBaseConfig(scene, reference_delay);
  config.scene_center_azimuth_m = 3.0;

  FocusedOmegaKImage first;
  FocusedOmegaKImage second;
  ASSERT_TRUE(FocusSpotlightOmegaK(config, raw, &first));
  ASSERT_TRUE(FocusSpotlightOmegaK(config, raw, &second));
  EXPECT_EQ(first.image.values, second.image.values);
}

// 拒绝路径:空输出指针。
TEST(SarOmegaKSpotlightTest, RejectsNullOutput) {
  test_support::ReferencePointScene scene;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  const OmegaKConfig base = MakeBaseConfig(scene, 20U);
  SpotlightOmegaKConfig config;
  static_cast<OmegaKConfig&>(config) = base;
  EXPECT_FALSE(FocusSpotlightOmegaK(config, signal::ComplexMatrix{}, nullptr));
}

// 拒绝路径:单脉冲(无法构成方位孔径)。
TEST(SarOmegaKSpotlightTest, RejectsSinglePulse) {
  test_support::ReferencePointScene scene;
  ASSERT_TRUE(test_support::BuildReferencePointScene(&scene));
  SpotlightOmegaKConfig config;
  static_cast<OmegaKConfig&>(config) = MakeBaseConfig(scene, 20U);
  config.azimuth_pulse_count = 1U;
  FocusedOmegaKImage output;
  EXPECT_FALSE(FocusSpotlightOmegaK(config, signal::ComplexMatrix{}, &output));
}

}  // namespace
}  // namespace imaging
}  // namespace sar
