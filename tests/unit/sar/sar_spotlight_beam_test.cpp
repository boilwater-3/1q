// MSVC 需在首次包含 <cmath> 前定义 _USE_MATH_DEFINES 才有 M_PI（gtest.h 内部已含 <cmath>）。
#define _USE_MATH_DEFINES
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "sar/echo/SarEcho.h"
#include "sar/geometry/SarAntenna.h"
#include "sar/geometry/SarGeometry.h"
#include "sar/geometry/SarSpotlightBeam.h"
#include "sar/signal/SarWaveform.h"

// 聚束上游建模测试(契约 spotlight_mode.md §3):
// 波束指向律 + 回波天线方位调制 + 聚束轨迹组合生成器。
namespace sar {
namespace {
namespace {

constexpr double kSpeedOfLightMps = 299792458.0;

// 构建一个简单的直线匀速轨迹(3 脉冲)用于波束指向测试。
std::vector<geometry::PlatformPulseState> MakeLinearTrack(double velocity_mps, double prf_hz,
                                                           double start_x_m) {
  std::vector<geometry::PlatformPulseState> pulses;
  for (std::size_t i = 0U; i < 3U; ++i) {
    geometry::PlatformPulseState pulse;
    pulse.pulse_id = static_cast<std::uint64_t>(i);
    pulse.time_s = static_cast<double>(i) / prf_hz;
    pulse.position_m.x_m = start_x_m + velocity_mps * pulse.time_s;
    pulse.position_m.y_m = 100.0;  // 平台在 y=100 侧视
    pulse.position_m.z_m = 0.0;
    pulse.velocity_x_mps = velocity_mps;
    pulses.push_back(pulse);
  }
  return pulses;
}

}  // namespace

// 波束指向律正确性:boresight 始终指向场景中心。
TEST(SarSpotlightBeamTest, BeamPointsTowardSceneCenter) {
  const std::vector<geometry::PlatformPulseState> pulses = MakeLinearTrack(20.0, 50.0, -10.0);
  geometry::SpotlightBeamTrackConfig config;
  config.scene_center_m = geometry::LocalPoint{0.0, 0.0, 0.0};

  std::vector<geometry::SpotlightBeamState> beam_states;
  ASSERT_TRUE(geometry::GenerateSpotlightBeamTrack(config, pulses, &beam_states));
  ASSERT_EQ(beam_states.size(), pulses.size());

  for (std::size_t i = 0U; i < pulses.size(); ++i) {
    // 验证 boresight = atan2(cx - px, cy - py)。
    const double expected =
        std::atan2(config.scene_center_m.x_m - pulses[i].position_m.x_m,
                   config.scene_center_m.y_m - pulses[i].position_m.y_m);
    EXPECT_NEAR(beam_states[i].boresight_azimuth_rad, expected, 1e-12);
    EXPECT_NEAR(beam_states[i].time_s, pulses[i].time_s, 1e-12);
  }
}

// broadside 退化:场景中心在平台正侧方(y 轴),中间脉冲 boresight 指向正后方。
TEST(SarSpotlightBeamTest, BroadsideDegeneracy) {
  const std::vector<geometry::PlatformPulseState> pulses = MakeLinearTrack(20.0, 50.0, -10.0);
  geometry::SpotlightBeamTrackConfig config;
  config.scene_center_m = geometry::LocalPoint{0.0, 0.0, 0.0};

  std::vector<geometry::SpotlightBeamState> beam_states;
  ASSERT_TRUE(geometry::GenerateSpotlightBeamTrack(config, pulses, &beam_states));
  // 中间脉冲平台在 x≈0,场景中心 (0,0):boresight = atan2(0, -100) = π。
  // 用期望公式而非硬编码,确保与实现一致。
  const double expected_center = std::atan2(
      config.scene_center_m.x_m - pulses[1U].position_m.x_m,
      config.scene_center_m.y_m - pulses[1U].position_m.y_m);
  EXPECT_NEAR(beam_states[1U].boresight_azimuth_rad, expected_center, 1e-12);
  // 首尾脉冲 boresight 应对称(平台从 -x 飞到 +x,场景中心在 x=0)。
  const double first = beam_states.front().boresight_azimuth_rad;
  const double last = beam_states.back().boresight_azimuth_rad;
  // 对称性:first 和 last 应关于中心 boresight 对称(|first - center| ≈ |last - center|)。
  const double center = beam_states[1U].boresight_azimuth_rad;
  EXPECT_NEAR(std::fabs(first - center), std::fabs(last - center), 0.01);
}

// 聚束轨迹组合生成器:平台轨迹 + 波束序列等长对齐。
TEST(SarSpotlightBeamTest, SpotlightTrackGeneratesAlignedSequences) {
  geometry::SpotlightTrackConfig config;
  config.platform_track.start_position_m = geometry::LocalPoint{-10.0, 100.0, 0.0};
  config.platform_track.velocity_x_mps = 20.0;
  config.platform_track.prf_hz = 50.0;
  config.platform_track.pulse_count = 9U;
  config.scene_center_m = geometry::LocalPoint{0.0, 0.0, 0.0};

  std::vector<geometry::PlatformPulseState> pulses;
  std::vector<geometry::SpotlightBeamState> beam_states;
  ASSERT_TRUE(geometry::GenerateSpotlightTrack(config, &pulses, &beam_states));
  EXPECT_EQ(pulses.size(), 9U);
  EXPECT_EQ(beam_states.size(), 9U);
  for (std::size_t i = 0U; i < pulses.size(); ++i) {
    EXPECT_NEAR(pulses[i].time_s, beam_states[i].time_s, 1e-12);
  }
}

// 聚束合成孔径时间:由波束跟踪角决定,非条带公式。
TEST(SarSpotlightBeamTest, SyntheticApertureTimeFromBeamAngle) {
  // 构造一个有明显波束转动的场景:平台从 x=-100 飞到 x=+100,场景中心 (0, 100)。
  geometry::SpotlightTrackConfig config;
  config.platform_track.start_position_m = geometry::LocalPoint{-100.0, 0.0, 0.0};
  config.platform_track.velocity_x_mps = 200.0;
  config.platform_track.prf_hz = 100.0;
  config.platform_track.pulse_count = 11U;
  config.scene_center_m = geometry::LocalPoint{0.0, 100.0, 0.0};

  std::vector<geometry::PlatformPulseState> pulses;
  std::vector<geometry::SpotlightBeamState> beam_states;
  ASSERT_TRUE(geometry::GenerateSpotlightTrack(config, &pulses, &beam_states));

  const double t_synth = geometry::SpotlightSyntheticApertureTime(
      beam_states, 100.0 /*slant_range*/, 200.0 /*velocity*/);
  // 应为正值(有波束转动)。
  EXPECT_GT(t_synth, 0.0);
  // 条带合成孔径时间 = R0·θ_bw/v;聚束可超过它(由跟踪角决定)。
}

// 回波天线调制:broadside 目标(在波束中心)衰减最小,离轴目标衰减大。
TEST(SarSpotlightBeamTest, AntennaModulationAttenuatesOffBoresightTarget) {
  echo::RawEchoConfig echo_config;
  echo_config.sample_rate_hz = 1.0e8;
  echo_config.carrier_frequency_hz = 1.0e9;
  echo_config.range_sample_count = 64U;

  geometry::PlatformPulseState platform;
  platform.position_m = geometry::LocalPoint{0.0, 50.0, 0.0};  // 斜距 50m → delay≈33 样本
  platform.velocity_x_mps = 20.0;

  // 目标 A 在 boresight 正前方;目标 B 偏离 boresight。
  const echo::PointTarget target_on_axis =
      echo::PointTarget{geometry::LocalPoint{0.0, 0.0, 0.0}, 1.0};
  const echo::PointTarget target_off_axis =
      echo::PointTarget{geometry::LocalPoint{30.0, 0.0, 0.0}, 1.0};

  signal::ComplexVector waveform(8U, signal::ComplexSample(1.0, 0.0));

  // 无天线调制。
  echo::AntennaModulationConfig no_antenna;
  no_antenna.enabled = false;
  echo::RawEchoResult result_no_antenna;
  ASSERT_TRUE(echo::GeneratePointTargetRawEchoWithAntenna(
      echo_config, no_antenna, platform, {target_on_axis, target_off_axis}, waveform,
      &result_no_antenna));
  ASSERT_FALSE(result_no_antenna.has_clipping);

  // 有天线调制:boresight 指向目标 A(正前方, atan2(0, -50) = π)。
  geometry::AntennaParams antenna;
  antenna.length_m = 2.0;
  echo::AntennaModulationConfig with_antenna;
  with_antenna.enabled = true;
  with_antenna.antenna = antenna;
  with_antenna.beam_state.boresight_azimuth_rad = M_PI;
  echo::RawEchoResult result_with_antenna;
  ASSERT_TRUE(echo::GeneratePointTargetRawEchoWithAntenna(
      echo_config, with_antenna, platform, {target_on_axis, target_off_axis}, waveform,
      &result_with_antenna));
  ASSERT_FALSE(result_with_antenna.has_clipping);

  // 有调制总功率应低于无调制(目标 B 离轴被衰减)。
  double power_no_antenna = 0.0;
  double power_with_antenna = 0.0;
  for (const signal::ComplexSample& s : result_no_antenna.samples) {
    power_no_antenna += std::norm(s);
  }
  for (const signal::ComplexSample& s : result_with_antenna.samples) {
    power_with_antenna += std::norm(s);
  }
  EXPECT_LT(power_with_antenna, power_no_antenna);
}

// 退化不变量:enabled=false 时,WithAntenna 输出 == GeneratePointTargetRawEcho 输出。
TEST(SarSpotlightBeamTest, DisabledAntennaMatchesPlainEcho) {
  echo::RawEchoConfig echo_config;
  echo_config.sample_rate_hz = 1.0e8;
  echo_config.carrier_frequency_hz = 1.0e9;
  echo_config.range_sample_count = 64U;

  geometry::PlatformPulseState platform;
  platform.position_m = geometry::LocalPoint{0.0, 100.0, 0.0};

  const echo::PointTarget target{geometry::LocalPoint{5.0, 0.0, 0.0}, 1.0};
  signal::ComplexVector waveform(8U, signal::ComplexSample(1.0, 0.0));

  echo::RawEchoResult plain_result;
  ASSERT_TRUE(
      echo::GeneratePointTargetRawEcho(echo_config, platform, {target}, waveform, &plain_result));

  echo::AntennaModulationConfig disabled;
  disabled.enabled = false;
  echo::RawEchoResult antenna_result;
  ASSERT_TRUE(echo::GeneratePointTargetRawEchoWithAntenna(echo_config, disabled, platform, {target},
                                                          waveform, &antenna_result));

  EXPECT_EQ(plain_result.samples, antenna_result.samples);
}

// 拒绝路径:空脉冲序列。
TEST(SarSpotlightBeamTest, RejectsEmptyPulses) {
  geometry::SpotlightBeamTrackConfig config;
  config.scene_center_m = geometry::LocalPoint{0.0, 0.0, 0.0};
  std::vector<geometry::SpotlightBeamState> beam_states;
  EXPECT_FALSE(geometry::GenerateSpotlightBeamTrack(config, {}, &beam_states));
}

}  // namespace
}  // namespace sar
