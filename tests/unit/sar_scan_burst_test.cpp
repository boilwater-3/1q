#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "sar/echo/SarEcho.h"
#include "sar/geometry/SarGeometry.h"
#include "sar/geometry/SarScanBurst.h"
#include "sar/signal/SarWaveform.h"

// 扫描模式上游建模测试(契约 scansar_mode.md §3):
// elevation burst 调度律 + 回波距离门控 + ScanSAR 轨迹组合生成器。
namespace sar {
namespace {
namespace {

// 构造一组等间距慢时间(模拟 PRF 采样)。
std::vector<double> MakePulseTimes(std::uint32_t count, double prf_hz, double start_s) {
  std::vector<double> times;
  times.reserve(count);
  const double dt = 1.0 / prf_hz;
  for (std::uint32_t i = 0U; i < count; ++i) {
    times.push_back(start_s + static_cast<double>(i) * dt);
  }
  return times;
}

}  // namespace

// ── 辅助函数 ──────────────────────────────────────────────

// 子带中心斜距正确性。
TEST(SarScanBurstTest, SubswathCenterRange) {
  geometry::ScanSubswath subswath{100.0, 200.0};
  EXPECT_NEAR(geometry::SubswathCenterRange(subswath), 150.0, 1e-12);
}

// 子带窗口判定:半开区间 [near, far)。
TEST(SarScanBurstTest, IsInSubswathHalfOpenInterval) {
  geometry::ScanSubswath subswath{100.0, 200.0};
  EXPECT_TRUE(geometry::IsInSubswath(subswath, 100.0));   // near 边界包含
  EXPECT_TRUE(geometry::IsInSubswath(subswath, 150.0));   // 内部
  EXPECT_FALSE(geometry::IsInSubswath(subswath, 200.0));  // far 边界排除(半开)
  EXPECT_FALSE(geometry::IsInSubswath(subswath, 99.9));   // 近端外
  EXPECT_FALSE(geometry::IsInSubswath(subswath, 200.1));  // 远端外
}

// ── burst 调度律 ──────────────────────────────────────────

// 双子带轮转:周期性切换 subswath_index。
TEST(SarScanBurstTest, DualSubswathAlternates) {
  geometry::ScanBurstScheduleConfig config;
  config.subswaths = {{100.0, 200.0}, {200.0, 300.0}};
  config.dwell_time_s = 0.1;
  // T_cycle = 0.2。PRF=50 → dt=0.02,每子带占 5 个脉冲。
  config.pulse_times_s = MakePulseTimes(10U, 50.0, 0.0);

  std::vector<geometry::ScanBurstState> schedule;
  ASSERT_TRUE(geometry::GenerateScanBurstSchedule(config, &schedule));
  ASSERT_EQ(schedule.size(), 10U);

  // 前 5 脉冲(0..0.08s)落在 slot 0 → subswath 0。
  for (std::size_t i = 0U; i < 5U; ++i) {
    EXPECT_EQ(schedule[i].subswath_index, 0U) << "pulse " << i;
    EXPECT_TRUE(schedule[i].illuminated);
    EXPECT_NEAR(schedule[i].near_range_m, 100.0, 1e-12);
  }
  // 后 5 脉冲(0.10..0.18s)落在 slot 1 → subswath 1。
  for (std::size_t i = 5U; i < 10U; ++i) {
    EXPECT_EQ(schedule[i].subswath_index, 1U) << "pulse " << i;
    EXPECT_TRUE(schedule[i].illuminated);
    EXPECT_NEAR(schedule[i].far_range_m, 300.0, 1e-12);
  }
}

// 跨周期续转:第二个周期回到 subswath 0。
TEST(SarScanBurstTest, ContinuesAcrossCycles) {
  geometry::ScanBurstScheduleConfig config;
  config.subswaths = {{100.0, 200.0}, {200.0, 300.0}};
  config.dwell_time_s = 0.1;  // T_cycle = 0.2
  // 11 脉冲,PRF=50:第 11 脉冲 t=0.20s 落在第二周期 slot 0。
  config.pulse_times_s = MakePulseTimes(11U, 50.0, 0.0);

  std::vector<geometry::ScanBurstState> schedule;
  ASSERT_TRUE(geometry::GenerateScanBurstSchedule(config, &schedule));
  // t=0.20s → cycle_offset = fmod(0.20, 0.20) = 0 → slot 0 → subswath 0。
  EXPECT_EQ(schedule[10U].subswath_index, 0U);
}

// 单子带退化不变量:N_swath=1 → 全程 subswath 0、illuminated(退化为条带)。
TEST(SarScanBurstTest, SingleSubswathDegeneratesToStripmap) {
  geometry::ScanBurstScheduleConfig config;
  config.subswaths = {{100.0, 300.0}};
  config.dwell_time_s = 0.1;
  config.pulse_times_s = MakePulseTimes(8U, 50.0, 0.0);

  std::vector<geometry::ScanBurstState> schedule;
  ASSERT_TRUE(geometry::GenerateScanBurstSchedule(config, &schedule));
  for (const geometry::ScanBurstState& state : schedule) {
    EXPECT_EQ(state.subswath_index, 0U);
    EXPECT_TRUE(state.illuminated);
    EXPECT_NEAR(state.near_range_m, 100.0, 1e-12);
    EXPECT_NEAR(state.far_range_m, 300.0, 1e-12);
  }
}

// ── 拒绝路径 ──────────────────────────────────────────────

TEST(SarScanBurstTest, RejectsEmptySubswaths) {
  geometry::ScanBurstScheduleConfig config;
  config.dwell_time_s = 0.1;
  config.pulse_times_s = MakePulseTimes(4U, 50.0, 0.0);
  std::vector<geometry::ScanBurstState> schedule;
  EXPECT_FALSE(geometry::GenerateScanBurstSchedule(config, &schedule));
}

TEST(SarScanBurstTest, RejectsZeroDwellTime) {
  geometry::ScanBurstScheduleConfig config;
  config.subswaths = {{100.0, 200.0}};
  config.dwell_time_s = 0.0;
  config.pulse_times_s = MakePulseTimes(4U, 50.0, 0.0);
  std::vector<geometry::ScanBurstState> schedule;
  EXPECT_FALSE(geometry::GenerateScanBurstSchedule(config, &schedule));
}

TEST(SarScanBurstTest, RejectsEmptyPulseTimes) {
  geometry::ScanBurstScheduleConfig config;
  config.subswaths = {{100.0, 200.0}};
  config.dwell_time_s = 0.1;
  std::vector<geometry::ScanBurstState> schedule;
  EXPECT_FALSE(geometry::GenerateScanBurstSchedule(config, &schedule));
}

TEST(SarScanBurstTest, RejectsNullOutput) {
  geometry::ScanBurstScheduleConfig config;
  config.subswaths = {{100.0, 200.0}};
  config.dwell_time_s = 0.1;
  config.pulse_times_s = MakePulseTimes(4U, 50.0, 0.0);
  EXPECT_FALSE(geometry::GenerateScanBurstSchedule(config, nullptr));
}

TEST(SarScanBurstTest, RejectsInvertedSubswathWindow) {
  geometry::ScanBurstScheduleConfig config;
  config.subswaths = {{200.0, 100.0}};  // near > far
  config.dwell_time_s = 0.1;
  config.pulse_times_s = MakePulseTimes(4U, 50.0, 0.0);
  std::vector<geometry::ScanBurstState> schedule;
  EXPECT_FALSE(geometry::GenerateScanBurstSchedule(config, &schedule));
}

// ── 组合生成器 ────────────────────────────────────────────

// ScanSAR 轨迹组合:平台脉冲与 burst 调度等长对齐。
TEST(SarScanBurstTest, TrackGeneratesAlignedSequences) {
  geometry::ScanSarTrackConfig config;
  config.platform_track.start_position_m = geometry::LocalPoint{-10.0, 100.0, 0.0};
  config.platform_track.velocity_x_mps = 20.0;
  config.platform_track.prf_hz = 50.0;
  config.platform_track.pulse_count = 9U;
  config.subswaths = {{80.0, 120.0}, {120.0, 160.0}};
  config.dwell_time_s = 0.1;

  std::vector<geometry::PlatformPulseState> pulses;
  std::vector<geometry::ScanBurstState> schedule;
  ASSERT_TRUE(geometry::GenerateScanSarTrack(config, &pulses, &schedule));
  EXPECT_EQ(pulses.size(), 9U);
  EXPECT_EQ(schedule.size(), 9U);
  for (std::size_t i = 0U; i < pulses.size(); ++i) {
    EXPECT_NEAR(pulses[i].time_s, schedule[i].time_s, 1e-12);
  }
}

// ── 回波 elevation 门控 ───────────────────────────────────

// 门控正确:窗内目标贡献,窗外目标被跳过。
TEST(SarScanBurstTest, ElevationGatePassesInWindowRejectsOutWindow) {
  echo::RawEchoConfig echo_config;
  echo_config.sample_rate_hz = 1.0e8;
  echo_config.carrier_frequency_hz = 1.0e9;
  echo_config.range_sample_count = 64U;

  geometry::PlatformPulseState platform;
  platform.position_m = geometry::LocalPoint{0.0, 0.0, 0.0};
  platform.velocity_x_mps = 20.0;

  // 目标近(target_near, 斜距≈30m)在子带 [20, 50) 内;目标远(target_far, 斜距≈100m)在窗外。
  // 延迟样本 = 2·R·fs/c。R=30 → delay=20 样本(在 64 内);R=100 → delay≈67 样本(越界)。
  // 为保证两目标都落在 64 样本内以便对比,用小斜距。
  const echo::PointTarget target_in_window{geometry::LocalPoint{0.0, 30.0, 0.0}, 1.0};   // R=30
  const echo::PointTarget target_out_window{geometry::LocalPoint{0.0, 60.0, 0.0}, 1.0};  // R=60

  signal::ComplexVector waveform(8U, signal::ComplexSample(1.0, 0.0));

  // 子带 [20, 50):target_in(R=30)在窗内,target_out(R=60)在窗外。
  echo::ElevationGateConfig gate;
  gate.enabled = true;
  gate.burst_state.illuminated = true;
  gate.burst_state.near_range_m = 20.0;
  gate.burst_state.far_range_m = 50.0;
  echo::RawEchoResult gated_result;
  ASSERT_TRUE(echo::GeneratePointTargetRawEchoWithElevationGate(
      echo_config, gate, platform, {target_in_window, target_out_window}, waveform,
      &gated_result));
  // 仅窗内 1 个目标贡献。
  EXPECT_EQ(gated_result.diagnostics.size(), 1U);

  // 关闭门控:两目标都贡献。
  echo::ElevationGateConfig no_gate;
  no_gate.enabled = false;
  echo::RawEchoResult ungated_result;
  ASSERT_TRUE(echo::GeneratePointTargetRawEchoWithElevationGate(
      echo_config, no_gate, platform, {target_in_window, target_out_window}, waveform,
      &ungated_result));
  EXPECT_EQ(ungated_result.diagnostics.size(), 2U);
}

// 退化不变量:enabled=false 时,WithElevationGate 输出 == GeneratePointTargetRawEcho 输出。
TEST(SarScanBurstTest, DisabledGateMatchesPlainEcho) {
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

  echo::ElevationGateConfig disabled;
  disabled.enabled = false;
  echo::RawEchoResult gated_result;
  ASSERT_TRUE(echo::GeneratePointTargetRawEchoWithElevationGate(echo_config, disabled, platform,
                                                                 {target}, waveform, &gated_result));
  EXPECT_EQ(plain_result.samples, gated_result.samples);
}

// illuminated=false 时,该脉冲全程无贡献(天线在别的子带)。
TEST(SarScanBurstTest, UnilluminatedBurstProducesNoEcho) {
  echo::RawEchoConfig echo_config;
  echo_config.sample_rate_hz = 1.0e8;
  echo_config.carrier_frequency_hz = 1.0e9;
  echo_config.range_sample_count = 64U;

  geometry::PlatformPulseState platform;
  platform.position_m = geometry::LocalPoint{0.0, 0.0, 0.0};

  const echo::PointTarget target{geometry::LocalPoint{0.0, 30.0, 0.0}, 1.0};  // R=30, 在 [20,50) 内
  signal::ComplexVector waveform(8U, signal::ComplexSample(1.0, 0.0));

  echo::ElevationGateConfig gate;
  gate.enabled = true;
  gate.burst_state.illuminated = false;  // 天线此刻不在看这个子带
  gate.burst_state.near_range_m = 20.0;
  gate.burst_state.far_range_m = 50.0;
  echo::RawEchoResult result;
  ASSERT_TRUE(echo::GeneratePointTargetRawEchoWithElevationGate(echo_config, gate, platform,
                                                                 {target}, waveform, &result));
  // 无任何目标贡献(全程零)。
  EXPECT_EQ(result.diagnostics.size(), 0U);
  for (const signal::ComplexSample& s : result.samples) {
    EXPECT_EQ(s, signal::ComplexSample(0.0, 0.0));
  }
}

}  // namespace
}  // namespace sar
