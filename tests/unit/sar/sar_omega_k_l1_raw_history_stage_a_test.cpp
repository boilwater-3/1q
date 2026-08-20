#include <gtest/gtest.h>

#include <cstdint>
#include <deque>

#include "sar/imaging/SarOmegaKFocusing.h"
#include "sar/runtime/PulseRingBuffer.h"
#include "sar/session/SarRawHistoryBuilder.h"

namespace sar {
namespace imaging {
namespace {

config::SarSessionConfig MakeL1Config() {
  config::SarSessionConfig config;
  config.hardware.carrier_frequency_hz = 1.0e9;
  config.hardware.bandwidth_hz = 25.0e6;
  config.hardware.pulse_width_s = 0.16e-6;
  config.hardware.pulse_repetition_frequency_hz = 20.0;
  config.hardware.sample_rate_hz = 100.0e6;
  config.mission.nominal_slant_range_m = 29.9792458;
  config.mission.platform_speed_mps = 2.0;
  config.mission.range_sample_count = 64U;
  config.mission.azimuth_pulse_count = 9U;
  return config;
}

session::SarCycleInput MakeL1Input() {
  session::SarCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 0.1f;
  session::SarPointTarget target;
  target.latitude_deg = 29.9792458 / 6378137.0 * 180.0 / 3.14159265358979323846;
  target.radar_cross_section_dbsm = 80.0;
  input.point_targets.push_back(target);
  return input;
}

OmegaKConfig MakeOmegaKConfig(const config::SarSessionConfig& config) {
  OmegaKConfig omega_k;
  omega_k.range_sample_count = config.mission.range_sample_count;
  omega_k.azimuth_pulse_count = config.mission.azimuth_pulse_count;
  omega_k.sample_rate_hz = config.hardware.sample_rate_hz;
  omega_k.prf_hz = config.hardware.pulse_repetition_frequency_hz;
  omega_k.carrier_frequency_hz = config.hardware.carrier_frequency_hz;
  omega_k.platform_velocity_mps = config.mission.platform_speed_mps;
  omega_k.reference_range_m = config.mission.nominal_slant_range_m;
  return omega_k;
}

bool BuildL1RawHistory(const config::SarSessionConfig& config, signal::ComplexMatrix* raw_history) {
  signal::LfmWaveform waveform;
  signal::ComplexVector matched_filter;
  if (!session::BuildWaveformAndFilter(config, &waveform, &matched_filter)) {
    return false;
  }

  runtime::PulseRingBuffer pulse_buffer(config.mission.azimuth_pulse_count);
  std::uint64_t next_pulse_id = 0U;
  double pulse_fraction_carry = 0.0;
  std::deque<geometry::PlatformPulseState> ideal_trajectory;
  std::deque<geometry::PlatformPulseState> actual_trajectory;
  session::SarCycleResult build_result;
  double estimated_snr_db = 0.0;
  return session::BuildRawPulseHistory(config, MakeL1Input(), waveform.samples, &pulse_buffer,
                                       &next_pulse_id, &pulse_fraction_carry, raw_history,
                                       &ideal_trajectory, &actual_trajectory, &estimated_snr_db,
                                       &build_result);
}

bool BuildGeometryProbe(const config::SarSessionConfig& config, const session::SarCycleInput& input,
                        std::deque<geometry::PlatformPulseState>* actual_trajectory) {
  signal::LfmWaveform waveform;
  signal::ComplexVector matched_filter;
  if (!session::BuildWaveformAndFilter(config, &waveform, &matched_filter)) {
    return false;
  }

  runtime::PulseRingBuffer pulse_buffer(config.mission.azimuth_pulse_count);
  std::uint64_t next_pulse_id = 0U;
  double pulse_fraction_carry = 0.0;
  signal::ComplexMatrix raw_history;
  std::deque<geometry::PlatformPulseState> ideal_trajectory;
  session::SarCycleResult build_result;
  double estimated_snr_db = 0.0;
  return session::BuildRawPulseHistory(
      config, input, waveform.samples, &pulse_buffer, &next_pulse_id, &pulse_fraction_carry,
      &raw_history, &ideal_trajectory, actual_trajectory, &estimated_snr_db, &build_result);
}

TEST(SarOmegaKL1RawHistoryStageATest, RejectsGeneratedL1ApertureAtGridReductionDeterministically) {
  const config::SarSessionConfig config = MakeL1Config();
  signal::ComplexMatrix raw_history;
  ASSERT_TRUE(BuildL1RawHistory(config, &raw_history));
  ASSERT_EQ(raw_history.rows, config.mission.azimuth_pulse_count);
  ASSERT_EQ(raw_history.cols, config.mission.range_sample_count);

  FocusedOmegaKImage first;
  FocusedOmegaKImage second;
  const OmegaKConfig omega_k = MakeOmegaKConfig(config);
  EXPECT_FALSE(FocusStripmapOmegaK(omega_k, raw_history, &first));
  EXPECT_FALSE(FocusStripmapOmegaK(omega_k, raw_history, &second));
  EXPECT_EQ(first.diagnostics.failure_stage, "grid_reduction");
  EXPECT_EQ(second.diagnostics.failure_stage, "grid_reduction");
  EXPECT_EQ(first.diagnostics.grid_reduction.reason,
            OmegaKGridReductionReason::kInvalidCommonSupport);
  EXPECT_EQ(first.diagnostics.common_support.common_valid_column_count, 0U);
  EXPECT_FALSE(first.diagnostics.common_support.usable_for_interpolation);
}

TEST(SarOmegaKL1RawHistoryStageATest, CompatibleVelocityRestoresCommonSupportAndFocusing) {
  config::SarSessionConfig config = MakeL1Config();
  config.mission.platform_speed_mps = 5.0;
  signal::ComplexMatrix raw_history;
  ASSERT_TRUE(BuildL1RawHistory(config, &raw_history));

  FocusedOmegaKImage first;
  FocusedOmegaKImage second;
  const OmegaKConfig omega_k = MakeOmegaKConfig(config);
  ASSERT_TRUE(FocusStripmapOmegaK(omega_k, raw_history, &first));
  ASSERT_TRUE(FocusStripmapOmegaK(omega_k, raw_history, &second));
  EXPECT_EQ(first.diagnostics.failure_stage, "none");
  EXPECT_TRUE(first.diagnostics.common_support.usable_for_interpolation);
  EXPECT_GE(first.diagnostics.common_support.largest_contiguous_column_count, 2U);
  EXPECT_EQ(first.image.values, second.image.values);
}

TEST(SarOmegaKL1RawHistoryStageATest, EnvironmentSelectsTerrainDatumAndEarthGeometry) {
  config::SarSessionConfig flat_config = MakeL1Config();
  flat_config.mission.scene_center_latitude_deg = 75.0;
  flat_config.mission.scene_center_longitude_deg = 0.0;
  flat_config.mission.scene_center_altitude_m = 1000.0;
  flat_config.environment.terrain_reference_altitude_m = 500.0;
  flat_config.environment.use_flat_earth_geometry = true;

  session::SarCycleInput input;
  input.dt_sec = 0.1f;
  input.platform.latitude_deg = 75.0;
  input.platform.longitude_deg = 1.0;
  input.platform.altitude_m = 2500.0;

  std::deque<geometry::PlatformPulseState> flat_trajectory;
  ASSERT_TRUE(BuildGeometryProbe(flat_config, input, &flat_trajectory));
  ASSERT_FALSE(flat_trajectory.empty());
  EXPECT_DOUBLE_EQ(flat_trajectory.front().position_m.z_m, 2000.0);

  config::SarSessionConfig curved_config = flat_config;
  curved_config.environment.use_flat_earth_geometry = false;
  std::deque<geometry::PlatformPulseState> curved_trajectory;
  ASSERT_TRUE(BuildGeometryProbe(curved_config, input, &curved_trajectory));
  ASSERT_EQ(curved_trajectory.size(), flat_trajectory.size());
  EXPECT_LT(curved_trajectory.front().position_m.z_m,
            flat_trajectory.front().position_m.z_m - 10.0);
  EXPECT_NE(curved_trajectory.front().position_m.x_m,
            flat_trajectory.front().position_m.x_m);
}

}  // namespace
}  // namespace imaging
}  // namespace sar
