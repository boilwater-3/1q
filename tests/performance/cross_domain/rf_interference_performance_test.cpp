#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <vector>

#include "1q/airborne_radar/airborne_radar.hpp"
#include "1q/electromagnetics/RfScene.h"
#include "1q/electronic_surveillance_radar/electronic_surveillance_radar.hpp"

namespace {

namespace ar_config = airborne_radar::config;
namespace ar_session = airborne_radar::session;
namespace esr_config = electronic_surveillance_radar::config;
namespace esr_session = electronic_surveillance_radar::session;
namespace rf = oneq::electromagnetics;

constexpr std::size_t kRfEmissionCount = 64U;
constexpr std::size_t kArTargetCount = 1000U;
constexpr std::size_t kEsrEmitterCount = 1000U;
constexpr std::uint32_t kWarmupCycles = 5U;
constexpr std::uint32_t kMeasuredCycles = 100U;
constexpr double kP95LimitMilliseconds = 100.0;

rf::RfEmissionFrame MakeRfEmissions(std::uint32_t cycle_index,
                                    double cycle_start_time_s,
                                    std::size_t emission_count,
                                    std::uint64_t identity_base) {
  rf::RfEmissionFrame frame;
  frame.world_cycle_index = cycle_index;
  frame.window_start_time_s = cycle_start_time_s;
  frame.window_duration_s = 1.0;
  std::vector<rf::RfSceneEmission>& emissions = frame.emissions;
  emissions.reserve(emission_count);
  for (std::size_t i = 0U; i < emission_count; ++i) {
    rf::RfSceneEmission emission;
    emission.identity.platform_id = identity_base + i;
    emission.identity.equipment_id = 1U;
    emission.identity.emission_id = identity_base + 100000U + i;
    emission.position_ecef_m.x_m = 6378137.0 + 20000.0 + 100.0 * static_cast<double>(i);
    emission.position_ecef_m.y_m = 500.0 * static_cast<double>(i % 8U);
    emission.antenna.boresight_ecef.x = -1.0;
    emission.antenna.peak_gain_dbi = 10.0;
    emission.polarization = rf::RfScenePolarization::kHorizontal;
    EXPECT_TRUE(rf::TryCreateRfNoiseWaveform(
        cycle_start_time_s, frame.window_duration_s,
        9.3e9 + 1.0e6 * static_cast<double>(i % 16U), 5.0e6, 100.0,
        &emission.waveform));
    emissions.push_back(emission);
  }
  return frame;
}

ar_session::ArCycleInput MakeArInput() {
  ar_session::ArCycleInput input;
  input.dt_sec = 1.0;
  input.platform.platform_entity_id = 1U;
  input.platform.platform_position_ecef_m.x_m = 6378137.0;
  input.targets.reserve(kArTargetCount);
  for (std::size_t i = 0U; i < kArTargetCount; ++i) {
    ar_session::ArExternalTargetInput target;
    target.target_id = i + 1U;
    target.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
    target.kinematics.position_ecef_m.x_m =
        6378137.0 + 10000.0 + 50.0 * static_cast<double>(i);
    target.kinematics.position_ecef_m.y_m =
        10.0 * static_cast<double>(static_cast<int>(i % 21U) - 10);
    target.kinematics.position_ecef_m.z_m =
        5.0 * static_cast<double>(static_cast<int>(i % 11U) - 5);
    target.rcs = 10.0f;
    input.targets.push_back(target);
  }
  return input;
}

esr_session::EsrCycleInput MakeEsrInput() {
  esr_session::EsrCycleInput input;
  input.dt_sec = 1.0f;
  input.platform_entity_id = 1U;
  input.has_platform_ecef_kinematics = true;
  input.platform_position_ecef_m.x_m = 6378137.0;
  return input;
}

TEST(RfInterferencePerformanceTest, FullScaleCyclesMeetReleaseP95Budget) {
  const rf::RfEmissionFrame initial_ar_frame =
      MakeRfEmissions(1U, 0.0, kRfEmissionCount, 20000U);
  const rf::RfEmissionFrame initial_esr_frame =
      MakeRfEmissions(1U, 0.0, kEsrEmitterCount, 40000U);
  ASSERT_EQ(initial_ar_frame.emissions.size(), kRfEmissionCount);
  ASSERT_EQ(initial_esr_frame.emissions.size(), kEsrEmitterCount);

  ar_config::ArSessionConfig ar_config =
      ar_config::ArSessionConfigBuilder()
          .Detection()
          .WithHardwareProfile(ar_config::profiles::ArHardwareProfile::kLongRangeHighPower)
          .End()
          .Build();
  ar_config.hardware.receiver.maximum_linear_input_power_w = 1.0e9f;
  ar_session::ArSession ar = ar_session::ArSession::Create(ar_config);

  esr_config::EsrSessionConfig esr_config;
  esr_config.hardware.beam_az_width_deg = 120.0f;
  esr_config.hardware.beam_el_width_deg = 120.0f;
  esr_config.hardware.maximum_linear_input_power_w = 1.0e9f;
  esr_config.policy.detection.enable_statistical_detection = false;
  esr_session::EsrSession esr = esr_session::EsrSession::Create(esr_config);

  ar_session::ArCycleInput ar_input = MakeArInput();
  esr_session::EsrCycleInput esr_input = MakeEsrInput();
  std::vector<double> elapsed_ms;
  elapsed_ms.reserve(kMeasuredCycles);

  for (std::uint32_t cycle = 1U; cycle <= kWarmupCycles + kMeasuredCycles; ++cycle) {
    ar_input.cycle_index = cycle;
    ar_input.cycle_start_time_s = static_cast<double>(cycle - 1U);
    ar_input.interference =
        MakeRfEmissions(cycle, ar_input.cycle_start_time_s, kRfEmissionCount,
                        20000U);
    esr_input.cycle_index = cycle;
    esr_input.cycle_start_time_s = ar_input.cycle_start_time_s;
    esr_input.rf_emissions =
        MakeRfEmissions(cycle, esr_input.cycle_start_time_s, kEsrEmitterCount,
                        40000U);
    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    const ar_session::ArCycleResult ar_result = ar.StepWithResult(ar_input);
    const esr_session::EsrCycleResult esr_result = esr.StepWithResult(esr_input);
    const double duration_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    ASSERT_FALSE(ar_result.has_validation_error);
    ASSERT_FALSE(esr_result.has_validation_error);
    if (cycle > kWarmupCycles) {
      elapsed_ms.push_back(duration_ms);
    }
  }

  std::sort(elapsed_ms.begin(), elapsed_ms.end());
  const std::size_t p95_index = (95U * elapsed_ms.size() + 99U) / 100U - 1U;
  const double p95_ms = elapsed_ms[p95_index];
  RecordProperty("rf_emission_count", static_cast<int>(kRfEmissionCount));
  RecordProperty("ar_target_count", static_cast<int>(kArTargetCount));
  RecordProperty("esr_emitter_count", static_cast<int>(kEsrEmitterCount));
  RecordProperty("measured_cycle_count", static_cast<int>(kMeasuredCycles));
  RecordProperty("p95_milliseconds", p95_ms);
  EXPECT_LT(p95_ms, kP95LimitMilliseconds);
}

}  // namespace
