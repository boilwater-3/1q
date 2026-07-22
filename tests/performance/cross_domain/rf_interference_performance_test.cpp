#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <vector>

#include "1q/airborne_radar/airborne_radar.hpp"
#include "1q/electromagnetics/RfLinkBudget.h"
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

std::vector<rf::RfEmission> MakeRfEmissions() {
  std::vector<rf::RfEmission> emissions;
  emissions.reserve(kRfEmissionCount);
  for (std::size_t i = 0U; i < kRfEmissionCount; ++i) {
    rf::RfEmission emission;
    emission.emission_id = 10000U + i;
    emission.entity_id = 20000U + i;
    emission.position_ecef_m.x_m = 6378137.0 + 20000.0 + 100.0 * static_cast<double>(i);
    emission.position_ecef_m.y_m = 500.0 * static_cast<double>(i % 8U);
    emission.antenna.boresight_ecef_unit.x = -1.0;
    emission.antenna.peak_gain_dbi = 10.0;
    emission.polarization = rf::RfPolarization::kHorizontal;
    emission.waveform_kind = rf::RfWaveformKind::kNoise;
    rf::RfEmissionSegment segment;
    segment.duration_s = 1.0;
    segment.center_frequency_hz = 9.3e9 + 1.0e6 * static_cast<double>(i % 16U);
    segment.bandwidth_hz = 5.0e6;
    segment.transmit_power_w = 100.0;
    emission.segments.push_back(segment);
    emissions.push_back(emission);
  }
  return emissions;
}

ar_session::ArCycleInput MakeArInput(const std::vector<rf::RfEmission>& emissions) {
  ar_session::ArCycleInput input;
  input.dt_sec = 1.0f;
  input.platform_entity_id = 1U;
  input.has_platform_ecef_kinematics = true;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.has_environment = true;
  input.environment.interference.mode = rf::RfInterferenceMode::kEngineering;
  input.environment.interference.engineering_emissions = emissions;
  input.scene.reserve(kArTargetCount);
  for (std::size_t i = 0U; i < kArTargetCount; ++i) {
    ar_session::ArSceneTarget target;
    target.external_target_id = i + 1U;
    target.position_x = 10000.0f + 50.0f * static_cast<float>(i);
    target.position_y = 10.0f * static_cast<float>(static_cast<int>(i % 21U) - 10);
    target.position_z = 5.0f * static_cast<float>(static_cast<int>(i % 11U) - 5);
    target.range_m = std::sqrt(target.position_x * target.position_x +
                               target.position_y * target.position_y +
                               target.position_z * target.position_z);
    target.rcs = 10.0f;
    input.scene.push_back(target);
  }
  return input;
}

esr_session::EsrCycleInput MakeEsrInput(const std::vector<rf::RfEmission>& emissions) {
  esr_session::EsrCycleInput input;
  input.dt_sec = 1.0f;
  input.platform_entity_id = 1U;
  input.has_platform_ecef_kinematics = true;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.environment.interference_mode = rf::RfInterferenceMode::kEngineering;
  input.environment.engineering_emissions = emissions;
  input.scene.reserve(kEsrEmitterCount);
  for (std::size_t i = 0U; i < kEsrEmitterCount; ++i) {
    esr_session::EsrSceneEmitter emitter;
    emitter.emitter_id = i + 1U;
    emitter.has_ecef_kinematics = true;
    emitter.position_ecef_m.x_m = 6378137.0 + 10000.0 + 50.0 * static_cast<double>(i);
    emitter.position_ecef_m.y_m = 10.0 * static_cast<double>(static_cast<int>(i % 21U) - 10);
    emitter.pose.position_m.x = static_cast<double>(10000U + 50U * i);
    emitter.pose.position_m.y = static_cast<double>(static_cast<int>(i % 21U) - 10) * 10.0;
    emitter.carrier_hz = 9.3e9 + 1.0e6 * static_cast<double>(i % 16U);
    emitter.bandwidth_hz = 5.0e6;
    emitter.tx_power_w = 1.0;
    emitter.pulse_width_s = 1.0e-6;
    emitter.pri_s = 1.0e-3;
    input.scene.push_back(emitter);
  }
  return input;
}

TEST(RfInterferencePerformanceTest, FullScaleCyclesMeetReleaseP95Budget) {
  const std::vector<rf::RfEmission> emissions = MakeRfEmissions();
  ASSERT_EQ(emissions.size(), kRfEmissionCount);

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

  ar_session::ArCycleInput ar_input = MakeArInput(emissions);
  esr_session::EsrCycleInput esr_input = MakeEsrInput(emissions);
  std::vector<double> elapsed_ms;
  elapsed_ms.reserve(kMeasuredCycles);

  for (std::uint32_t cycle = 1U; cycle <= kWarmupCycles + kMeasuredCycles; ++cycle) {
    ar_input.cycle_index = cycle;
    esr_input.cycle_index = cycle;
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
