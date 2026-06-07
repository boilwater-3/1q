/**
 * @file esr_session_test.cpp
 * @brief ESR Session 集成测试（高层语义配置版本）。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include "1q/electronic_surveillance_radar/config/EsrRuntimeConfigBuilder.h"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfigBuilder.h"
#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentTypes.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"
#include "1q/electronic_surveillance_radar/session/EsrSessionFactory.h"

namespace electronic_surveillance_radar {
namespace session {
namespace {

namespace esr_config = ::electronic_surveillance_radar::config;

EsrCycleInput MakeBaseInput() {
  EsrCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0f;
  input.platform_altitude_m = 5000.0f;
  input.platform_pose.position_m.z = 5000.0f;

  session::EsrSceneEmitter emitter;
  emitter.emitter_id = 1001U;
  emitter.pose.position_m.x = 1200.0f;
  emitter.pose.position_m.z = 5200.0f;
  emitter.carrier_hz = 10.0e9;
  emitter.bandwidth_hz = 2.0e6;
  emitter.tx_power_w = 5.0e7;
  emitter.pulse_width_s = 1.2e-6;
  emitter.pri_s = 1.0e-4;
  emitter.is_emitting = true;
  input.scene.push_back(emitter);
  return input;
}

config::EsrSessionConfig MakeSessionConfig() {
  config::EsrSessionConfig config = esr_config::EsrSessionConfigBuilder()
                                .Detection()
                                .WithMinDetectSnrDb(6.0f)
                                .End()
                                .Environment()
                                .WithEnvironmentPreset(esr_config::EsrEnvironmentPreset::kStandard)
                                .End()
                                .Build();
  config.mission.scan.use_explicit_scan_bounds = true;
  config.mission.scan.scan_start_az_deg = -60.0f;
  config.mission.scan.scan_end_az_deg = 60.0f;
  config.mission.scan.scan_start_el_deg = -20.0f;
  config.mission.scan.scan_end_el_deg = 20.0f;
  config.hardware.beam_az_width_deg = 120.0f;
  config.hardware.beam_el_width_deg = 40.0f;
  return config;
}

std::size_t CountMatchedTruthObservations(const EsrCycleResult& result,
                                          std::uint64_t truth_id) {
  std::size_t matched_count = 0U;
  for (std::size_t i = 0; i < result.output_frame.truth_evaluation_output.associations.size();
       ++i) {
    const extension::TruthAssociationRecord& association =
        result.output_frame.truth_evaluation_output.associations[i];
    if (association.matched && association.truth_emitter_id == truth_id) {
      ++matched_count;
    }
  }
  return matched_count;
}

std::size_t CountMatchedAcrossCycles(EsrSession* session, const EsrCycleInput& base_input,
                                     std::uint64_t truth_id, std::size_t cycle_count) {
  if (session == nullptr) {
    return 0U;
  }
  std::size_t total = 0U;
  for (std::size_t i = 0; i < cycle_count; ++i) {
    EsrCycleInput input = base_input;
    input.cycle_index = base_input.cycle_index + static_cast<std::uint32_t>(i);
    total += CountMatchedTruthObservations(session->StepWithResult(input), truth_id);
  }
  return total;
}

TEST(EsrSessionIntegrationTest, StepWithResultProducesThreeChannelOutput) {
  EsrSession session = EsrSessionFactory::Create(MakeSessionConfig());
  const EsrCycleResult result = session.StepWithResult(MakeBaseInput());

  EXPECT_FALSE(result.has_validation_error);
  EXPECT_EQ(result.output_frame.cycle_index, 1U);
}

TEST(EsrSessionIntegrationTest, WorkModeMappingMakesHgesmMoreDetectableThanRwr) {
  config::EsrSessionConfig hgesm_config = MakeSessionConfig();
  hgesm_config.mission.work_mode = config::EsrWorkMode::kHgesm;
  config::EsrSessionConfig rwr_config = hgesm_config;
  rwr_config.mission.work_mode = config::EsrWorkMode::kRwr;

  EsrCycleInput input = MakeBaseInput();
  input.scene.front().tx_power_w = 30.0;

  auto hgesm_session = EsrSessionFactory::Create(hgesm_config);
  auto rwr_session = EsrSessionFactory::Create(rwr_config);
  const std::size_t hgesm_matched =
      CountMatchedAcrossCycles(&hgesm_session, input, 1001U, 24U);
  const std::size_t rwr_matched =
      CountMatchedAcrossCycles(&rwr_session, input, 1001U, 24U);

  EXPECT_GE(hgesm_matched, rwr_matched);
}

TEST(EsrSessionIntegrationTest, MissionPowerOffReturnsEmptyChannels) {
  config::EsrSessionConfig config = MakeSessionConfig();
  config.mission.power_on = false;

  EsrSession session = EsrSessionFactory::Create(config);
  const EsrCycleResult result = session.StepWithResult(MakeBaseInput());
  EXPECT_TRUE(result.output_frame.observation_output.observations.empty());
  EXPECT_TRUE(result.output_frame.emitter_output.hypotheses.empty());
  EXPECT_TRUE(result.output_frame.truth_evaluation_output.associations.empty());
}

TEST(EsrSessionIntegrationTest, RuntimePatchCanDisableSensorWithoutReconstruction) {
  EsrSession session = EsrSessionFactory::Create(MakeSessionConfig());
  const EsrCycleResult baseline = session.StepWithResult(MakeBaseInput());
  EXPECT_GT(CountMatchedTruthObservations(baseline, 1001U), 0U);

  const config::EsrRuntimeConfigPatch patch =
      esr_config::EsrRuntimeConfigBuilder().WithSensorEnabled(false).Build();
  session.ApplyRuntimeConfig(patch);

  const EsrCycleResult updated = session.StepWithResult(MakeBaseInput());
  EXPECT_TRUE(updated.output_frame.observation_output.observations.empty());
}

TEST(EsrSessionIntegrationTest, RuntimePatchCanApplyExplicitScanBounds) {
  EsrSession session = EsrSessionFactory::Create(MakeSessionConfig());

  EsrCycleInput input = MakeBaseInput();
  input.scene.front().pose.position_m.x = 1200.0f;
  input.scene.front().pose.position_m.y = 0.0f;

  const EsrCycleResult baseline = session.StepWithResult(input);
  EXPECT_GT(CountMatchedTruthObservations(baseline, 1001U), 0U);

  const config::EsrRuntimeConfigPatch block_patch =
      esr_config::EsrRuntimeConfigBuilder()
          .WithExplicitScanBoundsDeg(-180.0f, -150.0f, -20.0f, 20.0f)
          .Build();
  session.ApplyRuntimeConfig(block_patch);
  const EsrCycleResult blocked = session.StepWithResult(input);
  EXPECT_EQ(CountMatchedTruthObservations(blocked, 1001U), 0U);
}

TEST(EsrSessionIntegrationTest, InvalidRuntimePatchRejectedAtomically) {
  EsrSession session = EsrSessionFactory::Create(MakeSessionConfig());
  const EsrCycleResult baseline = session.StepWithResult(MakeBaseInput());
  const std::size_t baseline_matched = CountMatchedTruthObservations(baseline, 1001U);

  config::EsrRuntimeConfigPatch invalid_patch;
  invalid_patch.has_explicit_scan_bounds = true;
  invalid_patch.explicit_scan_bounds.enabled = true;
  
  
  
  
  invalid_patch.explicit_scan_bounds.scan_start_az_deg = std::numeric_limits<float>::quiet_NaN();
  invalid_patch.explicit_scan_bounds.scan_end_az_deg = 10.0f;
  invalid_patch.explicit_scan_bounds.scan_start_el_deg = -10.0f;
  invalid_patch.explicit_scan_bounds.scan_end_el_deg = 10.0f;
  session.ApplyRuntimeConfig(invalid_patch);

  const EsrCycleResult after_invalid = session.StepWithResult(MakeBaseInput());
  EXPECT_EQ(CountMatchedTruthObservations(after_invalid, 1001U), baseline_matched);
}

}  // namespace
}  // namespace session
}  // namespace electronic_surveillance_radar
