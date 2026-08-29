// Copyright 2026. All Rights Reserved.
//
// @file rir_emission_frame_test.cpp
// @brief 验证 RIR 公开 API 输出本周期实际发射（与 AR emission_frame 同契约）。

#include <gtest/gtest.h>

#include "1q/remote_identification_radar/config/RirSessionConfig.h"
#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "1q/remote_identification_radar/session/RirSession.h"
#include "RirCycleInputTestUtil.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using config::RirSessionConfig;
using config::RirWorkMode;
using session::RirCycleInput;
using session::RirCycleStatus;
using session::RirSession;

RirCycleInput MakeInput(std::uint32_t cycle) {
  RirCycleInput input;
  input.input_cycle_index = cycle;
  input.dt_sec = 0.5;
  input.sim_time_sec = static_cast<float>(cycle - 1U) * 0.5f;
  SetDefaultTestPlatformEcef(&input);
  session::RirSceneTarget target;
  target.external_target_id = 9U;
  target.position_x = 8000.0f;
  target.rcs = 2.0f;
  input.scene_targets.push_back(target);
  return input;
}

RirSessionConfig MakeIdentifyConfig() {
  RirSessionConfig session_config;
  session_config.sensor_platform_id = 42U;
  session_config.mission.work_mode = RirWorkMode::kIdentify;
  session_config.policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
  return session_config;
}

TEST(RirEmissionFrameTest, CompletedIdentifyCyclePublishesEmissionFrame) {
  const RirSessionConfig session_config = MakeIdentifyConfig();
  RirSession session = RirSession::Create(session_config);
  const RirCycleInput input = MakeInput(1U);

  const auto result = session.StepWithResult(input);
  ASSERT_EQ(result.status, RirCycleStatus::kCompleted);
  EXPECT_EQ(result.emission_frame.world_cycle_index, 1U);
  EXPECT_DOUBLE_EQ(result.emission_frame.window_start_time_s, 0.0);
  EXPECT_DOUBLE_EQ(result.emission_frame.window_duration_s,
                   static_cast<double>(session_config.mission.recognition_dwell_sec));
  ASSERT_EQ(result.emission_frame.emissions.size(), 1U);
  EXPECT_EQ(result.emission_frame.emissions.front().identity.platform_id, 42U);
  EXPECT_EQ(result.emission_frame.emissions.front().identity.equipment_id,
            session_config.hardware.transmitter.equipment_id);
  EXPECT_GT(result.emission_frame.emissions.front().waveform.transmit_power_w, 0.0);
}

TEST(RirEmissionFrameTest, StandbyCycleLeavesEmissionFrameEmpty) {
  RirSessionConfig session_config = MakeIdentifyConfig();
  session_config.mission.work_mode = RirWorkMode::kStby;
  RirSession session = RirSession::Create(session_config);

  const auto result = session.StepWithResult(MakeInput(1U));
  ASSERT_EQ(result.status, RirCycleStatus::kCompleted);
  EXPECT_TRUE(result.emission_frame.emissions.empty());
}

TEST(RirEmissionFrameTest, ValidationRejectLeavesEmissionFrameEmpty) {
  RirSession session = RirSession::Create(MakeIdentifyConfig());
  RirCycleInput input = MakeInput(0U);
  input.input_cycle_index = 0U;

  const auto result = session.StepWithResult(input);
  EXPECT_EQ(result.status, RirCycleStatus::kRejectedInvalidInput);
  EXPECT_TRUE(result.emission_frame.emissions.empty());
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
