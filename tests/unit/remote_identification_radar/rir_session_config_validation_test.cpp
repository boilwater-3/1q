// Copyright 2026. All Rights Reserved.
//
// @file rir_session_config_validation_test.cpp
// @brief 验证 RIR 会话配置 hardware 域校验（对标 AR ValidateArSessionConfig hardware 段）。

#include <gtest/gtest.h>

#include <limits>

#include "1q/remote_identification_radar/config/RirSessionConfigValidation.h"
#include "1q/remote_identification_radar/session/RirIssueCodes.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using config::RirSessionConfig;
using config::ValidateRirSessionConfig;

bool HasCode(const session::RirIssueList& issues, const char* code) {
  for (const session::RirIssue& issue : issues) {
    if (issue.code == code) {
      return true;
    }
  }
  return false;
}

TEST(RirSessionConfigValidationTest, RejectsZeroSensorPlatformId) {
  RirSessionConfig session_config;
  session_config.sensor_platform_id = 0U;
  const auto issues = ValidateRirSessionConfig(session_config);
  EXPECT_TRUE(HasCode(issues, session::codes::kSensorPlatformIdInvalid));
}

TEST(RirSessionConfigValidationTest, AcceptsPhysicalApertureWhenNominalBeamwidthIsZero) {
  RirSessionConfig session_config;
  session_config.hardware.antenna.nominal_az_beamwidth_deg = 0.0f;
  session_config.hardware.antenna.antenna_length_m = 1.5f;
  session_config.hardware.antenna.nominal_el_beamwidth_deg = 0.0f;
  session_config.hardware.antenna.antenna_width_m = 0.8f;
  EXPECT_FALSE(HasCode(ValidateRirSessionConfig(session_config),
                        session::codes::kAntennaAzGeometryInvalid));
  EXPECT_FALSE(HasCode(ValidateRirSessionConfig(session_config),
                        session::codes::kAntennaElGeometryInvalid));
}

TEST(RirSessionConfigValidationTest, RejectsMissingOrNonFiniteAntennaGeometry) {
  RirSessionConfig session_config;
  session_config.hardware.antenna.nominal_az_beamwidth_deg = 0.0f;
  session_config.hardware.antenna.antenna_length_m = 0.0f;
  session_config.hardware.antenna.antenna_width_m = std::numeric_limits<float>::quiet_NaN();
  const auto issues = ValidateRirSessionConfig(session_config);
  EXPECT_TRUE(HasCode(issues, session::codes::kAntennaAzGeometryInvalid));
  EXPECT_TRUE(HasCode(issues, session::codes::kAntennaElGeometryInvalid));
}

TEST(RirSessionConfigValidationTest, RejectsInvalidTransmitterFrequency) {
  RirSessionConfig session_config;
  session_config.hardware.transmitter.frequency_hz = 0.0f;
  const auto issues = ValidateRirSessionConfig(session_config);
  EXPECT_TRUE(HasCode(issues, session::codes::kTransmitterFrequencyInvalid));
  EXPECT_TRUE(HasCode(issues, session::codes::kFrequencyPlanInvalid));
}

TEST(RirSessionConfigValidationTest, ValidatesTransmitterEnvelopeAndIdentity) {
  RirSessionConfig session_config;
  session_config.hardware.transmitter.frequency_plan_hz = {3.0e9, 3.1e9};
  EXPECT_FALSE(HasCode(ValidateRirSessionConfig(session_config),
                        session::codes::kFrequencyPlanInvalid));

  session_config.hardware.transmitter.frequency_plan_hz = {3.1e9};
  EXPECT_TRUE(HasCode(ValidateRirSessionConfig(session_config),
                       session::codes::kFrequencyPlanInvalid));

  session_config.hardware.transmitter.frequency_plan_hz = {3.0e9};
  session_config.hardware.transmitter.maximum_peak_power_w = 5.0e5f;
  EXPECT_TRUE(HasCode(ValidateRirSessionConfig(session_config),
                       session::codes::kTransmitterOperatingEnvelopeInvalid));

  session_config.hardware.transmitter.maximum_peak_power_w = 1.2e6f;
  session_config.hardware.receiver.equipment_id =
      session_config.hardware.transmitter.equipment_id;
  EXPECT_TRUE(HasCode(ValidateRirSessionConfig(session_config),
                       session::codes::kEquipmentIdentityInvalid));
}

TEST(RirSessionConfigValidationTest, ValidatesReceiverRfHardwareBoundary) {
  RirSessionConfig session_config;
  session_config.hardware.receiver.maximum_linear_input_power_w = 0.0f;
  EXPECT_TRUE(HasCode(ValidateRirSessionConfig(session_config),
                       session::codes::kReceiverRfHardwareInvalid));

  session_config.hardware.receiver.maximum_linear_input_power_w = 1.0e-3f;
  session_config.hardware.receiver.interference_observation_jn_gate_db =
      std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(HasCode(ValidateRirSessionConfig(session_config),
                       session::codes::kReceiverRfHardwareInvalid));
}

TEST(RirSessionConfigValidationTest, RejectsInvalidRcsPhysics) {
  RirSessionConfig session_config;
  session_config.hardware.rcs_physics.physics_mix_ratio = 1.5f;
  EXPECT_TRUE(HasCode(ValidateRirSessionConfig(session_config),
                       session::codes::kRcsPhysicsInvalid));
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
