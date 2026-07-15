#include <gtest/gtest.h>

#include <limits>

#include "1q/sbirs_sensor/config/SbirsSessionConfigBuilder.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfigValidation.h"

namespace {

TEST(SbirsSessionConfigBuilderTest, BuildsFourDomainConfiguration) {
  sbirs_sensor::config::SbirsHardwareConfig hardware;
  hardware.optical_aperture_m = 0.7f;
  sbirs_sensor::config::SbirsMissionConfig mission;
  mission.work_mode = sbirs_sensor::config::SbirsWorkMode::kSearchAndStare;
  mission.scan_rate_deg_per_sec = 2.0f;
  sbirs_sensor::config::SbirsPolicyConfig policy;
  policy.detection.wide_min_snr_linear = 3.0f;
  sbirs_sensor::config::SbirsEnvironmentConfig environment;
  environment.weather_type = sbirs_sensor::config::SbirsWeatherType::kCloudy;

  const sbirs_sensor::config::SbirsSessionConfig config =
      sbirs_sensor::config::SbirsSessionConfigBuilder()
          .WithHardware(hardware)
          .WithMission(mission)
          .WithPolicy(policy)
          .WithEnvironment(environment)
          .Build();

  EXPECT_FLOAT_EQ(config.hardware.optical_aperture_m, 0.7f);
  EXPECT_EQ(config.mission.work_mode, sbirs_sensor::config::SbirsWorkMode::kSearchAndStare);
  EXPECT_FLOAT_EQ(config.mission.scan_rate_deg_per_sec, 2.0f);
  EXPECT_FLOAT_EQ(config.policy.detection.wide_min_snr_linear, 3.0f);
  EXPECT_EQ(config.environment.weather_type, sbirs_sensor::config::SbirsWeatherType::kCloudy);
  EXPECT_TRUE(sbirs_sensor::config::ValidateSbirsSessionConfig(config).empty());
}

TEST(SbirsSessionConfigBuilderTest, RejectsInvalidScanRate) {
  sbirs_sensor::config::SbirsMissionConfig mission;
  mission.scan_rate_deg_per_sec = -1.0f;
  const sbirs_sensor::config::SbirsSessionConfig config =
      sbirs_sensor::config::SbirsSessionConfigBuilder().WithMission(mission).Build();

  EXPECT_FALSE(sbirs_sensor::config::ValidateSbirsSessionConfig(config).empty());
}

TEST(SbirsSessionConfigBuilderTest, PointingDefaultsAreProductionValues) {
  const sbirs_sensor::config::SbirsMissionConfig mission;
  const sbirs_sensor::config::SbirsTrackingConfig tracking;

  EXPECT_FLOAT_EQ(mission.narrow_pointing_max_slew_rate_deg_per_sec, 30.0f);
  EXPECT_FLOAT_EQ(mission.narrow_pointing_settle_tolerance_deg, 0.01f);
  EXPECT_EQ(tracking.nfov_tracking_gate_loss_cycles, 2U);
}

TEST(SbirsSessionConfigBuilderTest, RejectsInvalidPointingParameters) {
  sbirs_sensor::config::SbirsSessionConfig config;
  config.mission.narrow_pointing_max_slew_rate_deg_per_sec = 0.0f;
  EXPECT_FALSE(sbirs_sensor::config::ValidateSbirsSessionConfig(config).empty());

  config.mission.narrow_pointing_max_slew_rate_deg_per_sec = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(sbirs_sensor::config::ValidateSbirsSessionConfig(config).empty());

  config.mission.narrow_pointing_max_slew_rate_deg_per_sec = 30.0f;
  config.mission.narrow_pointing_settle_tolerance_deg = -0.01f;
  EXPECT_FALSE(sbirs_sensor::config::ValidateSbirsSessionConfig(config).empty());

  config.mission.narrow_pointing_settle_tolerance_deg = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(sbirs_sensor::config::ValidateSbirsSessionConfig(config).empty());
}

TEST(SbirsSessionConfigBuilderTest, RejectsZeroTrackingGateLossCycles) {
  sbirs_sensor::config::SbirsSessionConfig config;
  config.policy.tracking.nfov_tracking_gate_loss_cycles = 0U;

  EXPECT_FALSE(sbirs_sensor::config::ValidateSbirsSessionConfig(config).empty());
}

}  // namespace
