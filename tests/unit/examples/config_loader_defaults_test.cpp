// config_loader 缺键门控：缺省字段保持库结构体默认值（Has 门控，非 0.0 覆写）。

#include <cstdio>
#include <fstream>
#include <string>

#include "gtest/gtest.h"

#include "config_loaders/electro_optical/config_loader.h"
#include "config_loaders/remote_identification_radar/config_loader.h"
#include "config_loaders/sar/config_loader.h"
#include "config_loaders/sbirs_sensor/config_loader.h"

namespace examples {
namespace {

const char* kTestJsonPath = "oneq_config_loader_defaults_test.json";

class ConfigLoaderDefaultsTest : public ::testing::Test {
 protected:
  void TearDown() override { std::remove(kTestJsonPath); }

  bool LoadJson(const std::string& text) {
    std::ofstream out(kTestJsonPath, std::ios::binary);
    out << text;
    return out.good();
  }
};

TEST_F(ConfigLoaderDefaultsTest, EosEmptyRootKeepsAllDefaults) {
  ASSERT_TRUE(LoadJson("{}"));
  electro_optical_sensor::config::EosSessionConfig config;
  std::string error;
  ASSERT_TRUE(LoadEosSessionConfigFromFile(kTestJsonPath, &config, &error)) << error;
  EXPECT_FLOAT_EQ(config.hardware.wavelength_lower_um, 3.0f);
  EXPECT_FLOAT_EQ(config.hardware.wavelength_upper_um, 5.0f);
  EXPECT_FLOAT_EQ(config.hardware.optical_aperture_m, 0.2f);
  EXPECT_FLOAT_EQ(config.mission.frame_rate_hz, 30.0f);
  EXPECT_FLOAT_EQ(config.mission.scan_start_az_deg, -60.0f);
  EXPECT_FLOAT_EQ(config.mission.boresight_depression_deg, 45.0f);
  // 环境子块缺键同样保持库默认（1013.25 hPa / 288.15 K / 0.5）。
  EXPECT_FLOAT_EQ(
      config.environment.scenario_config.atmospheric_physics.pressure_hpa, 1013.25f);
  EXPECT_FLOAT_EQ(
      config.environment.scenario_config.atmospheric_physics.temperature_k, 288.15f);
}

TEST_F(ConfigLoaderDefaultsTest, EosPartialBlockKeepsSiblingDefaults) {
  ASSERT_TRUE(LoadJson(
      "{\"hardware\": {\"wavelength_lower_um\": 8.0},"
      " \"mission\": {\"frame_rate_hz\": 10.0}}"));
  electro_optical_sensor::config::EosSessionConfig config;
  std::string error;
  ASSERT_TRUE(LoadEosSessionConfigFromFile(kTestJsonPath, &config, &error)) << error;
  EXPECT_FLOAT_EQ(config.hardware.wavelength_lower_um, 8.0f);
  EXPECT_FLOAT_EQ(config.hardware.wavelength_upper_um, 5.0f);
  EXPECT_FLOAT_EQ(config.mission.frame_rate_hz, 10.0f);
  EXPECT_FLOAT_EQ(config.mission.scan_start_az_deg, -60.0f);
}

TEST_F(ConfigLoaderDefaultsTest, EosAtmospherePartialKeyKeepsSiblings) {
  ASSERT_TRUE(LoadJson(
      "{\"environment\": {\"scenario_config\":"
      " {\"atmospheric_physics\": {\"pressure_hpa\": 900.0}}}}"));
  electro_optical_sensor::config::EosSessionConfig config;
  std::string error;
  ASSERT_TRUE(LoadEosSessionConfigFromFile(kTestJsonPath, &config, &error)) << error;
  EXPECT_FLOAT_EQ(
      config.environment.scenario_config.atmospheric_physics.pressure_hpa, 900.0f);
  EXPECT_FLOAT_EQ(
      config.environment.scenario_config.atmospheric_physics.temperature_k, 288.15f);
  EXPECT_FLOAT_EQ(
      config.environment.scenario_config.atmospheric_physics.relative_humidity, 0.5f);
}

TEST_F(ConfigLoaderDefaultsTest, SarMissionMissingKeysKeepDefaults) {
  ASSERT_TRUE(LoadJson("{\"mission\": {\"platform_speed_mps\": 200.0}}"));
  sar::config::SarSessionConfig config;
  std::string error;
  ASSERT_TRUE(LoadSarSessionConfigFromFile(kTestJsonPath, &config, &error)) << error;
  EXPECT_FLOAT_EQ(config.mission.platform_speed_mps, 200.0);
  EXPECT_DOUBLE_EQ(config.mission.nominal_slant_range_m, 15000.0);
  EXPECT_EQ(config.mission.range_sample_count, 4096U);
}

TEST_F(ConfigLoaderDefaultsTest, SbirsEmptyRootKeepsPoweredOnAndFocalDefaults) {
  ASSERT_TRUE(LoadJson("{}"));
  sbirs_sensor::config::SbirsSessionConfig config;
  std::string error;
  ASSERT_TRUE(LoadSbirsSessionConfigFromFile(kTestJsonPath, &config, &error)) << error;
  // sensor_enabled 缺键曾静默置 false（传感器掉电）；门控后保持库默认 true。
  EXPECT_TRUE(config.sensor_enabled);
  EXPECT_FLOAT_EQ(config.hardware.focal_length_m, 2.0f);
  EXPECT_FLOAT_EQ(config.hardware.detector_pixel_pitch_m, 3.0e-5f);
}

TEST_F(ConfigLoaderDefaultsTest, RirEmptyRootKeepsPoweredOn) {
  ASSERT_TRUE(LoadJson("{}"));
  remote_identification_radar::config::RirSessionConfig config;
  std::string error;
  ASSERT_TRUE(LoadRirSessionConfigFromFile(kTestJsonPath, &config, &error)) << error;
  EXPECT_TRUE(config.sensor_enabled);
}

}  // namespace
}  // namespace examples
