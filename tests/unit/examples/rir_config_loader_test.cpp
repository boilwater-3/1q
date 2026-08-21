#include <gtest/gtest.h>

#include <string>

#include "1q/remote_identification_radar/config/RirMissionConfig.h"
#include "1q/remote_identification_radar/config/RirPolicyConfig.h"
#include "config_loaders/remote_identification_radar/config_loader.h"

namespace examples {
namespace {

TEST(RirConfigLoaderTest, LoadsDeliveredSessionJson) {
#if !defined(SCENE_CONFIG_DIR)
  GTEST_SKIP() << "SCENE_CONFIG_DIR not defined";
#endif
  remote_identification_radar::config::RirSessionConfig config;
  std::string error;
  ASSERT_TRUE(LoadRirSessionConfigFromFile(SCENE_CONFIG_DIR "/remote_identification_radar.json",
                                           &config, &error))
      << error;

  EXPECT_EQ(config.mission.work_mode,
            remote_identification_radar::config::RirWorkMode::kIdentify);
  EXPECT_EQ(config.policy.detection.gate_mode,
            remote_identification_radar::config::RirDetectionGateMode::kSnrFallback);
  EXPECT_TRUE(config.policy.recognition.enabled);
  EXPECT_EQ(config.policy.lifecycle.confirm_hits, 1U);
  EXPECT_EQ(config.policy.recognition.min_confirmed_hits, 1U);
  EXPECT_EQ(config.policy.recognition.min_observation_count, 1U);
  EXPECT_FLOAT_EQ(config.policy.recognition.acceptance_score, 0.6f);
  EXPECT_FLOAT_EQ(config.policy.recognition.minimum_margin, 0.05f);
  EXPECT_EQ(config.policy.recognition.database_path,
            "remote_identification_radar/target_feature_database_v1.1.db");
  EXPECT_TRUE(config.sensor_enabled);
  EXPECT_FLOAT_EQ(config.orientation.steerable_volume_deg.az_min_deg, -110.0f);
  EXPECT_FLOAT_EQ(config.orientation.steerable_volume_deg.az_max_deg, 110.0f);
  EXPECT_FLOAT_EQ(config.orientation.steerable_volume_deg.el_min_deg, 2.0f);
  EXPECT_FLOAT_EQ(config.orientation.steerable_volume_deg.el_max_deg, 85.0f);
  EXPECT_FLOAT_EQ(config.mission.scan_center_deg.az_deg, 0.0f);
  EXPECT_FLOAT_EQ(config.mission.scan_center_deg.el_deg, 0.0f);
}

}  // namespace
}  // namespace examples
