// Copyright 2026. All Rights Reserved.
//
// @file ar_recognition_example_database_test.cpp
// @brief 验证提交入库的示例识别基线可被加载（建库工具生成物端到端）。
//
// 示例库 examples/configs/recognition/target_feature_database_v1.1.db 由
// tools/recognition_db_builder.py 从 recognition_database_input.json 生成；
// 本用例证明 工具 → 权威 DDL → 加载器 全链路一致。

#include <gtest/gtest.h>

#include "airborne_radar/recognition/RecognitionFeatureDatabase.h"

namespace airborne_radar {
namespace tests {
namespace {

using recognition::RecognitionFeatureDatabase;

#ifndef ONEQ_RECOGNITION_EXAMPLE_DATABASE_PATH
#error "ONEQ_RECOGNITION_EXAMPLE_DATABASE_PATH 未定义（Integration.cmake 注入）"
#endif

TEST(RecognitionExampleDatabaseTest, LoadsCommittedExampleDatabase) {
  RecognitionFeatureDatabase database;
  std::string error;
  ASSERT_TRUE(RecognitionFeatureDatabase::Load(ONEQ_RECOGNITION_EXAMPLE_DATABASE_PATH,
                                               &database, &error))
      << error;
  EXPECT_TRUE(database.IsLoaded());
  EXPECT_EQ(database.database_id(), "ar-target-recognition-baseline");
  EXPECT_EQ(database.version(), "1.0.0");

  // 自描述字段（display_name / aspect）与模板数据往返保真。
  ASSERT_EQ(database.categories().size(), 2U);
  EXPECT_EQ(database.categories()[0].display_name, "弹道目标");
  ASSERT_EQ(database.models().size(), 2U);
  EXPECT_EQ(database.models()[0].display_name, "弹道目标示例 A");
  const auto& profile = database.models()[0].profiles.front();
  EXPECT_FLOAT_EQ(profile.aspect_az_min_deg, -180.0f);
  EXPECT_FLOAT_EQ(profile.aspect_az_max_deg, 180.0f);
  EXPECT_FLOAT_EQ(profile.aspect_el_min_deg, -90.0f);
  EXPECT_FLOAT_EQ(profile.aspect_el_max_deg, 90.0f);
  EXPECT_FLOAT_EQ(profile.rcs.mean_dbsm, -3.0f);
  EXPECT_FLOAT_EQ(profile.rcs.std_db, 2.0f);
  EXPECT_FLOAT_EQ(profile.rcs.azimuth_variation_db, 4.0f);
  EXPECT_FLOAT_EQ(profile.motion.speed_mps.mean, 1800.0f);
  EXPECT_FLOAT_EQ(profile.motion.turn_radius_log10.mean, 6.0f);
  EXPECT_FLOAT_EQ(profile.range_profile.minimum_bandwidth_hz, 3000000.0f);
}

}  // namespace
}  // namespace tests
}  // namespace airborne_radar
