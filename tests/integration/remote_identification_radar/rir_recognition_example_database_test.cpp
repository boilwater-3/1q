// Copyright 2026. All Rights Reserved.
//
// @file rir_recognition_example_database_test.cpp
// @brief 验证提交入库的示例识别基线可被加载（建库工具生成物端到端）。
//
// 示例库 examples/configs/recognition/target_feature_database_v1.1.db 由
// tools/recognition_db_builder.py 从 recognition_database_input.json 生成；
// 本用例证明 工具 → 权威 DDL → 加载器 全链路一致。
// 库内容：6 类别（弹道/临近空间/战斗机/轰炸机/导弹/无人机）、17 型号
// （含 15 个常见美方型号，公开估算参数，非敏感占位数据）。

#include <gtest/gtest.h>

#include <string>

#include "remote_identification_radar/recognition/RecognitionFeatureDatabase.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using recognition::RirFeatureDatabase;
using recognition::RirModel;

#ifndef ONEQ_RIR_EXAMPLE_DATABASE_PATH
#error "ONEQ_RIR_EXAMPLE_DATABASE_PATH 未定义（Integration.cmake 注入）"
#endif

/** @brief 按 model_id 查找型号；找不到返回 nullptr。 */
const RirModel* FindModel(const RirFeatureDatabase& database,
                                  const std::string& model_id) {
  for (const auto& model : database.models()) {
    if (model.model_id == model_id) {
      return &model;
    }
  }
  return nullptr;
}

TEST(RecognitionExampleDatabaseTest, LoadsCommittedExampleDatabase) {
  RirFeatureDatabase database;
  std::string error;
  ASSERT_TRUE(RirFeatureDatabase::Load(ONEQ_RIR_EXAMPLE_DATABASE_PATH,
                                               &database, &error))
      << error;
  EXPECT_TRUE(database.IsLoaded());
  EXPECT_EQ(database.database_id(), "ar-target-recognition-baseline");
  EXPECT_EQ(database.version(), "1.1.0");

  // 自描述字段（display_name / aspect）与模板数据往返保真。
  ASSERT_EQ(database.categories().size(), 6U);
  EXPECT_EQ(database.categories()[0].display_name, "弹道目标");
  ASSERT_EQ(database.models().size(), 17U);
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

  // 新类别与常见美方型号抽查（战斗机/轰炸机/导弹/无人机各一）。
  const RirModel* f16 = FindModel(database, "F-16C");
  ASSERT_NE(f16, nullptr);
  EXPECT_EQ(f16->category_id, "FIGHTER");
  EXPECT_EQ(f16->display_name, "F-16C 战隼");
  EXPECT_FLOAT_EQ(f16->profiles.front().rcs.mean_dbsm, 0.8f);
  EXPECT_FLOAT_EQ(f16->profiles.front().motion.speed_mps.mean, 250.0f);
  EXPECT_FLOAT_EQ(f16->profiles.front().motion.altitude_m.mean, 10500.0f);

  const RirModel* b52 = FindModel(database, "B-52H");
  ASSERT_NE(b52, nullptr);
  EXPECT_EQ(b52->category_id, "BOMBER");
  EXPECT_FLOAT_EQ(b52->profiles.front().motion.speed_mps.mean, 240.0f);
  EXPECT_FLOAT_EQ(b52->profiles.front().rcs.mean_dbsm, 20.0f);

  const RirModel* bgm109 = FindModel(database, "BGM-109");
  ASSERT_NE(bgm109, nullptr);
  EXPECT_EQ(bgm109->category_id, "MISSILE");
  EXPECT_FLOAT_EQ(bgm109->profiles.front().motion.altitude_m.mean, 40.0f);

  const RirModel* mq9 = FindModel(database, "MQ-9A");
  ASSERT_NE(mq9, nullptr);
  EXPECT_EQ(mq9->category_id, "UAV");
  EXPECT_FLOAT_EQ(mq9->profiles.front().motion.speed_mps.mean, 78.0f);

  // 新条目 gate 字段保持 NULL 语义（不触发 gating；min_snr_db 除外）。
  EXPECT_FLOAT_EQ(f16->profiles.front().max_range_resolution_m, 0.0f);
  EXPECT_FLOAT_EQ(f16->profiles.front().rcs.minimum_aspect_coverage_deg, 0.0f);
  EXPECT_FLOAT_EQ(f16->profiles.front().range_profile.minimum_bandwidth_hz, 0.0f);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
