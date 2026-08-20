// Copyright 2026. All Rights Reserved.
//
// @file rir_recognition_database_test.cpp
// @brief 验证识别特征数据库 SQLite 加载/校验与动态加权匹配行为。

#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "RirSqliteTestUtil.h"
#include "remote_identification_radar/recognition/RecognitionFeatureDatabase.h"
#include "remote_identification_radar/recognition/RecognitionMatcher.h"
#include "remote_identification_radar/recognition/RecognitionTypes.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using recognition::RirFeatureDatabase;
using recognition::RirFeatureSet;
using recognition::RirMatchResult;
using recognition::RirMatcher;
using recognition::RirObservationContext;

// schema v1.1 有效库：自描述元数据（meta 六键 + units 七量纲）+ 语义分组模板表。
// 注：rcs 变化量列与 range_profile.minimum_bandwidth_hz 保持 NULL（缺省 0），
// 避免触发 Applicable() 的视角/带宽门控（与 v1.0 测试行为一致）。
constexpr const char* kValidDatabaseSql = R"sql(
INSERT INTO meta VALUES
  ('schema_version','1.1'),
  ('database_id','ar-target-recognition-baseline'),
  ('version','1.0.0'),
  ('created_utc','2026-07-22T00:00:00Z'),
  ('polarization_channels','H,V'),
  ('polarization_energy_reference','range_propagation_antenna_compensated');
INSERT INTO units VALUES
  ('rcs','dBsm'),('speed','m/s'),('altitude','m'),('acceleration','m/s2'),
  ('turn_radius','m'),('polarization','dB'),('range','m');
INSERT INTO categories VALUES
  ('BALLISTIC','弹道目标',0.5), ('NEAR_SPACE','临近空间目标',0.5);
INSERT INTO models VALUES
  ('BALLISTIC_EXAMPLE_A','BALLISTIC','弹道目标示例 A',1.0),
  ('NEAR_SPACE_EXAMPLE_A','NEAR_SPACE','临近空间目标示例 A',0.5),
  ('BALLISTIC_CLONE','BALLISTIC','弹道克隆',0.5);
INSERT INTO profiles VALUES
  ('nominal','BALLISTIC_EXAMPLE_A',6.0,50.0,NULL,NULL,NULL,NULL),
  ('nominal','NEAR_SPACE_EXAMPLE_A',6.0,NULL,-120.0,120.0,-45.0,45.0),
  ('nominal','BALLISTIC_CLONE',6.0,50.0,NULL,NULL,NULL,NULL);
INSERT INTO rcs_templates VALUES
  ('nominal','BALLISTIC_EXAMPLE_A',-3.0,2.0,NULL,NULL,NULL),
  ('nominal','NEAR_SPACE_EXAMPLE_A',2.0,2.5,NULL,NULL,NULL),
  ('nominal','BALLISTIC_CLONE',-3.0,2.0,NULL,NULL,NULL);
INSERT INTO motion_templates VALUES
  ('nominal','BALLISTIC_EXAMPLE_A',1800.0,300.0,50000.0,12000.0,12.0,6.0,6.0,0.5),
  ('nominal','NEAR_SPACE_EXAMPLE_A',300.0,80.0,25000.0,5000.0,2.0,1.0,4.5,0.5),
  ('nominal','BALLISTIC_CLONE',1800.0,300.0,50000.0,12000.0,12.0,6.0,6.0,0.5);
INSERT INTO polarization_templates VALUES
  ('nominal','BALLISTIC_EXAMPLE_A',2.0,1.5,-6.0,2.0,5.0,4.0),
  ('nominal','NEAR_SPACE_EXAMPLE_A',-1.0,1.5,-8.0,3.0,8.0,4.0),
  ('nominal','BALLISTIC_CLONE',2.0,1.5,-6.0,2.0,5.0,4.0);
INSERT INTO range_profile_templates VALUES
  ('nominal','BALLISTIC_EXAMPLE_A',8.0,2.0,3.0,1.0,0.75,0.10,NULL),
  ('nominal','NEAR_SPACE_EXAMPLE_A',20.0,5.0,4.0,1.0,0.60,0.10,NULL),
  ('nominal','BALLISTIC_CLONE',8.0,2.0,3.0,1.0,0.75,0.10,NULL);
)sql";

// 空库：仅自描述元数据（meta + units），无类别/型号/模板。
constexpr const char* kEmptyDatabaseSql = R"sql(
INSERT INTO meta VALUES
  ('schema_version','1.1'),
  ('database_id','empty'),
  ('version','1.0.0'),
  ('created_utc','2026-07-22T00:00:00Z'),
  ('polarization_channels','H,V'),
  ('polarization_energy_reference','range_propagation_antenna_compensated');
INSERT INTO units VALUES
  ('rcs','dBsm'),('speed','m/s'),('altitude','m'),('acceleration','m/s2'),
  ('turn_radius','m'),('polarization','dB'),('range','m');
)sql";

RirFeatureDatabase LoadValidDatabase() {
  const std::string path =
      WriteTempSqlite("ar_recognition_valid.db", std::string(kRecognitionSchemaSql) + kValidDatabaseSql);
  RirFeatureDatabase database;
  std::string error;
  if (!RirFeatureDatabase::Load(path, &database, &error)) {
    ADD_FAILURE() << error;
  }
  return database;
}

RirFeatureSet MakeFeaturesAtTemplate(const RirFeatureDatabase& database,
                                             std::size_t model_index) {
  // 构造与指定型号模板完全一致的观测（z=0）。
  const auto& profile = database.models()[model_index].profiles.front();
  RirFeatureSet set;
  set.rcs.valid = true;
  set.rcs.mean_dbsm = profile.rcs.mean_dbsm;
  set.rcs.quality = 1.0f;
  set.motion.valid = true;
  set.motion.speed_mps = profile.motion.speed_mps.mean;
  set.motion.altitude_m = profile.motion.altitude_m.mean;
  set.motion.acceleration_mps2 = profile.motion.acceleration_mps2.mean;
  set.motion.turn_radius_m = std::pow(10.0f, profile.motion.turn_radius_log10.mean);
  set.motion.is_straight = false;
  set.motion.quality = 1.0f;
  set.polarization.valid = true;
  set.polarization.energy_difference_db = profile.polarization.energy_difference_db.mean;
  set.polarization.relative_difference_db = profile.polarization.relative_difference_db.mean;
  set.polarization.energy_sum_db = profile.polarization.energy_sum_db.mean;
  set.polarization.quality = 1.0f;
  set.range_profile.valid = true;
  set.range_profile.length_m = profile.range_profile.length_m.mean;
  set.range_profile.peak_count = static_cast<std::uint32_t>(profile.range_profile.peak_count.mean);
  set.range_profile.peak_energy_concentration =
      profile.range_profile.peak_energy_concentration.mean;
  set.range_profile.resolution_m = 10.0f;
  set.range_profile.quality = 1.0f;
  set.valid_feature_mask = 0x0FU;
  return set;
}

TEST(RirFeatureDatabaseTest, LoadsValidSqliteDatabaseAndExposesIdentity) {
  const RirFeatureDatabase database = LoadValidDatabase();
  EXPECT_TRUE(database.IsLoaded());
  EXPECT_EQ(database.database_id(), "ar-target-recognition-baseline");
  EXPECT_EQ(database.version(), "1.0.0");
  EXPECT_EQ(database.categories().size(), 2U);
  EXPECT_EQ(database.models().size(), 3U);
  // 自描述字段随数据入库（承载不消费）：display_name 与 aspect 区间。
  EXPECT_EQ(database.categories()[0].display_name, "弹道目标");
  EXPECT_EQ(database.models()[0].display_name, "弹道目标示例 A");
}

TEST(RirFeatureDatabaseTest, LoadsSelfDescribingAspectAndTemplateData) {
  const RirFeatureDatabase database = LoadValidDatabase();
  ASSERT_EQ(database.models().size(), 3U);
  // aspect 显式值（NEAR_SPACE profile）与全范围缺省（BALLISTIC profile）。
  const auto& explicit_profile = database.models()[1].profiles.front();
  EXPECT_FLOAT_EQ(explicit_profile.aspect_az_min_deg, -120.0f);
  EXPECT_FLOAT_EQ(explicit_profile.aspect_az_max_deg, 120.0f);
  EXPECT_FLOAT_EQ(explicit_profile.aspect_el_min_deg, -45.0f);
  EXPECT_FLOAT_EQ(explicit_profile.aspect_el_max_deg, 45.0f);
  const auto& default_profile = database.models()[0].profiles.front();
  EXPECT_FLOAT_EQ(default_profile.aspect_az_min_deg, -180.0f);
  EXPECT_FLOAT_EQ(default_profile.aspect_az_max_deg, 180.0f);
  EXPECT_FLOAT_EQ(default_profile.aspect_el_min_deg, -90.0f);
  EXPECT_FLOAT_EQ(default_profile.aspect_el_max_deg, 90.0f);
  // 模板数据往返保真（含 turn_radius log10 尺度——v1.1 修复其列名加载）。
  EXPECT_FLOAT_EQ(default_profile.rcs.mean_dbsm, -3.0f);
  EXPECT_FLOAT_EQ(default_profile.rcs.std_db, 2.0f);
  EXPECT_FLOAT_EQ(default_profile.motion.speed_mps.mean, 1800.0f);
  EXPECT_FLOAT_EQ(default_profile.motion.turn_radius_log10.mean, 6.0f);
  EXPECT_FLOAT_EQ(default_profile.motion.turn_radius_log10.std, 0.5f);
  EXPECT_FLOAT_EQ(default_profile.polarization.energy_difference_db.mean, 2.0f);
  EXPECT_FLOAT_EQ(default_profile.range_profile.length_m.mean, 8.0f);
  EXPECT_FLOAT_EQ(default_profile.range_profile.peak_count.mean, 3.0f);
}

TEST(RirFeatureDatabaseTest, RejectsStructuralErrorsWithPathInMessage) {
  // 缺模板组表（模板表是 v1.1 schema 组成部分，缺失即拒绝）。
  const std::string missing_rcs = WriteTempSqlite(
      "ar_recognition_missing_rcs.db",
      std::string(kRecognitionSchemaSql) + kValidDatabaseSql +
          R"sql(DROP TABLE rcs_templates;)sql");
  RirFeatureDatabase database;
  std::string error;
  EXPECT_FALSE(RirFeatureDatabase::Load(missing_rcs, &database, &error));
  EXPECT_NE(error.find("rcs_templates"), std::string::npos);

  // rcs std == 0（CHECK 由 PRAGMA ignore_check_constraints 绕过，加载器显式校验拒绝）
  const std::string zero_std = WriteTempSqlite(
      "ar_recognition_zero_std.db",
      std::string(kRecognitionSchemaSql) + kValidDatabaseSql +
          R"sql(PRAGMA ignore_check_constraints = ON;
UPDATE rcs_templates SET std_db = 0 WHERE model_id = 'BALLISTIC_EXAMPLE_A';)sql");
  error.clear();
  EXPECT_FALSE(RirFeatureDatabase::Load(zero_std, &database, &error));
  EXPECT_NE(error.find("std_db"), std::string::npos);

  // units.rcs 声明非 dBsm → 强校验拒绝（匹配数学是 dBsm 域）
  const std::string wrong_rcs_unit = WriteTempSqlite(
      "ar_recognition_wrong_rcs_unit.db",
      std::string(kRecognitionSchemaSql) + kValidDatabaseSql +
          R"sql(UPDATE units SET unit = 'm2' WHERE quantity = 'rcs';)sql");
  error.clear();
  EXPECT_FALSE(RirFeatureDatabase::Load(wrong_rcs_unit, &database, &error));
  EXPECT_NE(error.find("dBsm"), std::string::npos);

  // units 缺必填量纲（speed）
  const std::string missing_unit = WriteTempSqlite(
      "ar_recognition_missing_unit.db",
      std::string(kRecognitionSchemaSql) + kValidDatabaseSql +
          R"sql(DELETE FROM units WHERE quantity = 'speed';)sql");
  error.clear();
  EXPECT_FALSE(RirFeatureDatabase::Load(missing_unit, &database, &error));
  EXPECT_NE(error.find("speed"), std::string::npos);

  // aspect 区间 min > max
  const std::string bad_aspect = WriteTempSqlite(
      "ar_recognition_bad_aspect.db",
      std::string(kRecognitionSchemaSql) + kValidDatabaseSql +
          R"sql(UPDATE profiles SET aspect_az_min_deg = 100.0, aspect_az_max_deg = 10.0
WHERE model_id = 'BALLISTIC_EXAMPLE_A';)sql");
  error.clear();
  EXPECT_FALSE(RirFeatureDatabase::Load(bad_aspect, &database, &error));
  EXPECT_NE(error.find("aspect_az"), std::string::npos);

  // category_id 引用不存在（FK 关闭以写入非法引用；Load 显式校验拒绝）
  const std::string bad_category = WriteTempSqlite(
      "ar_recognition_bad_category.db",
      std::string(kRecognitionSchemaSql) + kValidDatabaseSql +
          R"sql(PRAGMA foreign_keys = OFF;
INSERT INTO models VALUES ('BAD','NOPE','bad',1.0);)sql");
  error.clear();
  EXPECT_FALSE(RirFeatureDatabase::Load(bad_category, &database, &error));
  EXPECT_NE(error.find("NOPE"), std::string::npos);

  // 模板行引用未知 profile（FK 关闭以写入非法引用）
  const std::string bad_template_ref = WriteTempSqlite(
      "ar_recognition_bad_template_ref.db",
      std::string(kRecognitionSchemaSql) + kValidDatabaseSql +
          R"sql(PRAGMA foreign_keys = OFF;
INSERT INTO rcs_templates VALUES ('ghost','NOPE',1.0,2.0,NULL,NULL,NULL);)sql");
  error.clear();
  EXPECT_FALSE(RirFeatureDatabase::Load(bad_template_ref, &database, &error));
  EXPECT_NE(error.find("unknown profile"), std::string::npos);

  // 缺自描述元数据 created_utc（v1.1 自描述契约必填）
  const std::string missing_created_utc = WriteTempSqlite(
      "ar_recognition_missing_created_utc.db",
      std::string(kRecognitionSchemaSql) + kValidDatabaseSql +
          R"sql(DELETE FROM meta WHERE key = 'created_utc';)sql");
  error.clear();
  EXPECT_FALSE(RirFeatureDatabase::Load(missing_created_utc, &database, &error));
  EXPECT_NE(error.find("created_utc"), std::string::npos);
}

TEST(RirFeatureDatabaseTest, RejectsUnsupportedSchemaVersionWithoutFallback) {
  const std::string unsupported = WriteTempSqlite(
      "ar_recognition_bad_schema.db",
      std::string(kRecognitionSchemaSql) + kValidDatabaseSql +
          R"sql(UPDATE meta SET value = '9.9' WHERE key = 'schema_version';)sql");
  RirFeatureDatabase database;
  std::string error;
  EXPECT_FALSE(RirFeatureDatabase::Load(unsupported, &database, &error));
  EXPECT_NE(error.find("schema_version"), std::string::npos);
  EXPECT_FALSE(database.IsLoaded());
}

TEST(RirMatcherTest, ExactTemplateMatchYieldsSimilarityOne) {
  const RirFeatureDatabase database = LoadValidDatabase();
  const RirFeatureSet features = MakeFeaturesAtTemplate(database, 0U);
  RirObservationContext context;
  context.snr_db = 20.0f;
  context.range_m = 100000.0f;

  const RirMatchResult result =
      RirMatcher::QueryBestMatch(features, context, database);

  ASSERT_TRUE(result.has_candidates);
  ASSERT_EQ(result.candidates.front().model_id, "BALLISTIC_EXAMPLE_A");
  EXPECT_NEAR(result.candidates.front().score, 1.0f, 0.01f);  // z=0 → s=1，prior=1
}

TEST(RirMatcherTest, TwoSigmaOffsetYieldsExpectedSimilarity) {
  const RirFeatureDatabase database = LoadValidDatabase();
  RirFeatureSet features = MakeFeaturesAtTemplate(database, 0U);
  // 仅 RCS 维度有效，且均值偏离模板 2σ → 得分 ≈ exp(-2)（RCS 为单特征维度）。
  features.valid_feature_mask = 0x01U;
  features.motion.valid = false;
  features.motion.quality = 0.0f;
  features.polarization.valid = false;
  features.polarization.quality = 0.0f;
  features.range_profile.valid = false;
  features.range_profile.quality = 0.0f;
  const auto& profile = database.models()[0].profiles.front();
  features.rcs.mean_dbsm = profile.rcs.mean_dbsm + 2.0f * profile.rcs.std_db;
  RirObservationContext context;
  context.snr_db = 20.0f;
  context.range_m = 100000.0f;

  const RirMatchResult result =
      RirMatcher::QueryBestMatch(features, context, database);

  ASSERT_TRUE(result.has_candidates);
  // BALLISTIC_EXAMPLE_A 的 RCS 相似度即 exp(-2)（z=2，先验 1.0）。
  bool found = false;
  for (const auto& candidate : result.candidates) {
    if (candidate.model_id == "BALLISTIC_EXAMPLE_A") {
      EXPECT_NEAR(candidate.score, std::exp(-2.0), 0.01);
      found = true;
    }
  }
  EXPECT_TRUE(found);
}

TEST(RirMatcherTest, InapplicableProfileYieldsZeroScore) {
  const RirFeatureDatabase database = LoadValidDatabase();
  const RirFeatureSet features = MakeFeaturesAtTemplate(database, 0U);
  RirObservationContext context;
  context.snr_db = 3.0f;  // 低于所有 profile 的 min_snr_db=6
  context.range_m = 100000.0f;

  const RirMatchResult result =
      RirMatcher::QueryBestMatch(features, context, database);

  EXPECT_FALSE(result.has_candidates);
}

TEST(RirMatcherTest, PriorOrdersEqualSimilarityCandidates) {
  const RirFeatureDatabase database = LoadValidDatabase();
  // 观测与 BALLISTIC_EXAMPLE_A / BALLISTIC_CLONE 两同模板型号都完全匹配
  // （prior 分别为 1.0 与 0.5）→ prior 大的排第一，得分比 = prior 比。
  const RirFeatureSet features = MakeFeaturesAtTemplate(database, 0U);
  RirObservationContext context;
  context.snr_db = 20.0f;
  context.range_m = 100000.0f;

  const RirMatchResult result =
      RirMatcher::QueryBestMatch(features, context, database);

  ASSERT_TRUE(result.has_candidates);
  ASSERT_GE(result.candidates.size(), 2U);
  EXPECT_EQ(result.candidates[0].model_id, "BALLISTIC_EXAMPLE_A");
  EXPECT_EQ(result.candidates[1].model_id, "BALLISTIC_CLONE");
  EXPECT_GT(result.candidates[0].score, result.candidates[1].score);
  EXPECT_NEAR(result.candidates[0].score / result.candidates[1].score, 2.0f, 0.1f);
}

TEST(RirMatcherTest, CategoryScoreSumsModelScores) {
  const RirFeatureDatabase database = LoadValidDatabase();
  const RirFeatureSet features = MakeFeaturesAtTemplate(database, 0U);
  RirObservationContext context;
  context.snr_db = 20.0f;
  context.range_m = 100000.0f;

  const RirMatchResult result =
      RirMatcher::QueryBestMatch(features, context, database);

  ASSERT_TRUE(result.has_candidates);
  ASSERT_EQ(result.category_scores.size(), 2U);
  float ballistic_sum = 0.0f;
  for (const auto& candidate : result.candidates) {
    if (candidate.category_id == "BALLISTIC") {
      ballistic_sum += candidate.score;
    }
  }
  EXPECT_NEAR(result.category_scores[0].second, ballistic_sum, 0.001f);
  EXPECT_EQ(result.category_scores[0].first, "BALLISTIC");
}

TEST(RirMatcherTest, ZeroQualityDimensionIsExcludedFromDenominator) {
  const RirFeatureDatabase database = LoadValidDatabase();
  RirFeatureSet features = MakeFeaturesAtTemplate(database, 0U);
  // RCS 维度质量 0（但 valid 仍为 true）→ 其权重不参与分母，其余维度按比例放大。
  features.rcs.quality = 0.0f;
  RirObservationContext context;
  context.snr_db = 20.0f;
  context.range_m = 100000.0f;

  const RirMatchResult result =
      RirMatcher::QueryBestMatch(features, context, database);

  ASSERT_TRUE(result.has_candidates);
  // 其余三维 z=0 → 相似度 1，加权平均仍为 1。
  EXPECT_NEAR(result.candidates.front().score, 1.0f, 0.01f);
}

TEST(RirMatcherTest, EmptyDatabaseReturnsEmptyResultWithoutCrash) {
  const std::string empty = WriteTempSqlite(
      "ar_recognition_empty.db",
      std::string(kRecognitionSchemaSql) + kEmptyDatabaseSql);
  RirFeatureDatabase database;
  std::string error;
  ASSERT_TRUE(RirFeatureDatabase::Load(empty, &database, &error)) << error;
  RirFeatureSet features;
  RirObservationContext context;
  context.snr_db = 20.0f;

  const RirMatchResult result =
      RirMatcher::QueryBestMatch(features, context, database);

  EXPECT_FALSE(result.has_candidates);
  EXPECT_TRUE(result.candidates.empty());
  EXPECT_TRUE(result.best_model_id.empty());
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
