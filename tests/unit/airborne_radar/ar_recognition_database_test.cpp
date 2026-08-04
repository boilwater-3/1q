// Copyright 2026. All Rights Reserved.
//
// @file ar_recognition_database_test.cpp
// @brief 验证识别特征数据库 JSON 加载/校验与动态加权匹配行为。

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <string>

#include "airborne_radar/recognition/RecognitionFeatureDatabase.h"
#include "airborne_radar/recognition/RecognitionMatcher.h"
#include "airborne_radar/recognition/RecognitionTypes.h"

namespace airborne_radar {
namespace tests {
namespace {

using recognition::RecognitionFeatureDatabase;
using recognition::RecognitionFeatureSet;
using recognition::RecognitionMatchResult;
using recognition::RecognitionMatcher;
using recognition::RecognitionObservationContext;

std::string WriteTempJson(const std::string& name, const std::string& content) {
  const std::string path = ::testing::TempDir() + "/" + name;
  std::ofstream stream(path, std::ios::trunc);
  stream << content;
  stream.close();
  return path;
}

const char* kValidDatabaseJson = R"json(
{
  "schema_version": "1.0",
  "database_id": "ar-target-recognition-baseline",
  "version": "1.0.0",
  "created_utc": "2026-07-22T00:00:00Z",
  "polarization_channels": ["H", "V"],
  "units": {"rcs": "dBsm"},
  "categories": [
    {"category_id": "BALLISTIC", "display_name": "弹道目标", "prior": 0.5},
    {"category_id": "NEAR_SPACE", "display_name": "临近空间目标", "prior": 0.5}
  ],
  "models": [
    {
      "model_id": "BALLISTIC_EXAMPLE_A",
      "category_id": "BALLISTIC",
      "prior": 1.0,
      "profiles": [
        {
          "profile_id": "nominal",
          "applicability": {"min_snr_db": 6.0, "max_range_resolution_m": 50.0},
          "rcs": {"mean_dbsm": -3.0, "std_db": 2.0},
          "motion": {
            "speed_mps": {"mean": 1800.0, "std": 300.0},
            "altitude_m": {"mean": 50000.0, "std": 12000.0},
            "acceleration_mps2": {"mean": 12.0, "std": 6.0},
            "turn_radius_m": {"mean_log10": 6.0, "std_log10": 0.5}
          },
          "polarization": {
            "energy_difference_db": {"mean": 2.0, "std": 1.5},
            "relative_difference_db": {"mean": -6.0, "std": 2.0},
            "energy_sum_db": {"mean": 5.0, "std": 4.0}
          },
          "range_profile": {
            "length_m": {"mean": 8.0, "std": 2.0},
            "peak_count": {"mean": 3.0, "std": 1.0},
            "peak_energy_concentration": {"mean": 0.75, "std": 0.10}
          }
        }
      ]
    },
    {
      "model_id": "NEAR_SPACE_EXAMPLE_A",
      "category_id": "NEAR_SPACE",
      "prior": 0.5,
      "profiles": [
        {
          "profile_id": "nominal",
          "applicability": {"min_snr_db": 6.0},
          "rcs": {"mean_dbsm": 2.0, "std_db": 2.5},
          "motion": {
            "speed_mps": {"mean": 300.0, "std": 80.0},
            "altitude_m": {"mean": 25000.0, "std": 5000.0},
            "acceleration_mps2": {"mean": 2.0, "std": 1.0},
            "turn_radius_m": {"mean_log10": 4.5, "std_log10": 0.5}
          },
          "polarization": {
            "energy_difference_db": {"mean": -1.0, "std": 1.5},
            "relative_difference_db": {"mean": -8.0, "std": 3.0},
            "energy_sum_db": {"mean": 8.0, "std": 4.0}
          },
          "range_profile": {
            "length_m": {"mean": 20.0, "std": 5.0},
            "peak_count": {"mean": 4.0, "std": 1.0},
            "peak_energy_concentration": {"mean": 0.60, "std": 0.10}
          }
        }
      ]
    },
    {
      "model_id": "BALLISTIC_CLONE",
      "category_id": "BALLISTIC",
      "prior": 0.5,
      "profiles": [
        {
          "profile_id": "nominal",
          "applicability": {"min_snr_db": 6.0, "max_range_resolution_m": 50.0},
          "rcs": {"mean_dbsm": -3.0, "std_db": 2.0},
          "motion": {
            "speed_mps": {"mean": 1800.0, "std": 300.0},
            "altitude_m": {"mean": 50000.0, "std": 12000.0},
            "acceleration_mps2": {"mean": 12.0, "std": 6.0},
            "turn_radius_m": {"mean_log10": 6.0, "std_log10": 0.5}
          },
          "polarization": {
            "energy_difference_db": {"mean": 2.0, "std": 1.5},
            "relative_difference_db": {"mean": -6.0, "std": 2.0},
            "energy_sum_db": {"mean": 5.0, "std": 4.0}
          },
          "range_profile": {
            "length_m": {"mean": 8.0, "std": 2.0},
            "peak_count": {"mean": 3.0, "std": 1.0},
            "peak_energy_concentration": {"mean": 0.75, "std": 0.10}
          }
        }
      ]
    }
  ]
}
)json";

RecognitionFeatureDatabase LoadValidDatabase() {
  const std::string path = WriteTempJson("ar_recognition_valid.json", kValidDatabaseJson);
  RecognitionFeatureDatabase database;
  std::string error;
  if (!RecognitionFeatureDatabase::Load(path, &database, &error)) {
    ADD_FAILURE() << error;
  }
  return database;
}

RecognitionFeatureSet MakeFeaturesAtTemplate(const RecognitionFeatureDatabase& database,
                                             std::size_t model_index) {
  // 构造与指定型号模板完全一致的观测（z=0）。
  const auto& profile = database.models()[model_index].profiles.front();
  RecognitionFeatureSet set;
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

TEST(RecognitionFeatureDatabaseTest, LoadsValidJsonAndExposesIdentity) {
  const RecognitionFeatureDatabase database = LoadValidDatabase();
  EXPECT_TRUE(database.IsLoaded());
  EXPECT_EQ(database.database_id(), "ar-target-recognition-baseline");
  EXPECT_EQ(database.version(), "1.0.0");
  EXPECT_EQ(database.categories().size(), 2U);
  EXPECT_EQ(database.models().size(), 3U);
}

TEST(RecognitionFeatureDatabaseTest, RejectsStructuralErrorsWithPathInMessage) {
  // 缺 models
  const std::string missing_models =
      WriteTempJson("ar_recognition_missing_models.json",
                    R"json({"schema_version":"1.0","database_id":"d","version":"1.0.0"})json");
  RecognitionFeatureDatabase database;
  std::string error;
  EXPECT_FALSE(RecognitionFeatureDatabase::Load(missing_models, &database, &error));
  EXPECT_NE(error.find("models"), std::string::npos);

  // std == 0
  const std::string zero_std = WriteTempJson(
      "ar_recognition_zero_std.json",
      R"json({"schema_version":"1.0","database_id":"d","version":"1.0.0","categories":[],
      "models":[{"model_id":"m","category_id":"c","profiles":[
        {"profile_id":"p","rcs":{"mean_dbsm":0.0,"std_db":0.0}}]}]})json");
  error.clear();
  EXPECT_FALSE(RecognitionFeatureDatabase::Load(zero_std, &database, &error));
  EXPECT_NE(error.find("std_db"), std::string::npos);

  // category_id 引用不存在
  const std::string bad_category = WriteTempJson(
      "ar_recognition_bad_category.json",
      R"json({"schema_version":"1.0","database_id":"d","version":"1.0.0","categories":[],
      "models":[{"model_id":"m","category_id":"NOPE","profiles":[
        {"profile_id":"p"}]}]})json");
  error.clear();
  EXPECT_FALSE(RecognitionFeatureDatabase::Load(bad_category, &database, &error));
  EXPECT_NE(error.find("NOPE"), std::string::npos);
}

TEST(RecognitionFeatureDatabaseTest, RejectsUnsupportedSchemaVersionWithoutFallback) {
  const std::string unsupported = WriteTempJson(
      "ar_recognition_bad_schema.json",
      R"json({"schema_version":"9.9","database_id":"d","version":"1.0.0","categories":[],
      "models":[]})json");
  RecognitionFeatureDatabase database;
  std::string error;
  EXPECT_FALSE(RecognitionFeatureDatabase::Load(unsupported, &database, &error));
  EXPECT_NE(error.find("schema_version"), std::string::npos);
  EXPECT_FALSE(database.IsLoaded());
}

TEST(RecognitionMatcherTest, ExactTemplateMatchYieldsSimilarityOne) {
  const RecognitionFeatureDatabase database = LoadValidDatabase();
  const RecognitionFeatureSet features = MakeFeaturesAtTemplate(database, 0U);
  RecognitionObservationContext context;
  context.snr_db = 20.0f;
  context.range_m = 100000.0f;

  const RecognitionMatchResult result =
      RecognitionMatcher::QueryBestMatch(features, context, database);

  ASSERT_TRUE(result.has_candidates);
  ASSERT_EQ(result.candidates.front().model_id, "BALLISTIC_EXAMPLE_A");
  EXPECT_NEAR(result.candidates.front().score, 1.0f, 0.01f);  // z=0 → s=1，prior=1
}

TEST(RecognitionMatcherTest, TwoSigmaOffsetYieldsExpectedSimilarity) {
  const RecognitionFeatureDatabase database = LoadValidDatabase();
  RecognitionFeatureSet features = MakeFeaturesAtTemplate(database, 0U);
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
  RecognitionObservationContext context;
  context.snr_db = 20.0f;
  context.range_m = 100000.0f;

  const RecognitionMatchResult result =
      RecognitionMatcher::QueryBestMatch(features, context, database);

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

TEST(RecognitionMatcherTest, InapplicableProfileYieldsZeroScore) {
  const RecognitionFeatureDatabase database = LoadValidDatabase();
  const RecognitionFeatureSet features = MakeFeaturesAtTemplate(database, 0U);
  RecognitionObservationContext context;
  context.snr_db = 3.0f;  // 低于所有 profile 的 min_snr_db=6
  context.range_m = 100000.0f;

  const RecognitionMatchResult result =
      RecognitionMatcher::QueryBestMatch(features, context, database);

  EXPECT_FALSE(result.has_candidates);
}

TEST(RecognitionMatcherTest, PriorOrdersEqualSimilarityCandidates) {
  const RecognitionFeatureDatabase database = LoadValidDatabase();
  // 观测与 BALLISTIC_EXAMPLE_A / BALLISTIC_CLONE 两同模板型号都完全匹配
  // （prior 分别为 1.0 与 0.5）→ prior 大的排第一，得分比 = prior 比。
  const RecognitionFeatureSet features = MakeFeaturesAtTemplate(database, 0U);
  RecognitionObservationContext context;
  context.snr_db = 20.0f;
  context.range_m = 100000.0f;

  const RecognitionMatchResult result =
      RecognitionMatcher::QueryBestMatch(features, context, database);

  ASSERT_TRUE(result.has_candidates);
  ASSERT_GE(result.candidates.size(), 2U);
  EXPECT_EQ(result.candidates[0].model_id, "BALLISTIC_EXAMPLE_A");
  EXPECT_EQ(result.candidates[1].model_id, "BALLISTIC_CLONE");
  EXPECT_GT(result.candidates[0].score, result.candidates[1].score);
  EXPECT_NEAR(result.candidates[0].score / result.candidates[1].score, 2.0f, 0.1f);
}

TEST(RecognitionMatcherTest, CategoryScoreSumsModelScores) {
  const RecognitionFeatureDatabase database = LoadValidDatabase();
  const RecognitionFeatureSet features = MakeFeaturesAtTemplate(database, 0U);
  RecognitionObservationContext context;
  context.snr_db = 20.0f;
  context.range_m = 100000.0f;

  const RecognitionMatchResult result =
      RecognitionMatcher::QueryBestMatch(features, context, database);

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

TEST(RecognitionMatcherTest, ZeroQualityDimensionIsExcludedFromDenominator) {
  const RecognitionFeatureDatabase database = LoadValidDatabase();
  RecognitionFeatureSet features = MakeFeaturesAtTemplate(database, 0U);
  // RCS 维度质量 0（但 valid 仍为 true）→ 其权重不参与分母，其余维度按比例放大。
  features.rcs.quality = 0.0f;
  RecognitionObservationContext context;
  context.snr_db = 20.0f;
  context.range_m = 100000.0f;

  const RecognitionMatchResult result =
      RecognitionMatcher::QueryBestMatch(features, context, database);

  ASSERT_TRUE(result.has_candidates);
  // 其余三维 z=0 → 相似度 1，加权平均仍为 1。
  EXPECT_NEAR(result.candidates.front().score, 1.0f, 0.01f);
}

TEST(RecognitionMatcherTest, EmptyDatabaseReturnsEmptyResultWithoutCrash) {
  const std::string empty = WriteTempJson(
      "ar_recognition_empty.json",
      R"json({"schema_version":"1.0","database_id":"d","version":"1.0.0","categories":[],
      "models":[]})json");
  RecognitionFeatureDatabase database;
  std::string error;
  ASSERT_TRUE(RecognitionFeatureDatabase::Load(empty, &database, &error)) << error;
  RecognitionFeatureSet features;
  RecognitionObservationContext context;
  context.snr_db = 20.0f;

  const RecognitionMatchResult result =
      RecognitionMatcher::QueryBestMatch(features, context, database);

  EXPECT_FALSE(result.has_candidates);
  EXPECT_TRUE(result.candidates.empty());
  EXPECT_TRUE(result.best_model_id.empty());
}

}  // namespace
}  // namespace tests
}  // namespace airborne_radar
