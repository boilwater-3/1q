// Copyright 2026. All Rights Reserved.
//
// @file RecognitionSqliteTestUtil.h
// @brief 识别特征数据库测试的 SQLite 构造工具（测试专用，非库代码）。
//
// 用 sqlite3 C API 创建临时库文件并执行 SQL 脚本（可含多语句 DDL+INSERT）。
// 非法数据用例（如外键引用不存在）由 SQL 脚本自行 PRAGMA foreign_keys=OFF
// 构造——Load 的显式校验才是这类失败的信息来源，SQLite 约束只作兜底。

#ifndef TESTS_UNIT_AIRBORNE_RADAR_RECOGNITION_SQLITE_TEST_UTIL_H_
#define TESTS_UNIT_AIRBORNE_RADAR_RECOGNITION_SQLITE_TEST_UTIL_H_

#include <gtest/gtest.h>

#include <sqlite3.h>

#include <cstdio>
#include <string>

namespace airborne_radar {
namespace tests {

/**
 * @brief 识别特征库测试 schema（与 RecognitionFeatureDatabase 加载器对应）。
 * meta/categories/models/profiles 四表；profile 模板列拍平，可空列对应
 * JSON 版可选字段；复合主键 (model_id, profile_id) 保持"模型内唯一"语义。
 */
inline constexpr const char* kRecognitionSchemaSql = R"sql(
CREATE TABLE meta(key TEXT PRIMARY KEY, value TEXT NOT NULL);
CREATE TABLE categories(category_id TEXT PRIMARY KEY, prior REAL);
CREATE TABLE models(model_id TEXT PRIMARY KEY,
                    category_id TEXT NOT NULL REFERENCES categories(category_id),
                    prior REAL);
CREATE TABLE profiles(
  profile_id TEXT NOT NULL,
  model_id TEXT NOT NULL REFERENCES models(model_id),
  min_snr_db REAL, max_range_resolution_m REAL,
  rcs_mean_dbsm REAL, rcs_std_db REAL,
  rcs_azimuth_variation_db REAL, rcs_elevation_variation_db REAL,
  rcs_minimum_aspect_coverage_deg REAL,
  motion_speed_mean REAL, motion_speed_std REAL,
  motion_altitude_mean REAL, motion_altitude_std REAL,
  motion_acceleration_mean REAL, motion_acceleration_std REAL,
  motion_turn_radius_mean_log10 REAL, motion_turn_radius_std_log10 REAL,
  polarization_energy_difference_mean REAL, polarization_energy_difference_std REAL,
  polarization_relative_difference_mean REAL, polarization_relative_difference_std REAL,
  polarization_energy_sum_mean REAL, polarization_energy_sum_std REAL,
  range_profile_length_mean REAL, range_profile_length_std REAL,
  range_profile_peak_count_mean REAL, range_profile_peak_count_std REAL,
  range_profile_peak_energy_concentration_mean REAL,
  range_profile_peak_energy_concentration_std REAL,
  range_profile_minimum_bandwidth_hz REAL,
  PRIMARY KEY (model_id, profile_id));
)sql";

/** @brief 覆写临时 SQLite 文件并执行 SQL 脚本；失败返回空串并填 error。 */
inline std::string WriteTempSqlite(const std::string& file_name, const std::string& sql,
                                   std::string* error = nullptr) {
  const std::string path = ::testing::TempDir() + "/" + file_name;
  std::remove(path.c_str());  // 幂等：覆写旧文件，避免残留表导致 exec 失败。
  sqlite3* db = nullptr;
  if (sqlite3_open_v2(path.c_str(), &db, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, nullptr) !=
      SQLITE_OK) {
    if (error != nullptr) {
      *error = db != nullptr ? sqlite3_errmsg(db) : "sqlite3_open_v2 failed";
    }
    if (db != nullptr) {
      sqlite3_close(db);
    }
    return {};
  }
  char* sqlite_error = nullptr;
  const int rc = sqlite3_exec(db, sql.c_str(), nullptr, nullptr, &sqlite_error);
  std::string failure;
  if (rc != SQLITE_OK) {
    failure = sqlite_error != nullptr ? sqlite_error : "sqlite3_exec failed";
    sqlite3_free(sqlite_error);
  }
  sqlite3_close(db);
  if (error != nullptr) {
    *error = failure;
  }
  return failure.empty() ? path : std::string{};
}

}  // namespace tests
}  // namespace airborne_radar

#endif  // TESTS_UNIT_AIRBORNE_RADAR_RECOGNITION_SQLITE_TEST_UTIL_H_
