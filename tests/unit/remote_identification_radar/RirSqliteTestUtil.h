// Copyright 2026. All Rights Reserved.
//
// @file RecognitionSqliteTestUtil.h
// @brief 识别特征数据库测试的 SQLite 构造工具（测试专用，非库代码）。
//
// 用 sqlite3 C API 创建临时库文件并执行 SQL 脚本（可含多语句 DDL+INSERT）。
// schema DDL 来自权威单源 schemas/recognition/recognition_feature_database.sql
// （CMake configure_file 生成头 recognition_feature_database_schema.h），
// 不在测试内维护第二份 DDL。
// 非法数据用例（如外键引用不存在）由 SQL 脚本自行 PRAGMA foreign_keys=OFF
// 构造——Load 的显式校验才是这类失败的信息来源，SQLite 约束只作兜底。

#ifndef TESTS_UNIT_REMOTE_IDENTIFICATION_RADAR_RECOGNITION_SQLITE_TEST_UTIL_H_
#define TESTS_UNIT_REMOTE_IDENTIFICATION_RADAR_RECOGNITION_SQLITE_TEST_UTIL_H_

#include <gtest/gtest.h>

#include <sqlite3.h>

#include <cstdio>
#include <string>

#include "recognition_feature_database_schema.h"

namespace remote_identification_radar {
namespace tests {

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
}  // namespace remote_identification_radar

#endif  // TESTS_UNIT_REMOTE_IDENTIFICATION_RADAR_RECOGNITION_SQLITE_TEST_UTIL_H_
