/**
 * @file RecognitionFeatureDatabase.cpp
 * @brief 目标特征数据库加载与校验实现（SQLite 存储）。
 *
 * SQLite 是加载期读取器：加载时只读打开 → 读 meta/表 → 校验 → 关闭连接，
 * 成功后数据库全量驻留内存，运行期不持有连接（Matcher/Tracker 只读消费
 * 内存结构）。失败语义与错误信息风格与 JSON 版一致（含路径与字段上下文）。
 */

#include "airborne_radar/recognition/RecognitionFeatureDatabase.h"

#include <cmath>
#include <cstring>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>

#include <sqlite3.h>

namespace airborne_radar {
namespace recognition {

namespace {

/** @brief 受支持的数据库 schema 版本。 */
constexpr const char* kSupportedSchemaVersion = "1.0";

/** @brief sqlite3 连接 RAII 关闭器（无异常；错误码转状态字符串）。 */
struct SqliteCloser {
  void operator()(sqlite3* db) const { sqlite3_close(db); }
};
using SqlitePtr = std::unique_ptr<sqlite3, SqliteCloser>;

/** @brief 以 sqlite3 错误消息填充 error（含路径与步骤），返回 false。 */
bool FailWithSqliteError(const std::string& path, sqlite3* db, const char* step,
                         std::string* error) {
  if (error != nullptr) {
    *error = path + ": " + step + ": " + sqlite3_errmsg(db);
  }
  return false;
}

/** @brief 查找语句列索引；未找到返回 -1。 */
int FindColumn(sqlite3_stmt* stmt, const char* name) {
  const int count = sqlite3_column_count(stmt);
  for (int i = 0; i < count; ++i) {
    if (std::strcmp(sqlite3_column_name(stmt, i), name) == 0) {
      return i;
    }
  }
  return -1;
}

/** @brief 取列浮点值；NULL 列回退默认值。 */
float ColumnFloat(sqlite3_stmt* stmt, int column, float fallback) {
  if (column < 0 || sqlite3_column_type(stmt, column) == SQLITE_NULL) {
    return fallback;
  }
  return static_cast<float>(sqlite3_column_double(stmt, column));
}

/**
 * @brief 单连续特征模板（mean/std 列对）读取。
 * 组内任一列非 NULL 即视为模板存在：std 必须非 NULL 且 > 0（与 JSON 版
 * "std 必填"语义一致），mean 可空（缺省 0.0f）。
 * @param std_field_name 错误信息中的 std 字段名（RCS 为 "std_db"，其余 "std"）。
 * @return 模板缺席返回 true 且 present=false；失败返回 false 并填 error。
 */
bool ReadTemplate(sqlite3_stmt* stmt, int mean_column, int std_column, const std::string& path,
                  bool* present, RecognitionFeatureTemplate* out, std::string* error,
                  const char* std_field_name = "std") {
  const bool mean_present =
      mean_column >= 0 && sqlite3_column_type(stmt, mean_column) != SQLITE_NULL;
  const bool std_present =
      std_column >= 0 && sqlite3_column_type(stmt, std_column) != SQLITE_NULL;
  if (!mean_present && !std_present) {
    *present = false;
    return true;
  }
  *present = true;
  if (!std_present) {
    if (error != nullptr) {
      *error = path + "." + std_field_name + " must be > 0";
    }
    return false;
  }
  const double std_value = sqlite3_column_double(stmt, std_column);
  if (!(std_value > 0.0)) {
    if (error != nullptr) {
      *error = path + "." + std_field_name + " must be > 0";
    }
    return false;
  }
  out->std = static_cast<float>(std_value);
  out->mean = ColumnFloat(stmt, mean_column, 0.0f);
  return true;
}

/** @brief 按列名前缀读取模板；列缺失（-1）视为缺席。 */
bool ReadNamedTemplate(sqlite3_stmt* stmt, const char* prefix, const std::string& path,
                       bool* present, RecognitionFeatureTemplate* out, std::string* error) {
  const std::string mean_column = std::string(prefix) + "_mean";
  const std::string std_column = std::string(prefix) + "_std";
  return ReadTemplate(stmt, FindColumn(stmt, mean_column.c_str()),
                      FindColumn(stmt, std_column.c_str()), path, present, out, error);
}

/** @brief 读取 rcs 模板组（列名 rcs_mean_dbsm/rcs_std_db，含变化量列）。 */
bool ReadRcsTemplate(sqlite3_stmt* stmt, const std::string& path,
                     RecognitionRcsTemplate* out, std::string* error) {
  bool present = false;
  RecognitionFeatureTemplate rcs_mean_std;
  if (!ReadTemplate(stmt, FindColumn(stmt, "rcs_mean_dbsm"), FindColumn(stmt, "rcs_std_db"),
                    path + ".rcs", &present, &rcs_mean_std, error, "std_db")) {
    return false;
  }
  if (!present) {
    return true;
  }
  out->mean_dbsm = rcs_mean_std.mean;
  out->std_db = rcs_mean_std.std;
  out->azimuth_variation_db = ColumnFloat(stmt, FindColumn(stmt, "rcs_azimuth_variation_db"), 0.0f);
  out->elevation_variation_db =
      ColumnFloat(stmt, FindColumn(stmt, "rcs_elevation_variation_db"), 0.0f);
  out->minimum_aspect_coverage_deg =
      ColumnFloat(stmt, FindColumn(stmt, "rcs_minimum_aspect_coverage_deg"), 0.0f);
  return true;
}

/** @brief 读取 motion 模板组。 */
bool ReadMotionTemplate(sqlite3_stmt* stmt, const std::string& path,
                        RecognitionMotionTemplate* out, std::string* error) {
  bool present = false;
  if (!ReadNamedTemplate(stmt, "motion_speed", path + ".motion.speed_mps", &present,
                         &out->speed_mps, error)) {
    return false;
  }
  if (!ReadNamedTemplate(stmt, "motion_altitude", path + ".motion.altitude_m", &present,
                         &out->altitude_m, error)) {
    return false;
  }
  if (!ReadNamedTemplate(stmt, "motion_acceleration", path + ".motion.acceleration_mps2",
                         &present, &out->acceleration_mps2, error)) {
    return false;
  }
  // 转弯半径按对数尺度比对（§7.4：mean_log10/std_log10）。
  if (!ReadNamedTemplate(stmt, "motion_turn_radius", path + ".motion.turn_radius_m", &present,
                         &out->turn_radius_log10, error)) {
    return false;
  }
  return true;
}

/** @brief 读取 polarization 模板组。 */
bool ReadPolarizationTemplate(sqlite3_stmt* stmt, const std::string& path,
                              RecognitionPolarizationTemplate* out, std::string* error) {
  bool present = false;
  if (!ReadNamedTemplate(stmt, "polarization_energy_difference",
                         path + ".polarization.energy_difference_db", &present,
                         &out->energy_difference_db, error)) {
    return false;
  }
  if (!ReadNamedTemplate(stmt, "polarization_relative_difference",
                         path + ".polarization.relative_difference_db", &present,
                         &out->relative_difference_db, error)) {
    return false;
  }
  if (!ReadNamedTemplate(stmt, "polarization_energy_sum", path + ".polarization.energy_sum_db",
                         &present, &out->energy_sum_db, error)) {
    return false;
  }
  return true;
}

/** @brief 读取 range_profile 模板组。 */
bool ReadRangeProfileTemplate(sqlite3_stmt* stmt, const std::string& path,
                              RecognitionRangeProfileTemplate* out, std::string* error) {
  bool present = false;
  if (!ReadNamedTemplate(stmt, "range_profile_length", path + ".range_profile.length_m",
                         &present, &out->length_m, error)) {
    return false;
  }
  if (!ReadNamedTemplate(stmt, "range_profile_peak_count", path + ".range_profile.peak_count",
                         &present, &out->peak_count, error)) {
    return false;
  }
  if (!ReadNamedTemplate(stmt, "range_profile_peak_energy_concentration",
                         path + ".range_profile.peak_energy_concentration", &present,
                         &out->peak_energy_concentration, error)) {
    return false;
  }
  out->minimum_bandwidth_hz =
      ColumnFloat(stmt, FindColumn(stmt, "range_profile_minimum_bandwidth_hz"), 0.0f);
  return true;
}

/** @brief 读取单行 profile 并校验（复合主键 (model_id, profile_id) 唯一性由 SQLite 保证）。 */
bool ReadProfileRow(sqlite3_stmt* stmt, RecognitionModelProfile* out, const std::string& path,
                    std::string* error) {
  const int profile_id_column = FindColumn(stmt, "profile_id");
  const int model_id_column = FindColumn(stmt, "model_id");
  if (profile_id_column < 0 || model_id_column < 0) {
    if (error != nullptr) {
      *error = path + ": profiles table missing profile_id/model_id column";
    }
    return false;
  }
  const unsigned char* profile_id_text = sqlite3_column_text(stmt, profile_id_column);
  const unsigned char* model_id_text = sqlite3_column_text(stmt, model_id_column);
  if (profile_id_text == nullptr || model_id_text == nullptr ||
      profile_id_text[0] == '\0' || model_id_text[0] == '\0') {
    if (error != nullptr) {
      *error = path + ".profiles: profile_id/model_id must be non-empty strings";
    }
    return false;
  }
  out->profile_id = reinterpret_cast<const char*>(profile_id_text);
  const std::string profile_path =
      path + ".profiles[" + out->profile_id + "]";
  out->min_snr_db = ColumnFloat(stmt, FindColumn(stmt, "min_snr_db"), 0.0f);
  out->max_range_resolution_m = ColumnFloat(stmt, FindColumn(stmt, "max_range_resolution_m"), 0.0f);
  if (!ReadRcsTemplate(stmt, profile_path, &out->rcs, error)) {
    return false;
  }
  if (!ReadMotionTemplate(stmt, profile_path, &out->motion, error)) {
    return false;
  }
  if (!ReadPolarizationTemplate(stmt, profile_path, &out->polarization, error)) {
    return false;
  }
  if (!ReadRangeProfileTemplate(stmt, profile_path, &out->range_profile, error)) {
    return false;
  }
  return true;
}

}  // namespace

bool RecognitionFeatureDatabase::Load(const std::string& path, RecognitionFeatureDatabase* database,
                                      std::string* error) {
  if (database == nullptr) {
    if (error != nullptr) {
      *error = "null database output";
    }
    return false;
  }

  // 只读打开（识别基线是只读数据；加载完成后连接即关闭）。
  sqlite3* raw_db = nullptr;
  if (sqlite3_open_v2(path.c_str(), &raw_db, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
    if (error != nullptr) {
      *error = path + ": cannot open sqlite database: " +
               (raw_db != nullptr ? sqlite3_errmsg(raw_db) : "unknown error");
    }
    if (raw_db != nullptr) {
      sqlite3_close(raw_db);
    }
    return false;
  }
  SqlitePtr db(raw_db);

  // 外键约束开启：models.category_id / profiles.model_id 引用由 SQLite 兜底。
  {
    char* sqlite_error = nullptr;
    if (sqlite3_exec(db.get(), "PRAGMA foreign_keys = ON", nullptr, nullptr,
                     &sqlite_error) != SQLITE_OK) {
      if (error != nullptr) {
        *error = path + ": enable foreign_keys: " +
                 (sqlite_error != nullptr ? sqlite_error : "unknown error");
      }
      sqlite3_free(sqlite_error);
      return false;
    }
  }

  RecognitionFeatureDatabase candidate;

  // -- meta 表：schema_version / database_id / version ------------------------
  {
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db.get(), "SELECT key, value FROM meta", -1, &stmt, nullptr) !=
        SQLITE_OK) {
      return FailWithSqliteError(path, db.get(), "missing or invalid meta table", error);
    }
    std::string schema_version;
    std::string database_id;
    std::string version;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      const unsigned char* key = sqlite3_column_text(stmt, 0);
      const unsigned char* value = sqlite3_column_text(stmt, 1);
      if (key == nullptr || value == nullptr) {
        continue;
      }
      const std::string key_str = reinterpret_cast<const char*>(key);
      const std::string value_str = reinterpret_cast<const char*>(value);
      if (key_str == "schema_version") {
        schema_version = value_str;
      } else if (key_str == "database_id") {
        database_id = value_str;
      } else if (key_str == "version") {
        version = value_str;
      }
    }
    sqlite3_finalize(stmt);
    if (schema_version.empty()) {
      if (error != nullptr) {
        *error = path + ".schema_version must be a non-empty string";
      }
      return false;
    }
    if (schema_version != kSupportedSchemaVersion) {
      if (error != nullptr) {
        *error = path + ".schema_version: unsupported version '" + schema_version +
                 "' (supported: " + kSupportedSchemaVersion + ")";
      }
      return false;
    }
    if (database_id.empty()) {
      if (error != nullptr) {
        *error = path + ".database_id must be a non-empty string";
      }
      return false;
    }
    if (version.empty()) {
      if (error != nullptr) {
        *error = path + ".version must be a non-empty string";
      }
      return false;
    }
    candidate.database_id_ = std::move(database_id);
    candidate.version_ = std::move(version);
  }

  // -- categories 表 -----------------------------------------------------------
  {
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db.get(), "SELECT category_id, prior FROM categories", -1, &stmt,
                           nullptr) != SQLITE_OK) {
      return FailWithSqliteError(path, db.get(), "missing or invalid categories table", error);
    }
    std::set<std::string> category_ids;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      const unsigned char* category_id_text = sqlite3_column_text(stmt, 0);
      if (category_id_text == nullptr || category_id_text[0] == '\0') {
        if (error != nullptr) {
          *error = path + ".categories: category_id must be a non-empty string";
        }
        sqlite3_finalize(stmt);
        return false;
      }
      RecognitionCategoryEntry entry;
      entry.category_id = reinterpret_cast<const char*>(category_id_text);
      if (!category_ids.insert(entry.category_id).second) {
        if (error != nullptr) {
          *error = path + ".categories[" + entry.category_id + "].category_id must be unique";
        }
        sqlite3_finalize(stmt);
        return false;
      }
      const int prior_index = FindColumn(stmt, "prior");
      const double prior = sqlite3_column_double(stmt, 1);
      if (prior_index >= 0 && sqlite3_column_type(stmt, prior_index) != SQLITE_NULL) {
        if (!(prior > 0.0)) {
          if (error != nullptr) {
            *error = path + ".categories[" + entry.category_id + "].prior must be > 0";
          }
          sqlite3_finalize(stmt);
          return false;
        }
        entry.prior = static_cast<float>(prior);
      }
      candidate.categories_.push_back(std::move(entry));
    }
    sqlite3_finalize(stmt);
  }

  // -- models 表（category 引用校验） ------------------------------------------
  std::unordered_map<std::string, std::size_t> model_index_by_id;
  {
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db.get(), "SELECT model_id, category_id, prior FROM models", -1,
                           &stmt, nullptr) != SQLITE_OK) {
      return FailWithSqliteError(path, db.get(), "missing or invalid models table", error);
    }
    std::set<std::string> model_ids;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      const unsigned char* model_id_text = sqlite3_column_text(stmt, 0);
      const unsigned char* category_id_text = sqlite3_column_text(stmt, 1);
      if (model_id_text == nullptr || model_id_text[0] == '\0') {
        if (error != nullptr) {
          *error = path + ".models: model_id must be a non-empty string";
        }
        sqlite3_finalize(stmt);
        return false;
      }
      if (category_id_text == nullptr || category_id_text[0] == '\0') {
        if (error != nullptr) {
          *error = path + ".models[" +
                   reinterpret_cast<const char*>(model_id_text) +
                   "].category_id must be a non-empty string";
        }
        sqlite3_finalize(stmt);
        return false;
      }
      RecognitionModel model;
      model.model_id = reinterpret_cast<const char*>(model_id_text);
      model.category_id = reinterpret_cast<const char*>(category_id_text);
      if (!model_ids.insert(model.model_id).second) {
        if (error != nullptr) {
          *error = path + ".models[" + model.model_id + "].model_id must be unique";
        }
        sqlite3_finalize(stmt);
        return false;
      }
      bool category_exists = false;
      for (const RecognitionCategoryEntry& category : candidate.categories_) {
        if (category.category_id == model.category_id) {
          category_exists = true;
          break;
        }
      }
      if (!category_exists) {
        if (error != nullptr) {
          *error = path + ".models[" + model.model_id + "].category_id references unknown category '" +
                   model.category_id + "'";
        }
        sqlite3_finalize(stmt);
        return false;
      }
      const int prior_index = FindColumn(stmt, "prior");
      if (prior_index >= 0 && sqlite3_column_type(stmt, prior_index) != SQLITE_NULL) {
        const double prior = sqlite3_column_double(stmt, prior_index);
        if (!(prior > 0.0)) {
          if (error != nullptr) {
            *error = path + ".models[" + model.model_id + "].prior must be > 0";
          }
          sqlite3_finalize(stmt);
          return false;
        }
        model.prior = static_cast<float>(prior);
      }
      model_index_by_id.emplace(model.model_id, candidate.models_.size());
      candidate.models_.push_back(std::move(model));
    }
    sqlite3_finalize(stmt);
  }

  // -- profiles 表（按 model_id 分组挂载） -------------------------------------
  {
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db.get(),
                           "SELECT profile_id, model_id, min_snr_db, max_range_resolution_m, "
                           "rcs_mean_dbsm, rcs_std_db, rcs_azimuth_variation_db, "
                           "rcs_elevation_variation_db, rcs_minimum_aspect_coverage_deg, "
                           "motion_speed_mean, motion_speed_std, motion_altitude_mean, "
                           "motion_altitude_std, motion_acceleration_mean, motion_acceleration_std, "
                           "motion_turn_radius_mean_log10, motion_turn_radius_std_log10, "
                           "polarization_energy_difference_mean, polarization_energy_difference_std, "
                           "polarization_relative_difference_mean, polarization_relative_difference_std, "
                           "polarization_energy_sum_mean, polarization_energy_sum_std, "
                           "range_profile_length_mean, range_profile_length_std, "
                           "range_profile_peak_count_mean, range_profile_peak_count_std, "
                           "range_profile_peak_energy_concentration_mean, "
                           "range_profile_peak_energy_concentration_std, "
                           "range_profile_minimum_bandwidth_hz "
                           "FROM profiles",
                           -1, &stmt, nullptr) != SQLITE_OK) {
      return FailWithSqliteError(path, db.get(), "missing or invalid profiles table", error);
    }
    std::unordered_map<std::string, std::set<std::string>> profile_ids_by_model;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
      RecognitionModelProfile profile;
      if (!ReadProfileRow(stmt, &profile, path, error)) {
        sqlite3_finalize(stmt);
        return false;
      }
      const unsigned char* model_id_text = sqlite3_column_text(stmt, FindColumn(stmt, "model_id"));
      const std::string model_id = reinterpret_cast<const char*>(model_id_text);
      // (model_id, profile_id) 模型内唯一：显式校验（与 JSON 版一致），SQLite 复合主键兜底。
      std::set<std::string>& model_profile_ids = profile_ids_by_model[model_id];
      if (!model_profile_ids.insert(profile.profile_id).second) {
        if (error != nullptr) {
          *error = path + ".profiles[" + profile.profile_id +
                   "].profile_id must be unique within the model";
        }
        sqlite3_finalize(stmt);
        return false;
      }
      const auto model_it = model_index_by_id.find(model_id);
      if (model_it == model_index_by_id.end()) {
        if (error != nullptr) {
          *error = path + ".profiles[" + profile.profile_id +
                   "].model_id references unknown model '" + model_id + "'";
        }
        sqlite3_finalize(stmt);
        return false;
      }
      candidate.models_[model_it->second].profiles.push_back(std::move(profile));
    }
    sqlite3_finalize(stmt);
  }

  // 全部校验通过后才提交候选数据库（全量原子替换语义）；连接随作用域关闭。
  candidate.loaded_ = true;
  *database = std::move(candidate);
  return true;
}

}  // namespace recognition
}  // namespace airborne_radar
