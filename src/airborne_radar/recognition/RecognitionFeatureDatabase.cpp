/**
 * @file RecognitionFeatureDatabase.cpp
 * @brief 目标特征数据库加载与校验实现。
 */

#include "airborne_radar/recognition/RecognitionFeatureDatabase.h"

#include <cmath>
#include <set>

#include "airborne_radar/recognition/RecognitionJsonParser.h"

namespace airborne_radar {
namespace recognition {

namespace {

/** @brief 受支持的数据库 schema 版本。 */
constexpr const char* kSupportedSchemaVersion = "1.0";

bool RequireDouble(const RecognitionJsonValue& object, const char* key, const std::string& path,
                   float* out, std::string* error) {
  const RecognitionJsonValue& value = object[key];
  if (!value.IsDouble()) {
    *error = path + "." + key + " must be a finite number";
    return false;
  }
  const double raw = value.AsDouble();
  if (!std::isfinite(raw)) {
    *error = path + "." + key + " must be a finite number";
    return false;
  }
  *out = static_cast<float>(raw);
  return true;
}

bool RequireString(const RecognitionJsonValue& object, const char* key, const std::string& path,
                   std::string* out, std::string* error) {
  const RecognitionJsonValue& value = object[key];
  if (!value.IsString() || value.AsString().empty()) {
    *error = path + "." + key + " must be a non-empty string";
    return false;
  }
  *out = value.AsString();
  return true;
}

bool ParseTemplate(const RecognitionJsonValue& object, const std::string& path,
                   RecognitionFeatureTemplate* out, std::string* error) {
  float std_value = 0.0f;
  if (!RequireDouble(object, "std", path, &std_value, error)) {
    return false;
  }
  if (std_value <= 0.0f) {
    *error = path + ".std must be > 0";
    return false;
  }
  out->std = std_value;
  if (object.Has("mean")) {
    float mean_value = 0.0f;
    if (!RequireDouble(object, "mean", path, &mean_value, error)) {
      return false;
    }
    out->mean = mean_value;
  }
  return true;
}

bool ParseMotionTemplate(const RecognitionJsonValue& object, const std::string& path,
                         RecognitionMotionTemplate* out, std::string* error) {
  if (object.Has("speed_mps") &&
      !ParseTemplate(object["speed_mps"], path + ".speed_mps", &out->speed_mps, error)) {
    return false;
  }
  if (object.Has("altitude_m") &&
      !ParseTemplate(object["altitude_m"], path + ".altitude_m", &out->altitude_m, error)) {
    return false;
  }
  if (object.Has("acceleration_mps2") &&
      !ParseTemplate(object["acceleration_mps2"], path + ".acceleration_mps2",
                     &out->acceleration_mps2, error)) {
    return false;
  }
  if (object.Has("turn_radius_m")) {
    // 转弯半径按对数尺度比对（§7.4：mean_log10/std_log10）。
    const RecognitionJsonValue& turn_radius = object["turn_radius_m"];
    const std::string radius_path = path + ".turn_radius_m";
    float std_value = 0.0f;
    if (!RequireDouble(turn_radius, "std_log10", radius_path, &std_value, error)) {
      return false;
    }
    if (std_value <= 0.0f) {
      *error = radius_path + ".std_log10 must be > 0";
      return false;
    }
    out->turn_radius_log10.std = std_value;
    if (turn_radius.Has("mean_log10")) {
      float mean_value = 0.0f;
      if (!RequireDouble(turn_radius, "mean_log10", radius_path, &mean_value, error)) {
        return false;
      }
      out->turn_radius_log10.mean = mean_value;
    }
  }
  return true;
}

bool ParsePolarizationTemplate(const RecognitionJsonValue& object, const std::string& path,
                               RecognitionPolarizationTemplate* out, std::string* error) {
  if (object.Has("energy_difference_db") &&
      !ParseTemplate(object["energy_difference_db"], path + ".energy_difference_db",
                     &out->energy_difference_db, error)) {
    return false;
  }
  if (object.Has("relative_difference_db") &&
      !ParseTemplate(object["relative_difference_db"], path + ".relative_difference_db",
                     &out->relative_difference_db, error)) {
    return false;
  }
  if (object.Has("energy_sum_db") &&
      !ParseTemplate(object["energy_sum_db"], path + ".energy_sum_db", &out->energy_sum_db,
                     error)) {
    return false;
  }
  return true;
}

bool ParseRangeProfileTemplate(const RecognitionJsonValue& object, const std::string& path,
                               RecognitionRangeProfileTemplate* out, std::string* error) {
  if (object.Has("length_m") &&
      !ParseTemplate(object["length_m"], path + ".length_m", &out->length_m, error)) {
    return false;
  }
  if (object.Has("peak_count") &&
      !ParseTemplate(object["peak_count"], path + ".peak_count", &out->peak_count, error)) {
    return false;
  }
  if (object.Has("peak_energy_concentration") &&
      !ParseTemplate(object["peak_energy_concentration"], path + ".peak_energy_concentration",
                     &out->peak_energy_concentration, error)) {
    return false;
  }
  if (object.Has("minimum_bandwidth_hz")) {
    float bandwidth = 0.0f;
    if (!RequireDouble(object, "minimum_bandwidth_hz", path, &bandwidth, error)) {
      return false;
    }
    out->minimum_bandwidth_hz = bandwidth;
  }
  return true;
}

bool ParseProfile(const RecognitionJsonValue& object, const std::string& path,
                  RecognitionModelProfile* out, std::string* error) {
  if (!RequireString(object, "profile_id", path, &out->profile_id, error)) {
    return false;
  }
  if (object.Has("applicability")) {
    const RecognitionJsonValue& applicability = object["applicability"];
    const std::string app_path = path + ".applicability";
    if (applicability.Has("min_snr_db") &&
        !RequireDouble(applicability, "min_snr_db", app_path, &out->min_snr_db, error)) {
      return false;
    }
    if (applicability.Has("max_range_resolution_m") &&
        !RequireDouble(applicability, "max_range_resolution_m", app_path,
                       &out->max_range_resolution_m, error)) {
      return false;
    }
  }
  if (object.Has("rcs")) {
    const RecognitionJsonValue& rcs = object["rcs"];
    const std::string rcs_path = path + ".rcs";
    if (rcs.Has("mean_dbsm") &&
        !RequireDouble(rcs, "mean_dbsm", rcs_path, &out->rcs.mean_dbsm, error)) {
      return false;
    }
    if (rcs.Has("std_db")) {
      float std_db = 0.0f;
      if (!RequireDouble(rcs, "std_db", rcs_path, &std_db, error)) {
        return false;
      }
      if (std_db <= 0.0f) {
        *error = rcs_path + ".std_db must be > 0";
        return false;
      }
      out->rcs.std_db = std_db;
    }
    if (rcs.Has("azimuth_variation_db") &&
        !RequireDouble(rcs, "azimuth_variation_db", rcs_path,
                       &out->rcs.azimuth_variation_db, error)) {
      return false;
    }
    if (rcs.Has("elevation_variation_db") &&
        !RequireDouble(rcs, "elevation_variation_db", rcs_path,
                       &out->rcs.elevation_variation_db, error)) {
      return false;
    }
    if (rcs.Has("minimum_aspect_coverage_deg") &&
        !RequireDouble(rcs, "minimum_aspect_coverage_deg", rcs_path,
                       &out->rcs.minimum_aspect_coverage_deg, error)) {
      return false;
    }
  }
  if (object.Has("motion") &&
      !ParseMotionTemplate(object["motion"], path + ".motion", &out->motion, error)) {
    return false;
  }
  if (object.Has("polarization") &&
      !ParsePolarizationTemplate(object["polarization"], path + ".polarization",
                                 &out->polarization, error)) {
    return false;
  }
  if (object.Has("range_profile") &&
      !ParseRangeProfileTemplate(object["range_profile"], path + ".range_profile",
                                 &out->range_profile, error)) {
    return false;
  }
  return true;
}

bool ParseModel(const RecognitionJsonValue& object, const std::string& path,
                RecognitionModel* out, std::string* error) {
  if (!RequireString(object, "model_id", path, &out->model_id, error)) {
    return false;
  }
  if (!RequireString(object, "category_id", path, &out->category_id, error)) {
    return false;
  }
  if (object.Has("prior")) {
    float prior = 0.0f;
    if (!RequireDouble(object, "prior", path, &prior, error)) {
      return false;
    }
    if (prior <= 0.0f) {
      *error = path + ".prior must be > 0";
      return false;
    }
    out->prior = prior;
  }
  const RecognitionJsonValue& profiles = object["profiles"];
  if (!profiles.IsArray() || profiles.Size() == 0U) {
    *error = path + ".profiles must be a non-empty array";
    return false;
  }
  std::set<std::string> profile_ids;
  for (std::size_t i = 0U; i < profiles.Size(); ++i) {
    RecognitionModelProfile profile;
    const std::string profile_path =
        path + ".profiles[" + std::to_string(i) + "]";
    if (!ParseProfile(profiles[i], profile_path, &profile, error)) {
      return false;
    }
    if (!profile_ids.insert(profile.profile_id).second) {
      *error = profile_path + ".profile_id must be unique within the model";
      return false;
    }
    out->profiles.push_back(std::move(profile));
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
  RecognitionJsonValue root;
  if (!RecognitionJsonReader::ParseFile(path, &root, error)) {
    if (error != nullptr && error->find(path) == std::string::npos) {
      *error = path + ": " + *error;
    }
    return false;
  }
  if (!root.IsObject()) {
    if (error != nullptr) {
      *error = path + ": top-level value must be an object";
    }
    return false;
  }

  RecognitionFeatureDatabase candidate;
  {
    std::string schema_version;
    if (!RequireString(root, "schema_version", path, &schema_version, error)) {
      return false;
    }
    if (schema_version != kSupportedSchemaVersion) {
      if (error != nullptr) {
        *error = path + ".schema_version: unsupported version '" + schema_version +
                 "' (supported: " + kSupportedSchemaVersion + ")";
      }
      return false;
    }
  }
  if (!RequireString(root, "database_id", path, &candidate.database_id_, error)) {
    return false;
  }
  if (!RequireString(root, "version", path, &candidate.version_, error)) {
    return false;
  }

  std::set<std::string> category_ids;
  const RecognitionJsonValue& categories = root["categories"];
  if (categories.IsArray()) {
    for (std::size_t i = 0U; i < categories.Size(); ++i) {
      RecognitionCategoryEntry entry;
      const std::string entry_path = path + ".categories[" + std::to_string(i) + "]";
      if (!RequireString(categories[i], "category_id", entry_path, &entry.category_id, error)) {
        return false;
      }
      if (!category_ids.insert(entry.category_id).second) {
        *error = entry_path + ".category_id must be unique";
        return false;
      }
      if (categories[i].Has("prior")) {
        float prior = 0.0f;
        if (!RequireDouble(categories[i], "prior", entry_path, &prior, error)) {
          return false;
        }
        if (prior <= 0.0f) {
          *error = entry_path + ".prior must be > 0";
          return false;
        }
        entry.prior = prior;
      }
      candidate.categories_.push_back(std::move(entry));
    }
  }

  const RecognitionJsonValue& models = root["models"];
  if (!models.IsArray()) {
    if (error != nullptr) {
      *error = path + ".models must be an array";
    }
    return false;
  }
  std::set<std::string> model_ids;
  for (std::size_t i = 0U; i < models.Size(); ++i) {
    RecognitionModel model;
    const std::string model_path = path + ".models[" + std::to_string(i) + "]";
    if (!ParseModel(models[i], model_path, &model, error)) {
      return false;
    }
    if (!model_ids.insert(model.model_id).second) {
      *error = model_path + ".model_id must be unique";
      return false;
    }
    if (category_ids.count(model.category_id) == 0U) {
      *error = model_path + ".category_id references unknown category '" +
               model.category_id + "'";
      return false;
    }
    candidate.models_.push_back(std::move(model));
  }

  // 全部校验通过后才提交候选数据库（全量原子替换语义）。
  candidate.loaded_ = true;
  *database = std::move(candidate);
  return true;
}

}  // namespace recognition
}  // namespace airborne_radar
