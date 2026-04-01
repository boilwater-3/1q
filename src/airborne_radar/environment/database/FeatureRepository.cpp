#include "airborne_radar/environment/database/FeatureRepository.h"

#include <algorithm>
#include <string>

#include "common/logging/ProjectLog.h"
#include <array>
#include <cmath>
#include <limits>
#include <numeric>

namespace airborne_radar {
namespace environment {
namespace database {

namespace {

/**
 * @brief `FeatureVector` 中表示速度维度的内建键名。
 * @note 代码行为依据：默认样本、行转换和距离计算都会通过该键访问速度特征。
 */
constexpr const char kSpeedFeature[] = "speed";
/**
 * @brief `FeatureVector` 中表示 RCS 维度的内建键名。
 * @note 代码行为依据：默认样本、行转换和距离计算都会通过该键访问 RCS 特征。
 */
constexpr const char kRcsFeature[] = "rcs";
/**
 * @brief `FeatureVector` 中表示干扰存在性的内建键名。
 * @note 代码行为依据：默认样本、行转换和距离计算都会通过该键访问干扰特征，
 *       并把布尔干扰事实映射为 `0.0f / 1.0f`。
 */
constexpr const char kJammingFeature[] = "jamming";

/**
 * @brief 仓储内建标准特征键集合。
 * @note 代码行为依据：`IsKnownFeature()` 依赖该集合把内建维度与扩展维度区分开，
 *       避免速度、RCS 与干扰键在附加距离循环中被重复累计。
 */
constexpr std::array<const char*, 3> kKnownFeatureKeys = {kSpeedFeature, kRcsFeature,
                                                          kJammingFeature};
/**
 * @brief 判断给定特征名是否属于仓储内建标准维度。
 * @param feature_name 待判定的特征名。
 * @return 命中标准维度时返回 true。
 */
bool IsKnownFeature(const std::string& feature_name) {
  return std::find(kKnownFeatureKeys.begin(), kKnownFeatureKeys.end(), feature_name) !=
         kKnownFeatureKeys.end();
}

}  // namespace

FeatureRepository::FeatureRepository() : records_(BuildDefaultRecords()) {}

bool FeatureRepository::ConnectDataSource(const std::string& connection_string) {
  // 数据库驱动尚未实现；连接请求始终返回 false，以防止调用方误以为连接已建立。
  PROJECT_LOG_WARN(
      "[FeatureRepository] ConnectDataSource called with '{}' but data source is not yet "
      "implemented; returning false.",
      connection_string);
  (void)connection_string;
  return false;
}

bool FeatureRepository::ReloadFromDataSource() {
  std::vector<RawFeatureRow> rows;
  if (!FetchRawRowsFromDataSource(rows)) {
    return false;
  }

  const auto transformed = TransformRowsToRecords(rows);
  if (transformed.empty()) {
    return false;
  }

  records_ = transformed;
  return true;
}

bool FeatureRepository::QueryBestMatch(const FeatureVector& input, MatchResult& result) const {
  if (records_.empty()) {
    result = MatchResult();
    return false;
  }

  std::vector<float> scores;
  scores.reserve(records_.size());

  float best_score = -1.0f;
  float best_distance = std::numeric_limits<float>::max();
  std::size_t best_index = 0;

  for (std::size_t i = 0; i < records_.size(); ++i) {
    const float distance = ComputeDistance(input, records_[i]);
    const float score = DistanceToScore(distance) * records_[i].prior;
    scores.push_back(score);

    if (score > best_score) {
      best_score = score;
      best_distance = distance;
      best_index = i;
    }
  }

  const float score_sum = std::accumulate(scores.begin(), scores.end(), 0.0f);
  constexpr float kScoreSumEpsilon = 1.0e-12f;
  if (!std::isfinite(score_sum) || score_sum <= kScoreSumEpsilon || !std::isfinite(best_score) ||
      !std::isfinite(best_distance)) {
    PROJECT_LOG_WARN(
        "[FeatureRepository] Invalid score normalization detected: score_sum={}, best_score={}, "
        "best_distance={}",
        score_sum, best_score, best_distance);
    result = MatchResult();
    return false;
  }

  result.target_type = records_[best_index].target_type;
  result.probability = best_score / score_sum;
  result.distance = best_distance;
  return true;
}

bool FeatureRepository::FetchRawRowsFromDataSource(std::vector<RawFeatureRow>& rows) const {
  rows.clear();
  // 预留：后续按甲方确定的数据库类型接入具体驱动与查询语句。
  return false;
}

std::vector<FeatureRecord> FeatureRepository::TransformRowsToRecords(
    const std::vector<RawFeatureRow>& rows) {
  std::vector<FeatureRecord> records;
  records.reserve(rows.size());

  for (const auto& row : rows) {
    FeatureRecord record;
    record.target_type = row.target_type;
    record.prototype = FeatureVector{{kSpeedFeature, row.speed},
                                     {kRcsFeature, row.rcs},
                                     {kJammingFeature, row.jamming_expected ? 1.0f : 0.0f}};
    record.sigma_speed = std::max(1.0f, row.sigma_speed);
    record.sigma_rcs = std::max(0.1f, row.sigma_rcs);
    record.prior = std::max(0.01f, row.prior);
    records.push_back(record);
  }

  return records;
}

std::vector<FeatureRecord> FeatureRepository::BuildDefaultRecords() {
  return {
      FeatureRecord{
          "HIGH_THREAT_FIGHTER",
          FeatureVector{{kSpeedFeature, 780.0f}, {kRcsFeature, 2.8f}, {kJammingFeature, 1.0f}},
          180.0f, 1.1f, 1.0f},
      FeatureRecord{
          "LOW_THREAT_TARGET",
          FeatureVector{{kSpeedFeature, 180.0f}, {kRcsFeature, 1.1f}, {kJammingFeature, 0.0f}},
          140.0f, 0.9f, 1.0f},
      FeatureRecord{
          "UNKNOWN",
          FeatureVector{{kSpeedFeature, 40.0f}, {kRcsFeature, 0.4f}, {kJammingFeature, 0.0f}},
          120.0f, 0.8f, 0.8f},
  };
}

float FeatureRepository::ComputeDistance(const FeatureVector& input, const FeatureRecord& record) {
  const float speed_term =
      std::fabs(input.GetOrDefault(kSpeedFeature) - record.prototype.GetOrDefault(kSpeedFeature)) /
      std::max(1.0f, record.sigma_speed);
  const float rcs_term =
      std::fabs(input.GetOrDefault(kRcsFeature) - record.prototype.GetOrDefault(kRcsFeature)) /
      std::max(0.1f, record.sigma_rcs);
  const float jamming_term = std::fabs(input.GetOrDefault(kJammingFeature) -
                                       record.prototype.GetOrDefault(kJammingFeature));

  float distance = 0.45f * speed_term + 0.35f * rcs_term + 0.20f * jamming_term;

  // 可扩展维度：仅当输入与原型均含该特征时才参与距离计算，避免单侧惩罚。
  for (const auto& entry : input.values) {
    if (IsKnownFeature(entry.first) || !record.prototype.Has(entry.first)) {
      continue;
    }
    distance += std::fabs(entry.second - record.prototype.GetOrDefault(entry.first));
  }

  return distance;
}

float FeatureRepository::DistanceToScore(float distance) { return std::exp(-distance); }

}  // namespace database
}  // namespace environment
}  // namespace airborne_radar
