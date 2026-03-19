// Copyright 2026. All Rights Reserved.
//
// @file FeatureRepository.cpp
// @brief 实现 FeatureRepository 的特征仓储加载与查询逻辑。

#include "airborne_radar/environment/database/FeatureRepository.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>

namespace airborne_radar { namespace environment { namespace database {

namespace {

constexpr const char kSpeedFeature[] = "speed";
constexpr const char kRcsFeature[] = "rcs";
constexpr const char kJammingFeature[] = "jamming";

constexpr std::array<const char *, 3> kKnownFeatureKeys = {
		kSpeedFeature, kRcsFeature, kJammingFeature};

/// @brief 判断给定特征名是否属于仓储内建标准维度。
/// @param feature_name 待判定的特征名。
/// @return 命中标准维度时返回 true。
bool IsKnownFeature(const std::string &feature_name) {
	return std::find(kKnownFeatureKeys.begin(), kKnownFeatureKeys.end(),
									 feature_name) != kKnownFeatureKeys.end();
}

} // namespace

FeatureRepository::FeatureRepository() : records_(BuildDefaultRecords()) {}

bool FeatureRepository::ConnectDataSource(const std::string &connection_string) {
	connection_string_ = connection_string;
	return !connection_string_.empty();
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

bool FeatureRepository::QueryBestMatch(const FeatureVector &input,
																			 MatchResult &result) const {
	if (records_.empty()) {
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

	const float score_sum =
			std::accumulate(scores.begin(), scores.end(), 0.0f);
	if (score_sum <= 0.0f) {
		return false;
	}

	result.target_type = records_[best_index].target_type;
	result.probability = best_score / score_sum;
	result.distance = best_distance;
	return true;
}

bool FeatureRepository::FetchRawRowsFromDataSource(
		std::vector<RawFeatureRow> &rows) const {
	rows.clear();

	// 预留：后续按甲方确定的数据库类型接入具体驱动与查询语句。
	if (connection_string_.empty()) {
		return false;
	}

	return false;
}

std::vector<FeatureRecord> FeatureRepository::TransformRowsToRecords(
		const std::vector<RawFeatureRow> &rows) {
	std::vector<FeatureRecord> records;
	records.reserve(rows.size());

	for (const auto &row : rows) {
		FeatureRecord record;
		record.target_type = row.target_type;
		record.prototype = FeatureVector{{kSpeedFeature, row.speed},
		                               {kRcsFeature, row.rcs},
		                               {kJammingFeature,
		                                row.jamming_expected ? 1.0f : 0.0f}};
		record.sigma_speed = std::max(1.0f, row.sigma_speed);
		record.sigma_rcs = std::max(0.1f, row.sigma_rcs);
		record.prior = std::max(0.01f, row.prior);
		records.push_back(record);
	}

	return records;
}

std::vector<FeatureRecord> FeatureRepository::BuildDefaultRecords() {
	return {
			FeatureRecord{"HIGH_THREAT_FIGHTER",
			            FeatureVector{{kSpeedFeature, 780.0f},
			                         {kRcsFeature, 2.8f},
			                         {kJammingFeature, 1.0f}},
										180.0f, 1.1f, 1.0f},
			FeatureRecord{"LOW_THREAT_TARGET",
			            FeatureVector{{kSpeedFeature, 180.0f},
			                         {kRcsFeature, 1.1f},
			                         {kJammingFeature, 0.0f}},
										140.0f, 0.9f, 1.0f},
			FeatureRecord{"UNKNOWN",
			            FeatureVector{{kSpeedFeature, 40.0f},
			                         {kRcsFeature, 0.4f},
			                         {kJammingFeature, 0.0f}},
			            120.0f, 0.8f, 0.8f},
	};
}

float FeatureRepository::ComputeDistance(const FeatureVector &input,
																				 const FeatureRecord &record) {
	const float speed_term =
			std::fabs(input.GetOrDefault(kSpeedFeature) -
							record.prototype.GetOrDefault(kSpeedFeature)) /
			std::max(1.0f, record.sigma_speed);
	const float rcs_term =
			std::fabs(input.GetOrDefault(kRcsFeature) -
							record.prototype.GetOrDefault(kRcsFeature)) /
			std::max(0.1f, record.sigma_rcs);
	const float jamming_term =
			std::fabs(input.GetOrDefault(kJammingFeature) -
							record.prototype.GetOrDefault(kJammingFeature));

	float distance = 0.45f * speed_term + 0.35f * rcs_term + 0.20f * jamming_term;

	// 可扩展维度：除已知特征外，其余按单位尺度参与距离计算。
	for (const auto &entry : input.values) {
		if (IsKnownFeature(entry.first) || !record.prototype.Has(entry.first)) {
			continue;
		}
		distance +=
				std::fabs(entry.second - record.prototype.GetOrDefault(entry.first));
	}

	for (const auto &entry : record.prototype.values) {
		if (IsKnownFeature(entry.first) || input.Has(entry.first)) {
			continue;
		}
		distance += std::fabs(entry.second);
	}

	return distance;
}

float FeatureRepository::DistanceToScore(float distance) {
	return std::exp(-distance);
}

} } } // namespace airborne_radar::environment::database
