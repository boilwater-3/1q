#include "electronic_surveillance_radar/pipeline/ObservationPreprocessor.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "common/geometry/GeometryTransform.h"

namespace electronic_surveillance_radar {
namespace pipeline {
namespace internal {

namespace {
constexpr double kTimeCompareEpsilonSec = 1.0e-9;
constexpr double kRfCompareEpsilonHz = 1.0e-3;
constexpr double kPwCompareEpsilonSec = 1.0e-12;
constexpr float kAngleCompareEpsilonDeg = 1.0e-6f;

bool LessOrNearEqual(double lhs, double rhs, double epsilon) {
  return lhs <= rhs + epsilon;
}

bool LessOrNearEqual(float lhs, float rhs, float epsilon) {
  return lhs <= rhs + epsilon;
}

/**
 * @brief 判断观测字段是否满足有限值与范围约束。
 * @param[in] observation 输入观测。
 * @return 合法时返回 `true`。
 */
bool IsValidObservation(const model::EmitterObservation& observation) {
  if (!std::isfinite(observation.timestamp_s) || !std::isfinite(observation.aoa_az_deg) ||
      !std::isfinite(observation.aoa_el_deg) || !std::isfinite(observation.rf_hz) ||
      !std::isfinite(observation.pulse_width_s) || !std::isfinite(observation.amplitude_db) ||
      !std::isfinite(observation.snr_db)) {
    return false;
  }
  return observation.rf_hz > 0.0 && observation.pulse_width_s >= 0.0;
}

/**
 * @brief 根据观测信噪比重标定质量等级。
 * @param[in] snr_db 观测信噪比（单位：dB）。
 * @return 重标定后的质量等级。
 */
model::EsrObservationQuality NormalizeQuality(float snr_db) {
  if (snr_db >= 18.0f) {
    return model::EsrObservationQuality::kHigh;
  }
  if (snr_db >= 10.0f) {
    return model::EsrObservationQuality::kMedium;
  }
  return model::EsrObservationQuality::kLow;
}

/**
 * @brief 判断两条观测是否满足去重窗口约束。
 * @param[in] lhs 左侧观测。
 * @param[in] rhs 右侧观测。
 * @param[in] config 预处理配置。
 * @return 满足去重条件时返回 `true`。
 */
bool IsDuplicateObservation(const model::EmitterObservation& lhs,
                            const model::EmitterObservation& rhs,
                            const extension::InterceptPreprocessConfig& config) {
  const double dedup_time_window_sec = static_cast<double>(config.dedup_time_window_sec);
  const float az_diff = std::fabs(
      oneq::internal::geometry::ComputeAzimuthDifferenceDeg(lhs.aoa_az_deg, rhs.aoa_az_deg));
  return LessOrNearEqual(std::fabs(lhs.timestamp_s - rhs.timestamp_s), dedup_time_window_sec,
                         kTimeCompareEpsilonSec) &&
         LessOrNearEqual(std::fabs(lhs.rf_hz - rhs.rf_hz), config.dedup_rf_window_hz,
                         kRfCompareEpsilonHz) &&
         LessOrNearEqual(std::fabs(lhs.pulse_width_s - rhs.pulse_width_s),
                         config.dedup_pw_window_sec, kPwCompareEpsilonSec) &&
         LessOrNearEqual(az_diff, config.dedup_az_window_deg, kAngleCompareEpsilonDeg) &&
         LessOrNearEqual(std::fabs(lhs.aoa_el_deg - rhs.aoa_el_deg),
                         config.dedup_el_window_deg, kAngleCompareEpsilonDeg);
}

/**
 * @brief 比较两条记录优先级，优先保留更高 SNR 的观测。
 * @param[in] lhs 左侧记录。
 * @param[in] rhs 右侧记录。
 * @return 若 lhs 优先于 rhs 则返回 `true`。
 */
bool IsPreferredRecord(const RawObservationRecord& lhs, const RawObservationRecord& rhs) {
  if (lhs.observation.snr_db != rhs.observation.snr_db) {
    return lhs.observation.snr_db > rhs.observation.snr_db;
  }
  return lhs.observation.observation_id < rhs.observation.observation_id;
}

/**
 * @brief 按时间与观测 ID 升序排序。
 * @param[in] lhs 左侧记录。
 * @param[in] rhs 右侧记录。
 * @return 排序谓词结果。
 */
bool CompareRecordByTime(const RawObservationRecord& lhs, const RawObservationRecord& rhs) {
  if (lhs.observation.timestamp_s != rhs.observation.timestamp_s) {
    return lhs.observation.timestamp_s < rhs.observation.timestamp_s;
  }
  return lhs.observation.observation_id < rhs.observation.observation_id;
}

}  // namespace

std::vector<RawObservationRecord> ObservationPreprocessor::Run(
    const std::vector<RawObservationRecord>& records,
    const extension::InterceptPreprocessConfig& config) const {
  std::vector<RawObservationRecord> sorted_records = records;
  std::sort(sorted_records.begin(), sorted_records.end(), CompareRecordByTime);

  std::vector<RawObservationRecord> output;
  output.reserve(sorted_records.size());
  for (std::size_t i = 0; i < sorted_records.size(); ++i) {
    RawObservationRecord current = sorted_records[i];
    if (!IsValidObservation(current.observation)) {
      continue;
    }

    if (config.normalize_quality) {
      current.observation.quality = NormalizeQuality(current.observation.snr_db);
    }

    bool consumed = false;
    for (std::size_t j = output.size(); j > 0; --j) {
      RawObservationRecord& candidate = output[j - 1U];
      if (current.observation.timestamp_s - candidate.observation.timestamp_s >
          static_cast<double>(config.dedup_time_window_sec) + kTimeCompareEpsilonSec) {
        break;
      }

      if (!IsDuplicateObservation(current.observation, candidate.observation, config)) {
        continue;
      }

      if (IsPreferredRecord(current, candidate)) {
        candidate = current;
      }
      consumed = true;
      break;
    }

    if (!consumed) {
      output.push_back(current);
    }
  }

  return output;
}

}  // namespace internal
}  // namespace pipeline
}  // namespace electronic_surveillance_radar
