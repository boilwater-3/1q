/**
 * @file RirSessionConfigValidation.cpp
 * @brief 远程识别雷达会话配置校验实现。
 *
 * 校验逻辑为 `ArSessionConfigBuilder.cpp` 识别校验段（审计基线 96de367c）
 * 的平移改写：识别策略字段路径改为 `policy.recognition.*` 域内、识别任务
 * 作用距离/驻留字段四域归位至 `mission.*` 域内；issue code 改为
 * `rir.validation.recognition_*`（RirIssueCodes.h），语义与门限值不变。
 */

#include "1q/remote_identification_radar/config/RirSessionConfigValidation.h"

#include <cmath>

#include "1q/remote_identification_radar/session/RirIssueCodes.h"

namespace remote_identification_radar {
namespace config {

namespace {

bool IsFinite(float value) { return std::isfinite(value); }

void PushIssue(session::RirIssueList* issues, const char* code, const char* field,
               const std::string& message) {
  session::RirIssue issue;
  issue.severity = session::RirIssueSeverity::kError;
  issue.phase = session::RirIssuePhase::kInputValidation;
  issue.code = code;
  issue.field = field;
  issue.message = message;
  issues->push_back(issue);
}

void ValidateRirEnvironmentConfig(const RirEnvironmentConfig& environment,
                                  const char* field_prefix, session::RirIssueList* issues) {
  const std::string prefix = field_prefix == nullptr ? "" : std::string(field_prefix) + ".";
  if (!IsFinite(environment.weather_attenuation_db) || environment.weather_attenuation_db < 0.0f) {
    PushIssue(issues, session::codes::kInvalidEnvironmentSnapshot,
              (prefix + "weather_attenuation_db").c_str(),
              "Weather attenuation must be finite and non-negative.");
  }
  const RirVegetationCoverProfile cover =
      environment.vegetation_scatter_physics.cover_profile;
  if (cover < RirVegetationCoverProfile::kDisabled ||
      cover > RirVegetationCoverProfile::kTropicalDense) {
    PushIssue(issues, session::codes::kInvalidEnvironmentSnapshot,
              (prefix + "vegetation_scatter_physics.cover_profile").c_str(),
              "Vegetation cover profile must be a valid enum value.");
  }
}

}  // namespace

session::RirIssueList ValidateRirSessionConfig(const RirSessionConfig& config) {
  session::RirIssueList issues;
  const RirMissionConfig& mission = config.mission;
  const RirRecognitionPolicy& recognition = config.policy.recognition;
  const RirRecognitionFeatureWeights& weights = recognition.feature_weights;

  // 权重：有限、[0, 1]、四者之和为 1.0。
  const float weight_sum = weights.rcs_weight + weights.motion_weight +
                           weights.polarization_weight + weights.range_profile_weight;
  if (!IsFinite(weights.rcs_weight) || !IsFinite(weights.motion_weight) ||
      !IsFinite(weights.polarization_weight) || !IsFinite(weights.range_profile_weight) ||
      weights.rcs_weight < 0.0f || weights.rcs_weight > 1.0f || weights.motion_weight < 0.0f ||
      weights.motion_weight > 1.0f || weights.polarization_weight < 0.0f ||
      weights.polarization_weight > 1.0f || weights.range_profile_weight < 0.0f ||
      weights.range_profile_weight > 1.0f || std::fabs(weight_sum - 1.0f) > 1.0e-5f) {
    PushIssue(&issues, session::codes::kRecognitionWeightsInvalid,
              "policy.recognition.feature_weights",
              "Recognition feature weights must be finite values in [0, 1] summing to 1.0.");
  }

  // 数据库路径：启用识别时必须非空。
  if (recognition.enabled && recognition.database_path.empty()) {
    PushIssue(&issues, session::codes::kRecognitionDatabasePathMissing,
              "policy.recognition.database_path",
              "Recognition database path must be non-empty when recognition is enabled.");
  }

  // 判定门限：[0, 1]。
  if (!IsFinite(recognition.acceptance_score) || recognition.acceptance_score < 0.0f ||
      recognition.acceptance_score > 1.0f || !IsFinite(recognition.minimum_margin) ||
      recognition.minimum_margin < 0.0f || recognition.minimum_margin > 1.0f) {
    PushIssue(&issues, session::codes::kRecognitionThresholdInvalid,
              "policy.recognition.acceptance_score / minimum_margin",
              "Recognition acceptance score and minimum margin must be finite values in [0, 1].");
  }

  // 积累计数：至少为 1。
  if (recognition.min_confirmed_hits == 0U || recognition.min_observation_count == 0U) {
    PushIssue(&issues, session::codes::kRecognitionAccumulationInvalid,
              "policy.recognition.min_confirmed_hits / min_observation_count",
              "Recognition accumulation counts must be at least 1.");
  }

  // 识别策略时间范围：保持时间非负；累积窗口有限且为正。
  if (!IsFinite(recognition.result_hold_sec) || recognition.result_hold_sec < 0.0f ||
      !IsFinite(recognition.accumulation_window_sec) ||
      recognition.accumulation_window_sec <= 0.0f) {
    PushIssue(&issues, session::codes::kRecognitionTimeRangeInvalid,
              "policy.recognition.result_hold_sec / accumulation_window_sec",
              "Recognition hold time must be non-negative and accumulation window must be "
              "finite and positive.");
  }

  // 任务域识别作用范围/驻留：最大距离与驻留时间有限且为正（四域归位后）。
  if (!IsFinite(mission.max_range_m) || mission.max_range_m <= 0.0f ||
      !IsFinite(mission.recognition_dwell_sec) || mission.recognition_dwell_sec <= 0.0f) {
    PushIssue(&issues, session::codes::kRecognitionTimeRangeInvalid,
              "mission.max_range_m / recognition_dwell_sec",
              "Recognition max range and dwell must be finite and positive.");
  }

  // 扫描策略（库内驻留调度器）：限位有限有序且在合法域（驻留中心契约
  // az∈[-180,180]、el∈[-90,90]），步长系数有限且为正。
  {
    const RirScanConfig& scan = mission.scan;
    if (!IsFinite(scan.scan_limits_deg.az_min_deg) ||
        !IsFinite(scan.scan_limits_deg.az_max_deg) ||
        !IsFinite(scan.scan_limits_deg.el_min_deg) ||
        !IsFinite(scan.scan_limits_deg.el_max_deg) ||
        scan.scan_limits_deg.az_min_deg > scan.scan_limits_deg.az_max_deg ||
        scan.scan_limits_deg.el_min_deg > scan.scan_limits_deg.el_max_deg ||
        scan.scan_limits_deg.az_min_deg < -180.0f || scan.scan_limits_deg.az_max_deg > 180.0f ||
        scan.scan_limits_deg.el_min_deg < -90.0f || scan.scan_limits_deg.el_max_deg > 90.0f ||
        !IsFinite(scan.step_scale) || scan.step_scale <= 0.0f) {
      PushIssue(&issues, session::codes::kScanStrategyInvalid, "mission.scan",
                "Scan limits must be finite, ordered and within az[-180,180]/el[-90,90]; step "
                "scale must be finite and positive.");
    }
  }

  // 自持检测策略（阶段 2-S）：Pfa/SNR 门限有限且范围合理，脉冲数/种子为正。
  {
    const RirDetectionPolicyConfig& detection = config.policy.detection;
    if (!IsFinite(detection.cfar_pfa) || detection.cfar_pfa <= 0.0f || detection.cfar_pfa > 1.0f ||
        !IsFinite(detection.min_snr_db) || !IsFinite(detection.min_detection_margin_db) ||
        detection.pulse_count <= 0 || detection.random_seed == 0U) {
      PushIssue(&issues, session::codes::kDetectionPolicyInvalid, "policy.detection",
                "Detection policy Pfa/SNR gates must be finite, pulse count and seed positive.");
    }
  }

  // 关联策略：波门 sigma 有限且 > 0。
  {
    const RirAssociationPolicyConfig& association = config.policy.association;
    if (!IsFinite(association.distance_gate_sigma) || association.distance_gate_sigma <= 0.0f) {
      PushIssue(&issues, session::codes::kAssociationPolicyInvalid, "policy.association",
                "Association distance gate sigma must be finite and positive.");
    }
  }

  // 跟踪策略：KF 噪声参数有限且 > 0。
  {
    const RirTrackingPolicyConfig& tracking = config.policy.tracking;
    if (!IsFinite(tracking.kalman_noise_diff_coeff) || tracking.kalman_noise_diff_coeff <= 0.0f ||
        !IsFinite(tracking.kalman_measurement_noise_std) ||
        tracking.kalman_measurement_noise_std <= 0.0f) {
      PushIssue(&issues, session::codes::kTrackingPolicyInvalid, "policy.tracking",
                "Kalman process/measurement noise parameters must be finite and positive.");
    }
  }

  // 生命周期策略：confirm_hits/max_lost_cycles 至少为 1。
  {
    const RirLifecyclePolicyConfig& lifecycle = config.policy.lifecycle;
    if (lifecycle.confirm_hits == 0U || lifecycle.max_lost_cycles == 0U) {
      PushIssue(&issues, session::codes::kLifecyclePolicyInvalid, "policy.lifecycle",
                "Lifecycle confirm hits and max lost cycles must be at least 1.");
    }
  }

  // 信号处理增益偏置（阶段 2-M M3）：有限且 [0, 40] dB。
  {
    const RirHardwareConfig& hardware = config.hardware;
    const hardware::RirSignalProcessingConfig& gains = hardware.signal_processing;
    if (!IsFinite(gains.target_processing_gain_db) || !IsFinite(gains.noise_processing_gain_db) ||
        !IsFinite(gains.clutter_suppression_gain_db) ||
        !IsFinite(gains.jamming_suppression_gain_db) || gains.target_processing_gain_db < 0.0f ||
        gains.target_processing_gain_db > 40.0f || gains.noise_processing_gain_db < 0.0f ||
        gains.noise_processing_gain_db > 40.0f || gains.clutter_suppression_gain_db < 0.0f ||
        gains.clutter_suppression_gain_db > 40.0f || gains.jamming_suppression_gain_db < 0.0f ||
        gains.jamming_suppression_gain_db > 40.0f) {
      PushIssue(&issues, session::codes::kSignalProcessingGainsInvalid,
                "hardware.signal_processing",
                "Signal processing gain offsets must be finite values in [0, 40] dB.");
    }
  }

  ValidateRirEnvironmentConfig(config.environment, "environment", &issues);

  return issues;
}

}  // namespace config
}  // namespace remote_identification_radar
