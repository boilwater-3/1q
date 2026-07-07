#include "sbirs_sensor/pipeline/SbirsPipeline.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "sbirs_sensor/environment/SbirsEnvironmentModel.h"
#include "sbirs_sensor/foundation/SbirsErrorModel.h"
#include "sbirs_sensor/foundation/SbirsGeometry.h"
#include "sbirs_sensor/foundation/SbirsNoiseModel.h"
#include "sbirs_sensor/foundation/SbirsRadiometry.h"

namespace sbirs_sensor {
namespace pipeline {
namespace {

const double kEarthRadiusM = 6371000.0;

float NormalizeAzimuth(float azimuth_deg) {
  float result = std::fmod(azimuth_deg + 180.0f, 360.0f);
  if (result < 0.0f) {
    result += 360.0f;
  }
  return result - 180.0f;
}

float AzimuthDelta(float lhs_deg, float rhs_deg) { return NormalizeAzimuth(lhs_deg - rhs_deg); }

bool InRectangularFov(float target_az_deg, float target_el_deg, float center_az_deg,
                      float center_el_deg, float fov_az_deg, float fov_el_deg) {
  return std::fabs(AzimuthDelta(target_az_deg, center_az_deg)) <= 0.5f * fov_az_deg &&
         std::fabs(target_el_deg - center_el_deg) <= 0.5f * fov_el_deg;
}

double ComputeSnr(const config::SbirsInternalExecutionConfig& config,
                  const session::SbirsSceneTarget& target, double range_m, float transmittance) {
  const config::SbirsHardwareConfig& hardware = config.session.hardware;
  const double band_radiance = foundation::ComputeBandRadiance(
      hardware.wavelength_lower_um, hardware.wavelength_upper_um, target.temperature_k);
  const double received_power = foundation::ComputeReceivedPowerW(
      band_radiance * std::max(0.0f, target.emissivity), target.projected_area_m2, range_m,
      hardware.optical_aperture_m, hardware.optical_transmission, transmittance,
      hardware.detector_quantum_efficiency);
  // 2.8 噪声分解：背景/热/读出三项 RMS 合成；默认全 0 时回退到 NEP 标量。
  const foundation::SbirsNoiseStatistics noise =
      foundation::ComputeBackgroundNoiseStatistics(hardware);
  const double effective_noise = foundation::ResolveEffectiveNoiseW(hardware, noise);
  const double signal_energy =
      std::max(0.0, received_power) * std::max(0.0f, hardware.integration_time_sec);
  return signal_energy / effective_noise;
}

struct Candidate {
  const session::SbirsSceneTarget* target{nullptr};
  float azimuth_deg{0.0f};
  float elevation_deg{0.0f};
  float predicted_azimuth_deg{0.0f};   // cue 延迟外推后的真值方位角（无速度时等同 azimuth_deg）
  float predicted_elevation_deg{0.0f};  // cue 延迟外推后的真值俯仰角（无速度时等同 elevation_deg）
  float measured_azimuth_deg{0.0f};
  float measured_elevation_deg{0.0f};
  double range_m{0.0};        // 真值距离（调度优先级用）
  double measured_range_m{0.0};  // 带误差距离（NFOV cue 与 attribution 诊断用）
  double snr{0.0};
};

}  // namespace

SbirsPipeline::SbirsPipeline(const config::SbirsInternalExecutionConfig& config)
    : config_(config),
      scan_azimuth_deg_(config.session.mission.scan_start_az_deg),
      random_source_(config.session.policy.error_model.random_seed) {}

void SbirsPipeline::ApplyConfig(const config::SbirsInternalExecutionConfig& config) {
  config_ = config;
  // 配置变更后重置随机源种子，保证 runtime patch 后的 replay 可复现。
  random_source_ = foundation::SbirsRandomSource(config.session.policy.error_model.random_seed);
}

SbirsPipelineResult SbirsPipeline::RunCycle(const session::SbirsCycleInput& input) {
  SbirsPipelineResult result;
  const config::SbirsMissionConfig& mission = config_.session.mission;
  const config::SbirsPolicyConfig& policy = config_.session.policy;
  const config::SbirsEnvironmentConfig environment_config =
      input.environment.has_environment_override ? input.environment.environment
                                                 : config_.session.environment;

  if (mission.work_mode == config::SbirsWorkMode::kStandby || !mission.sensor_enabled) {
    target_states_.clear();
    has_locked_target_ = false;
    locked_target_id_ = 0U;
    result.scan_azimuth_deg = scan_azimuth_deg_;
    return result;
  }

  scan_azimuth_deg_ = NormalizeAzimuth(scan_azimuth_deg_ + mission.scan_rate_deg_per_sec *
                                                               std::max(0.0f, input.dt_sec));
  if (scan_azimuth_deg_ > mission.scan_end_az_deg) {
    scan_azimuth_deg_ = mission.scan_start_az_deg;
  }
  result.scan_azimuth_deg = scan_azimuth_deg_;

  const float transmittance = environment::ResolveEffectiveTransmittance(environment_config);
  std::vector<Candidate> candidates;

  for (const session::SbirsSceneTarget& target : input.scene) {
    if (!target.active) {
      target_states_[target.target_id] = SbirsTargetState::kLost;
      if (has_locked_target_ && locked_target_id_ == target.target_id) {
        has_locked_target_ = false;
      }
      continue;
    }

    if (foundation::IsEarthOcculted(input.satellite_position_ecef_m, target.position_ecef_m,
                                    kEarthRadiusM)) {
      target_states_[target.target_id] = SbirsTargetState::kUndetected;
      continue;
    }

    const session::SbirsVector3M los =
        foundation::Subtract(target.position_ecef_m, input.satellite_position_ecef_m);
    const double range_m = foundation::Norm(los);
    if (range_m < mission.min_range_m || range_m > mission.max_range_m) {
      target_states_[target.target_id] = SbirsTargetState::kUndetected;
      continue;
    }

    const float azimuth_deg = foundation::ComputeAzimuthDeg(los);
    const float elevation_deg = foundation::ComputeElevationDeg(los);
    const double snr = ComputeSnr(config_, target, range_m, transmittance);

    const bool in_wfov =
        InRectangularFov(azimuth_deg, elevation_deg, scan_azimuth_deg_, mission.scan_center_el_deg,
                         mission.wide_field_fov_az_deg, mission.wide_field_fov_el_deg);

    const bool is_locked =
        has_locked_target_ && locked_target_id_ == target.target_id &&
        target_states_[target.target_id] == SbirsTargetState::kTruthAssistedTracking;
    if (is_locked) {
      SbirsPipelineDetection detection;
      detection.record.detection_id = next_detection_id_++;
      detection.record.azimuth_deg = azimuth_deg;
      detection.record.elevation_deg = elevation_deg;
      detection.record.infrared_snr_linear = static_cast<float>(snr);
      detection.record.observation_stage = output::SbirsObservationStage::kNarrowFieldTrack;
      detection.record.detected = true;
      detection.attribution.detection_id = detection.record.detection_id;
      detection.attribution.target_id = target.target_id;
      detection.attribution.target_name = target.target_name;
      detection.attribution.estimated_range_m = static_cast<float>(range_m);
      detection.attribution.used_truth_assist = true;
      result.detections.push_back(detection);
      continue;
    }

    if (!in_wfov || snr < policy.detection.wide_min_snr_linear) {
      target_states_[target.target_id] = SbirsTargetState::kUndetected;
      continue;
    }

    Candidate candidate;
    candidate.target = &target;
    candidate.azimuth_deg = azimuth_deg;
    candidate.elevation_deg = elevation_deg;
    candidate.range_m = range_m;  // 真值距离：调度优先级用（design 2.6）
    // 2.10 WFOV 带误差位置：施加 5 类物理误差（高斯随机 + 折射 + 滞后）。
    // 目标角速度由 velocity_ecef_m_per_s 推导；未提供时按 0 处理（动态滞后项为 0）。
    float omega_deg_per_sec = 0.0f;
    if (target.has_velocity_ecef_m_per_s) {
      // 当前无卫星速度输入，相对速度按目标速度处理（design 假设）。
      omega_deg_per_sec = foundation::ComputeRelativeAngularRateDegPerSec(los, target.velocity_ecef_m_per_s);
    }
    const foundation::SbirsErrorBearing bearing = foundation::ApplyAngularErrorModel(
        policy.error_model, &random_source_, azimuth_deg, elevation_deg, range_m,
        /*target_angular_rate_deg_per_sec=*/omega_deg_per_sec);
    candidate.measured_azimuth_deg = bearing.azimuth_deg;
    candidate.measured_elevation_deg = bearing.elevation_deg;
    candidate.measured_range_m = bearing.range_m;
    // cue 延迟外推：narrow_cue_latency_s 期间目标继续运动，真值 az/el 需按延迟后位置重算。
    const float cue_latency_s = mission.narrow_cue_latency_s;
    if (cue_latency_s > 0.0f && target.has_velocity_ecef_m_per_s) {
      session::SbirsVector3M predicted_position;
      predicted_position.x = target.position_ecef_m.x + target.velocity_ecef_m_per_s.x * cue_latency_s;
      predicted_position.y = target.position_ecef_m.y + target.velocity_ecef_m_per_s.y * cue_latency_s;
      predicted_position.z = target.position_ecef_m.z + target.velocity_ecef_m_per_s.z * cue_latency_s;
      const session::SbirsVector3M predicted_los =
          foundation::Subtract(predicted_position, input.satellite_position_ecef_m);
      candidate.predicted_azimuth_deg = foundation::ComputeAzimuthDeg(predicted_los);
      candidate.predicted_elevation_deg = foundation::ComputeElevationDeg(predicted_los);
    } else {
      candidate.predicted_azimuth_deg = azimuth_deg;
      candidate.predicted_elevation_deg = elevation_deg;
    }
    candidate.snr = snr;
    candidates.push_back(candidate);
    target_states_[target.target_id] = SbirsTargetState::kWideCandidate;
  }

  if (!has_locked_target_ && !candidates.empty()) {
    std::sort(candidates.begin(), candidates.end(), [](const Candidate& lhs, const Candidate& rhs) {
      if (lhs.snr != rhs.snr) {
        return lhs.snr > rhs.snr;
      }
      if (lhs.range_m != rhs.range_m) {
        return lhs.range_m < rhs.range_m;
      }
      return lhs.target->target_id < rhs.target->target_id;
    });
    Candidate& selected = candidates.front();
    target_states_[selected.target->target_id] = SbirsTargetState::kAwaitingNfovAcquisition;
    const float cue_az = selected.measured_azimuth_deg + mission.narrow_pointing_settle_error_deg;
    const float cue_el = selected.measured_elevation_deg;
    // 捕获判据用延迟外推后的真值 az/el（predicted_*）；无速度时等同当前帧真值，保持旧行为。
    const bool in_nfov =
        InRectangularFov(selected.predicted_azimuth_deg, selected.predicted_elevation_deg, cue_az,
                         cue_el, mission.narrow_field_fov_az_deg, mission.narrow_field_fov_el_deg);
    const bool captured = in_nfov && selected.snr >= policy.detection.narrow_min_snr_linear;
    if (captured) {
      has_locked_target_ = true;
      locked_target_id_ = selected.target->target_id;
      target_states_[selected.target->target_id] = SbirsTargetState::kTruthAssistedTracking;
      SbirsPipelineDetection detection;
      detection.record.detection_id = next_detection_id_++;
      detection.record.azimuth_deg = selected.azimuth_deg;
      detection.record.elevation_deg = selected.elevation_deg;
      detection.record.infrared_snr_linear = static_cast<float>(selected.snr);
      detection.record.observation_stage = output::SbirsObservationStage::kNarrowFieldAcquisition;
      detection.record.detected = true;
      detection.attribution.detection_id = detection.record.detection_id;
      detection.attribution.target_id = selected.target->target_id;
      detection.attribution.target_name = selected.target->target_name;
      detection.attribution.estimated_range_m = static_cast<float>(selected.range_m);
      detection.attribution.used_truth_assist = true;
      result.detections.push_back(detection);
    } else {
      target_states_[selected.target->target_id] = SbirsTargetState::kWideCandidate;
      // 捕获失败：产出 detected=false 的诊断 attribution（record 不进 raw output，
      // 仅 attribution 进 result.detection_attributions 供调试/lifecycle 消费）。
      SbirsPipelineDetection detection;
      detection.record.detection_id = next_detection_id_++;
      detection.record.azimuth_deg = selected.azimuth_deg;
      detection.record.elevation_deg = selected.elevation_deg;
      detection.record.infrared_snr_linear = static_cast<float>(selected.snr);
      detection.record.observation_stage = output::SbirsObservationStage::kNarrowFieldAcquisition;
      detection.record.detected = false;
      detection.attribution.detection_id = detection.record.detection_id;
      detection.attribution.target_id = selected.target->target_id;
      detection.attribution.target_name = selected.target->target_name;
      detection.attribution.estimated_range_m = static_cast<float>(selected.range_m);
      detection.attribution.used_truth_assist = false;
      detection.attribution.capture_failure_reason =
          attribution::SbirsCaptureFailureReason::kNfovAcquisitionFailed;
      result.detections.push_back(detection);
    }
  }

  for (const Candidate& candidate : candidates) {
    if (has_locked_target_ && locked_target_id_ == candidate.target->target_id) {
      continue;
    }
    SbirsPipelineDetection detection;
    detection.record.detection_id = next_detection_id_++;
    detection.record.azimuth_deg = candidate.measured_azimuth_deg;
    detection.record.elevation_deg = candidate.measured_elevation_deg;
    detection.record.infrared_snr_linear = static_cast<float>(candidate.snr);
    detection.record.observation_stage = output::SbirsObservationStage::kWideFieldSearch;
    detection.record.detected = true;
    detection.attribution.detection_id = detection.record.detection_id;
    detection.attribution.target_id = candidate.target->target_id;
    detection.attribution.target_name = candidate.target->target_name;
    detection.attribution.estimated_range_m = static_cast<float>(candidate.range_m);
    detection.attribution.used_truth_assist = false;
    // 进入 WFOV 候选但未被调度器选中（资源被占用或排序靠后）：标记为调度跳过。
    if (has_locked_target_) {
      detection.attribution.capture_failure_reason =
          attribution::SbirsCaptureFailureReason::kSchedulerSkipped;
    }
    result.detections.push_back(detection);
  }

  return result;
}

SbirsPipelineSnapshot SbirsPipeline::CaptureRuntimeState() const {
  SbirsPipelineSnapshot snapshot;
  snapshot.scan_azimuth_deg = scan_azimuth_deg_;
  snapshot.next_detection_id = next_detection_id_;
  snapshot.target_states = target_states_;
  snapshot.has_locked_target = has_locked_target_;
  snapshot.locked_target_id = locked_target_id_;
  snapshot.random_state = random_source_.Capture();
  return snapshot;
}

bool SbirsPipeline::RestoreRuntimeState(const SbirsPipelineSnapshot& snapshot) {
  scan_azimuth_deg_ = snapshot.scan_azimuth_deg;
  next_detection_id_ = snapshot.next_detection_id;
  target_states_ = snapshot.target_states;
  has_locked_target_ = snapshot.has_locked_target;
  locked_target_id_ = snapshot.locked_target_id;
  random_source_.Restore(snapshot.random_state);
  return true;
}

}  // namespace pipeline
}  // namespace sbirs_sensor
