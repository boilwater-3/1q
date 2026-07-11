#include "sbirs_sensor/pipeline/SbirsPipeline.h"

#include <algorithm>
#include <cmath>
#include "sbirs_sensor/environment/SbirsEnvironmentModel.h"
#include "sbirs_sensor/foundation/SbirsErrorModel.h"
#include "sbirs_sensor/foundation/SbirsGeometry.h"
#include "sbirs_sensor/foundation/SbirsNoiseModel.h"
#include "sbirs_sensor/foundation/SbirsRadiometry.h"
#include "sbirs_sensor/pipeline/SbirsNfovAcquisition.h"

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

}  // namespace

SbirsPipeline::SbirsPipeline(const config::SbirsInternalExecutionConfig& config)
    : config_(config),
      scan_azimuth_deg_(config.session.mission.scan_start_az_deg),
      nfov_scheduler_(config.session.policy.scheduler.max_concurrent_nfov_locks),
      random_source_(config.session.policy.error_model.random_seed) {}

void SbirsPipeline::ApplyConfig(const config::SbirsInternalExecutionConfig& config) {
  config_ = config;
  // 配置变更后重置随机源种子，保证 runtime patch 后的 replay 可复现。
  random_source_ = foundation::SbirsRandomSource(config.session.policy.error_model.random_seed);
  // 同步调度器通道上限；运行期 patch 改变通道数后清空既有分配，避免越界。
  nfov_scheduler_ = SbirsNfovScheduler(config.session.policy.scheduler.max_concurrent_nfov_locks);
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
    tracking_coordinator_.ClearForStandby();
    nfov_scheduler_.Clear();
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
  std::vector<SbirsCandidate> candidates;

  for (const session::SbirsSceneTarget& target : input.scene) {
    if (!target.active) {
      target_states_[target.target_id] = SbirsTargetState::kLost;
      nfov_scheduler_.Release(target.target_id);
      tracking_coordinator_.ReleaseTarget(target.target_id);
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

    // 目标角速度：用于动态滞后误差与 R 矩阵（design 2.10）。未提供速度时按 0 处理。
    float omega_deg_per_sec_cached = 0.0f;
    if (target.has_velocity_ecef_m_per_s) {
      omega_deg_per_sec_cached =
          foundation::ComputeRelativeAngularRateDegPerSec(los, target.velocity_ecef_m_per_s);
    }

    const bool in_wfov =
        InRectangularFov(azimuth_deg, elevation_deg, scan_azimuth_deg_, mission.scan_center_el_deg,
                         mission.wide_field_fov_az_deg, mission.wide_field_fov_el_deg);

    const SbirsTargetState state = target_states_[target.target_id];
    const bool is_locked =
        nfov_scheduler_.IsLocked(target.target_id) &&
        (state == SbirsTargetState::kTruthAssistedTracking ||
         state == SbirsTargetState::kEstimatedTracking);
    if (is_locked) {
      // 输出角度来源：真值辅助态用真值 az/el；估计跟踪态用 EKF 滤波估计（更平滑）。
      // SNR / range / 可探测性一律用真值几何（design 2.5 第 3 点：滤波发散不影响可探测性）。
      float output_azimuth_deg = azimuth_deg;
      float output_elevation_deg = elevation_deg;
      bool used_truth_assist = true;
      bool has_estimation_nis = false;
      float estimation_nis = 0.0f;
      bool estimation_nis_gate_exceeded = false;
      bool lost_due_to_estimation_nis = false;
      if (state == SbirsTargetState::kEstimatedTracking &&
          policy.tracking.enable_estimated_tracking) {
        used_truth_assist = false;
        const SbirsTrackingUpdateResult tracking_result = tracking_coordinator_.Update(
            target.target_id, policy, &random_source_, azimuth_deg, elevation_deg, range_m,
            omega_deg_per_sec_cached, input.dt_sec, input.satellite_position_ecef_m);
        output_azimuth_deg = tracking_result.output_azimuth_deg;
        output_elevation_deg = tracking_result.output_elevation_deg;
        has_estimation_nis = tracking_result.has_estimation_nis;
        estimation_nis = tracking_result.estimation_nis;
        estimation_nis_gate_exceeded = tracking_result.estimation_nis_gate_exceeded;
        lost_due_to_estimation_nis = tracking_result.lost_due_to_estimation_nis;
      }
      SbirsPipelineDetection detection;
      detection.record.detection_id = next_detection_id_++;
      detection.record.azimuth_deg = output_azimuth_deg;
      detection.record.elevation_deg = output_elevation_deg;
      detection.record.infrared_snr_linear = static_cast<float>(snr);
      detection.record.observation_stage = output::SbirsObservationStage::kNarrowFieldTrack;
      detection.record.detected = !lost_due_to_estimation_nis;
      detection.attribution.detection_id = detection.record.detection_id;
      detection.attribution.target_id = target.target_id;
      detection.attribution.target_name = target.target_name;
      detection.attribution.estimated_range_m = static_cast<float>(range_m);
      detection.attribution.used_truth_assist = used_truth_assist;
      detection.attribution.nfov_channel_id = nfov_scheduler_.ChannelOf(target.target_id);
      detection.attribution.has_estimation_nis = has_estimation_nis;
      detection.attribution.estimation_nis = estimation_nis;
      detection.attribution.estimation_nis_gate_exceeded = estimation_nis_gate_exceeded;
      if (lost_due_to_estimation_nis) {
        detection.attribution.capture_failure_reason =
            attribution::SbirsCaptureFailureReason::kEstimationNisGateLost;
        target_states_[target.target_id] = SbirsTargetState::kWideCandidate;
        nfov_scheduler_.Release(target.target_id);
        tracking_coordinator_.ReleaseTarget(target.target_id);
      }
      result.detections.push_back(detection);
      continue;
    }

    if (!in_wfov || snr < policy.detection.wide_min_snr_linear) {
      target_states_[target.target_id] = SbirsTargetState::kUndetected;
      continue;
    }

    SbirsCandidate candidate;
    candidate.target = &target;
    candidate.azimuth_deg = azimuth_deg;
    candidate.elevation_deg = elevation_deg;
    candidate.range_m = range_m;  // 真值距离：调度优先级用（design 2.6）
    // 2.10 WFOV 带误差位置：施加 5 类物理误差（高斯随机 + 折射 + 滞后）。
    // 复用上方已算的目标角速度 omega_deg_per_sec_cached。
    const foundation::SbirsErrorBearing bearing = foundation::ApplyAngularErrorModel(
        policy.error_model, &random_source_, azimuth_deg, elevation_deg, range_m,
        /*target_angular_rate_deg_per_sec=*/omega_deg_per_sec_cached);
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

  // 多通道调度：在并发上限内按 design 2.6 优先级选取候选进入首次捕获。
  const std::vector<const SbirsCandidate*> selected_candidates =
      nfov_scheduler_.SelectForAcquisition(candidates);
  for (const SbirsCandidate* selected_ptr : selected_candidates) {
    const SbirsCandidate& selected = *selected_ptr;
    target_states_[selected.target->target_id] = SbirsTargetState::kAwaitingNfovAcquisition;
    // 捕获判据用延迟外推后的真值 az/el（predicted_*）；无速度时等同当前帧真值，保持旧行为。
    SbirsNfovAcquisitionRequest acquisition_request;
    acquisition_request.predicted_azimuth_deg = selected.predicted_azimuth_deg;
    acquisition_request.predicted_elevation_deg = selected.predicted_elevation_deg;
    acquisition_request.measured_azimuth_deg = selected.measured_azimuth_deg;
    acquisition_request.measured_elevation_deg = selected.measured_elevation_deg;
    acquisition_request.pointing_settle_error_deg = mission.narrow_pointing_settle_error_deg;
    acquisition_request.field_of_view_azimuth_deg = mission.narrow_field_fov_az_deg;
    acquisition_request.field_of_view_elevation_deg = mission.narrow_field_fov_el_deg;
    acquisition_request.snr = selected.snr;
    acquisition_request.minimum_snr_linear = policy.detection.narrow_min_snr_linear;
    const bool captured = IsNfovAcquisitionEligible(acquisition_request);
    if (captured) {
      const int channel_id = nfov_scheduler_.Acquire(selected.target->target_id);
      const bool use_estimated = policy.tracking.enable_estimated_tracking;
      target_states_[selected.target->target_id] =
          use_estimated ? SbirsTargetState::kEstimatedTracking : SbirsTargetState::kTruthAssistedTracking;
      if (use_estimated) {
        tracking_coordinator_.InitializeTarget(selected.target->target_id, *selected.target,
                                               policy.tracking);
      }
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
      detection.attribution.used_truth_assist = !use_estimated;
      detection.attribution.nfov_channel_id = channel_id;
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

  // 通道已满（无并发余量）时，未被选中的 WFOV 候选标记为调度跳过。
  const bool resources_full =
      static_cast<int>(nfov_scheduler_.LockedCount()) >= nfov_scheduler_.max_locks();
  for (const SbirsCandidate& candidate : candidates) {
    if (nfov_scheduler_.IsLocked(candidate.target->target_id)) {
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
    if (resources_full) {
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
  snapshot.nfov_scheduler = nfov_scheduler_.Capture();
  snapshot.random_state = random_source_.Capture();
  const SbirsTrackingRuntimeState tracking_state = tracking_coordinator_.CaptureRuntimeState();
  snapshot.filter_states = tracking_state.filter_states;
  snapshot.nis_gate_exceeded_counts = tracking_state.nis_gate_exceeded_counts;
  snapshot.imm_active = tracking_state.imm_active;
  snapshot.imm_snapshots = tracking_state.imm_snapshots;
  return snapshot;
}

bool SbirsPipeline::RestoreRuntimeState(const SbirsPipelineSnapshot& snapshot) {
  scan_azimuth_deg_ = snapshot.scan_azimuth_deg;
  next_detection_id_ = snapshot.next_detection_id;
  target_states_ = snapshot.target_states;
  nfov_scheduler_.Restore(snapshot.nfov_scheduler);
  random_source_.Restore(snapshot.random_state);
  SbirsTrackingRuntimeState tracking_state;
  tracking_state.filter_states = snapshot.filter_states;
  tracking_state.nis_gate_exceeded_counts = snapshot.nis_gate_exceeded_counts;
  tracking_state.imm_active = snapshot.imm_active;
  tracking_state.imm_snapshots = snapshot.imm_snapshots;
  tracking_coordinator_.RestoreRuntimeState(tracking_state);
  return true;
}

}  // namespace pipeline
}  // namespace sbirs_sensor
