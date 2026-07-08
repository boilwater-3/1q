#include "sbirs_sensor/pipeline/SbirsPipeline.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Cholesky>

#include "sbirs_sensor/environment/SbirsEnvironmentModel.h"
#include "sbirs_sensor/foundation/SbirsErrorModel.h"
#include "sbirs_sensor/foundation/SbirsGeometry.h"
#include "sbirs_sensor/foundation/SbirsNoiseModel.h"
#include "sbirs_sensor/foundation/SbirsRadiometry.h"

namespace sbirs_sensor {
namespace pipeline {
namespace {

const double kEarthRadiusM = 6371000.0;
constexpr float kSbirsNisChiSquare2Dof95 = 5.99f;

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

float ComputeNormalizedInnovationSquared(
    const tracking::SbirsKalmanUpdateResult& update_result) {
  const Eigen::LLT<tracking::SbirsMeasurementCovariance> llt(
      update_result.innovation_covariance);
  if (llt.info() != Eigen::Success) {
    return std::numeric_limits<float>::infinity();
  }
  const tracking::SbirsMeasurementVector solved = llt.solve(update_result.innovation);
  return update_result.innovation.dot(solved);
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
    nis_gate_exceeded_counts_.clear();
    imm_snapshots_.clear();
    imm_initialized_ = false;
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
      filter_states_.erase(target.target_id);  // 释放滤波状态
      nis_gate_exceeded_counts_.erase(target.target_id);
      imm_snapshots_.erase(target.target_id);
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
        has_locked_target_ && locked_target_id_ == target.target_id &&
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
        // 带误差角度测量（deg → rad）——IMM 和单 EKF 共用。
        const foundation::SbirsErrorBearing bearing = foundation::ApplyAngularErrorModel(
            policy.error_model, &random_source_, azimuth_deg, elevation_deg, range_m, omega_deg_per_sec_cached);
        tracking::SbirsMeasurementVector measurement_rad;
        const float deg2rad = 0.0174532925f;
        measurement_rad << bearing.azimuth_deg * deg2rad, bearing.elevation_deg * deg2rad;
        const tracking::SbirsMeasurementCovariance R = tracking::BuildMeasurementCovariance(
            policy.error_model, range_m, elevation_deg, omega_deg_per_sec_cached);

        if (policy.tracking.enable_imm_tracking) {
          // === IMM 滤波测量跟踪 ===
          if (!imm_initialized_) {
            InitializeImmComponents(policy.tracking);
          }
          // 从快照恢复模型状态（replay/restore 后首次使用）
          auto imm_it = imm_snapshots_.find(target.target_id);
          if (imm_it != imm_snapshots_.end() && !imm_it->second.model_states.empty()) {
            imm_filter_->SetModelStates(imm_it->second.model_states);
            imm_snapshots_.erase(imm_it);
          }
          // 更新所有子模型的卫星位置
          for (auto& meas : imm_measurement_models_) {
            meas->SetSatellitePosition(input.satellite_position_ecef_m);
          }
          // IMM predict-update（动态 R 共用）
          imm_filter_->Process(measurement_rad, input.dt_sec, R);
          const tracking::SbirsGaussianState combined = imm_filter_->GetCombinedState();
          filter_states_[target.target_id] = combined;

          // 持久化 IMM 状态（供 snapshot/replay）
          tracking::SbirsImmSnapshot imm_snap;
          imm_snap.model_states = imm_filter_->GetModelStates();
          imm_snap.model_weights = imm_filter_->GetModelWeights();
          imm_snapshots_[target.target_id] = imm_snap;

          // NIS：取各模型最大值（所有模型均超门限 → 即使最宽松模型也无法解释当前测量）
          has_estimation_nis = true;
          estimation_nis = 0.0f;
          estimation_nis_gate_exceeded = false;
          const auto& imm_results = imm_filter_->GetModelUpdateResults();
          for (std::size_t m = 0U; m < imm_results.size(); ++m) {
            const float model_nis = ComputeNormalizedInnovationSquared(imm_results[m]);
            if (model_nis > estimation_nis) estimation_nis = model_nis;
            if (model_nis > kSbirsNisChiSquare2Dof95) estimation_nis_gate_exceeded = true;
          }
          if (policy.tracking.nis_gate_loss_cycles > 0U && estimation_nis_gate_exceeded) {
            const unsigned int exceeded_count = ++nis_gate_exceeded_counts_[target.target_id];
            lost_due_to_estimation_nis = exceeded_count >= policy.tracking.nis_gate_loss_cycles;
          } else {
            nis_gate_exceeded_counts_[target.target_id] = 0U;
          }
          // 滤波估计 → LOS → az/el
          const session::SbirsVector3M estimated_position{combined.mean(0), combined.mean(2),
                                                          combined.mean(4)};
          const session::SbirsVector3M est_los =
              foundation::Subtract(estimated_position, input.satellite_position_ecef_m);
          output_azimuth_deg = foundation::ComputeAzimuthDeg(est_los);
          output_elevation_deg = foundation::ComputeElevationDeg(est_los);
        } else {
          // === 单 EKF 滤波测量跟踪（现有路径） ===
          angle_measurement_model_.SetSatellitePosition(input.satellite_position_ecef_m);
          const tracking::SbirsEkfPredictor ekf_predictor(
              &cv_transition_model_, tracking::SbirsEkfPredictorConfig{policy.tracking.process_noise_diff_coeff});
          const tracking::SbirsGaussianState predicted =
              ekf_predictor.Predict(filter_states_[target.target_id], input.dt_sec);
          const tracking::SbirsEkfUpdater ekf_updater(&angle_measurement_model_);
          const tracking::SbirsKalmanUpdateResult update_result =
              ekf_updater.Update(predicted, measurement_rad, R);
          filter_states_[target.target_id] = update_result.posterior;
          has_estimation_nis = true;
          estimation_nis = ComputeNormalizedInnovationSquared(update_result);
          estimation_nis_gate_exceeded = estimation_nis > kSbirsNisChiSquare2Dof95;
          if (policy.tracking.nis_gate_loss_cycles > 0U && estimation_nis_gate_exceeded) {
            const unsigned int exceeded_count = ++nis_gate_exceeded_counts_[target.target_id];
            lost_due_to_estimation_nis = exceeded_count >= policy.tracking.nis_gate_loss_cycles;
          } else {
            nis_gate_exceeded_counts_[target.target_id] = 0U;
          }
          const session::SbirsVector3M estimated_position{update_result.posterior.mean(0),
                                                          update_result.posterior.mean(2),
                                                          update_result.posterior.mean(4)};
          const session::SbirsVector3M est_los =
              foundation::Subtract(estimated_position, input.satellite_position_ecef_m);
          output_azimuth_deg = foundation::ComputeAzimuthDeg(est_los);
          output_elevation_deg = foundation::ComputeElevationDeg(est_los);
        }
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
      detection.attribution.has_estimation_nis = has_estimation_nis;
      detection.attribution.estimation_nis = estimation_nis;
      detection.attribution.estimation_nis_gate_exceeded = estimation_nis_gate_exceeded;
      if (lost_due_to_estimation_nis) {
        detection.attribution.capture_failure_reason =
            attribution::SbirsCaptureFailureReason::kEstimationNisGateLost;
        target_states_[target.target_id] = SbirsTargetState::kWideCandidate;
        has_locked_target_ = false;
        locked_target_id_ = 0U;
        filter_states_.erase(target.target_id);
        nis_gate_exceeded_counts_.erase(target.target_id);
        imm_snapshots_.erase(target.target_id);
      }
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
      const bool use_estimated = policy.tracking.enable_estimated_tracking;
      target_states_[selected.target->target_id] =
          use_estimated ? SbirsTargetState::kEstimatedTracking : SbirsTargetState::kTruthAssistedTracking;
      if (use_estimated) {
        // 滤波初始化（方案 A）：状态均值用真值 ECEF 位置 + 速度（无速度时置 0）。
        // 状态布局为 CV 交错 [x,vx,y,vy,z,vz]（与 common KalmanPredictor 一致）。
        // 协方差 P0 由配置的 position/velocity 1-σ 构造为对角阵。
        tracking::SbirsGaussianState initial_state;
        initial_state.mean(0) = static_cast<float>(selected.target->position_ecef_m.x);  // x
        initial_state.mean(2) = static_cast<float>(selected.target->position_ecef_m.y);  // y
        initial_state.mean(4) = static_cast<float>(selected.target->position_ecef_m.z);  // z
        if (selected.target->has_velocity_ecef_m_per_s) {
          initial_state.mean(1) = static_cast<float>(selected.target->velocity_ecef_m_per_s.x);  // vx
          initial_state.mean(3) = static_cast<float>(selected.target->velocity_ecef_m_per_s.y);  // vy
          initial_state.mean(5) = static_cast<float>(selected.target->velocity_ecef_m_per_s.z);  // vz
        }
        const float pos_var = policy.tracking.initial_position_std_m *
                              policy.tracking.initial_position_std_m;
        const float vel_var = policy.tracking.initial_velocity_std_m_per_s *
                              policy.tracking.initial_velocity_std_m_per_s;
        initial_state.covariance = tracking::SbirsStateCovariance::Zero();
        // 交错布局：偶数索引位置（0,2,4），奇数索引速度（1,3,5）。
        initial_state.covariance(0, 0) = pos_var;
        initial_state.covariance(1, 1) = vel_var;
        initial_state.covariance(2, 2) = pos_var;
        initial_state.covariance(3, 3) = vel_var;
        initial_state.covariance(4, 4) = pos_var;
        initial_state.covariance(5, 5) = vel_var;
        filter_states_[selected.target->target_id] = initial_state;
        nis_gate_exceeded_counts_[selected.target->target_id] = 0U;
        if (policy.tracking.enable_imm_tracking) {
          if (!imm_initialized_) {
            InitializeImmComponents(policy.tracking);
          }
          const int num_models =
              static_cast<int>(imm_filter_->GetModelStates().size());
          const float init_weight = 1.0f / static_cast<float>(num_models);
          std::vector<tracking::SbirsImmModelState> init_states;
          for (int i = 0; i < num_models; ++i) {
            init_states.push_back({initial_state, init_weight});
          }
          imm_filter_->SetModelStates(init_states);
        }
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

void SbirsPipeline::InitializeImmComponents(const config::SbirsTrackingConfig& tracking) {
  imm_predictors_owned_.clear();
  imm_predictors_.clear();
  imm_measurement_models_.clear();
  imm_updaters_owned_.clear();
  imm_updaters_.clear();
  imm_filter_.reset();

  const std::vector<float> q_values =
      tracking.imm_model_noise_diff_coeffs.empty()
          ? std::vector<float>{1.0f, 100.0f}
          : tracking.imm_model_noise_diff_coeffs;
  const int num_models = static_cast<int>(q_values.size());

  for (int i = 0; i < num_models; ++i) {
    auto meas = std::make_unique<tracking::SbirsAngleMeasurementModel>();
    imm_measurement_models_.push_back(std::move(meas));

    tracking::SbirsEkfPredictorConfig pred_cfg;
    pred_cfg.noise_diff_coeff = q_values[static_cast<std::size_t>(i)];
    auto pred =
        std::make_unique<tracking::SbirsEkfPredictor>(&cv_transition_model_, pred_cfg);
    imm_predictors_.push_back(pred.get());
    imm_predictors_owned_.push_back(std::move(pred));

    auto upd =
        std::make_unique<tracking::SbirsEkfUpdater>(imm_measurement_models_.back().get());
    imm_updaters_.push_back(upd.get());
    imm_updaters_owned_.push_back(std::move(upd));
  }

  tracking::SbirsImmConfig imm_cfg;
  imm_cfg.transition_probability.resize(num_models, num_models);
  for (int i = 0; i < num_models; ++i) {
    for (int j = 0; j < num_models; ++j) {
      imm_cfg.transition_probability(i, j) =
          (i == j) ? 0.95f : 0.05f / static_cast<float>(num_models - 1);
    }
  }
  imm_cfg.initial_weights.setConstant(num_models, 1.0f / static_cast<float>(num_models));

  imm_filter_ = std::make_unique<tracking::SbirsImmFilter>(
      imm_cfg, imm_predictors_, imm_updaters_);
  imm_initialized_ = true;
}

SbirsPipelineSnapshot SbirsPipeline::CaptureRuntimeState() const {
  SbirsPipelineSnapshot snapshot;
  snapshot.scan_azimuth_deg = scan_azimuth_deg_;
  snapshot.next_detection_id = next_detection_id_;
  snapshot.target_states = target_states_;
  snapshot.has_locked_target = has_locked_target_;
  snapshot.locked_target_id = locked_target_id_;
  snapshot.random_state = random_source_.Capture();
  snapshot.filter_states = filter_states_;
  snapshot.nis_gate_exceeded_counts = nis_gate_exceeded_counts_;
  snapshot.imm_active = imm_initialized_;
  snapshot.imm_snapshots = imm_snapshots_;
  return snapshot;
}

bool SbirsPipeline::RestoreRuntimeState(const SbirsPipelineSnapshot& snapshot) {
  scan_azimuth_deg_ = snapshot.scan_azimuth_deg;
  next_detection_id_ = snapshot.next_detection_id;
  target_states_ = snapshot.target_states;
  has_locked_target_ = snapshot.has_locked_target;
  locked_target_id_ = snapshot.locked_target_id;
  random_source_.Restore(snapshot.random_state);
  filter_states_ = snapshot.filter_states;
  nis_gate_exceeded_counts_ = snapshot.nis_gate_exceeded_counts;
  imm_snapshots_ = snapshot.imm_snapshots;
  imm_initialized_ = false;  // 强制下周期重新初始化 IMM 组件，从 imm_snapshots_ 恢复模型状态
  return true;
}

}  // namespace pipeline
}  // namespace sbirs_sensor
