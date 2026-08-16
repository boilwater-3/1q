/**
 * @file RirController.cpp
 * @brief 远程识别雷达自持链路控制器实现（阶段 2-S S2）。
 */

#include "remote_identification_radar/runtime/RirController.h"

#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <random>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/logging/ProjectLog.h"
#include "common/numerics/Constants.h"
#include "remote_identification_radar/dwell/RirBeamControl.h"
#include "remote_identification_radar/dwell/RirMeasurementErrorModel.h"
#include "remote_identification_radar/recognition/RecognitionObservationBuilder.h"

namespace remote_identification_radar {
namespace runtime {

namespace {

constexpr float kSnrFallbackGateDb = 6.0f;
constexpr float kCovarianceFloorM2 = 1.0e-6f;

internal::RirSwerlingModel ToInternalSwerling(session::RirSwerlingType type) {
  switch (type) {
    case session::RirSwerlingType::kSwerling1:
      return internal::RirSwerlingModel::kSwerling1;
    case session::RirSwerlingType::kSwerling2:
      return internal::RirSwerlingModel::kSwerling2;
    case session::RirSwerlingType::kSwerling3:
      return internal::RirSwerlingModel::kSwerling3;
    case session::RirSwerlingType::kSwerling4:
      return internal::RirSwerlingModel::kSwerling4;
    case session::RirSwerlingType::kSwerling0:
    default:
      return internal::RirSwerlingModel::kSwerling0;
  }
}

Eigen::Vector3f PositionOf(const session::RirSceneTarget& target) {
  const Eigen::Vector3f position(target.position_x, target.position_y, target.position_z);
  if (position.squaredNorm() > 0.0f) {
    return position;
  }
  return Eigen::Vector3f(target.range_m, 0.0f, 0.0f);
}

float DbToLinear(float db_value) { return std::pow(10.0f, db_value / 10.0f); }

bool IsValidIdentity(const oneq::electromagnetics::RfEmissionIdentity& identity) {
  return identity.platform_id != 0U && identity.equipment_id != 0U && identity.emission_id != 0U;
}

bool TryBuildOwnWaveform(const session::RirCycleInput& input,
                         const config::hardware::RirTransmitterConfig& transmitter, float dwell_sec,
                         std::uint32_t random_seed,
                         oneq::electromagnetics::RfWaveformSchedule* waveform) {
  if (waveform == nullptr || transmitter.prf_hz <= 0.0f) {
    return false;
  }
  const double radiated_peak_power_w =
      static_cast<double>(transmitter.peak_power_w) *
      std::pow(10.0, -static_cast<double>(transmitter.transmit_loss_db) / 10.0);
  const double pri_s = 1.0 / static_cast<double>(transmitter.prf_hz);
  const std::uint32_t pulse_count = static_cast<std::uint32_t>(std::max(1.0, dwell_sec / pri_s));
  return oneq::electromagnetics::TryCreateRfPulseTrainWaveform(
      input.sim_time_sec, static_cast<double>(transmitter.frequency_hz),
      static_cast<double>(transmitter.bandwidth_hz), radiated_peak_power_w,
      static_cast<double>(transmitter.pulse_width_s), pri_s, pulse_count, 0.0, random_seed,
      static_cast<std::uint64_t>(input.input_cycle_index), waveform);
}

}  // namespace

void RirController::SetHardware(const config::RirHardwareConfig& hardware) {
  hardware_ = hardware;
  if (detector_ == nullptr) {
    detector_.reset(new dwell::RirSignalDetector(MakeDetectorConfig()));
  } else {
    detector_->UpdateConfig(MakeDetectorConfig());
  }
  detector_->SetRandomSeed(detection_random_seed_);
  measurement_rng_.seed(detection_random_seed_);
}

dwell::RirDetectorConfig RirController::MakeDetectorConfig() const {
  dwell::RirDetectorConfig config;
  config.transmitter = hardware_.transmitter;
  config.antenna = hardware_.antenna;
  config.receiver = hardware_.receiver;
  config.detection_policy.cfar_pfa = policy_.detection.cfar_pfa;
  config.detection_policy.min_snr_db = policy_.detection.min_snr_db;
  config.min_detection_margin_db = policy_.detection.min_detection_margin_db;
  config.pulse_count = policy_.detection.pulse_count;
  return config;
}

void RirController::UpdateRuntime(const config::RirMissionConfig& mission,
                                  const config::RirPolicyConfig& policy) {
  mission_ = mission;
  work_mode_ = mission.work_mode;
  policy_ = policy;
  detection_random_seed_ = policy_.detection.random_seed;

  recognition::RirTracker::Options options;
  options.min_confirmed_hits = policy_.recognition.min_confirmed_hits;
  options.accumulation_window_sec = policy_.recognition.accumulation_window_sec;
  options.min_observation_count = policy_.recognition.min_observation_count;
  options.acceptance_score = policy_.recognition.acceptance_score;
  options.minimum_margin = policy_.recognition.minimum_margin;
  options.result_hold_sec = policy_.recognition.result_hold_sec;
  options.max_range_m = mission_.max_range_m;
  tracker_.SetOptions(options);

  tracking::RirLifecycleConfig lifecycle_config;
  lifecycle_config.confirm_hits = policy_.lifecycle.confirm_hits;
  lifecycle_config.max_miss_before_lost = policy_.lifecycle.max_miss_before_lost;
  lifecycle_config.max_lost_cycles = policy_.lifecycle.max_lost_cycles;
  lifecycle_config.enable_imm_lifecycle = policy_.lifecycle.enable_imm_lifecycle;
  lifecycle_config.model_count_hint = policy_.lifecycle.model_count_hint;
  tracking::RirTrackFilterConfig filter_config;
  filter_config.process_noise_diff_coeff = policy_.tracking.kalman_noise_diff_coeff;
  filter_config.default_measurement_noise_std = policy_.tracking.kalman_measurement_noise_std;
  lifecycle_->UpdateConfig(lifecycle_config, filter_config);

  tracking::RirAssociationConfig association_config;
  const float sigma = std::max(0.0f, policy_.association.distance_gate_sigma);
  association_config.gate_threshold = sigma * sigma;
  association_config.kalman_noise_diff_coeff = policy_.tracking.kalman_noise_diff_coeff;
  association_config.default_measurement_noise_std = policy_.tracking.kalman_measurement_noise_std;
  associator_.UpdateConfig(association_config);

  if (detector_ == nullptr) {
    detector_.reset(new dwell::RirSignalDetector(MakeDetectorConfig()));
  } else {
    detector_->UpdateConfig(MakeDetectorConfig());
  }
  detector_->SetRandomSeed(detection_random_seed_);
  measurement_rng_.seed(detection_random_seed_);

  // 数据库按需加载：路径变化且已启用时重新加载；失败保持原库（识别降级 kDisabled）。
  if (policy_.recognition.enabled && !policy_.recognition.database_path.empty() &&
      policy_.recognition.database_path != database_path_) {
    std::unique_ptr<recognition::RirFeatureDatabase> candidate(
        new recognition::RirFeatureDatabase());
    std::string error;
    if (recognition::RirFeatureDatabase::Load(policy_.recognition.database_path, candidate.get(),
                                              &error)) {
      database_ = std::move(candidate);
      database_path_ = policy_.recognition.database_path;
      tracker_.SetActiveDatabaseVersion(database_->version());
    } else {
      // 中译：识别特征数据库加载失败，本次更新不生效，识别链路保持当前库或 kDisabled 降级。
      // 标识：RirController::UpdateRuntime，数据库路径/格式错误时触发；不影响检测跟踪链路。
      PROJECT_LOG_ERROR("[RirController] recognition database load failed: {}", error);
    }
  }
}

void RirController::ResolveEnvironment(const session::RirCycleInput& input,
                                       float* propagation_loss_db, float* clutter_power_w) const {
  *propagation_loss_db = 0.0f;
  *clutter_power_w = 0.0f;
  if (!input.environment_snapshot.has_environment_data) {
    return;
  }
  internal::RirEnvironmentSceneState scene_state;
  scene_state.vegetation_scatter_physics = input.environment_snapshot.vegetation_scatter_physics;
  const internal::RirPropagationResult propagation = propagation_model_.Evaluate(scene_state);
  *propagation_loss_db =
      propagation.propagation_loss_db + input.environment_snapshot.weather_attenuation_db;
  *clutter_power_w = DbToLinear(propagation.clutter_power_db);
}

void RirController::ComputeLookAngles(const session::RirSceneTarget& target, float* look_az_deg,
                                      float* look_el_deg, float* slant_range_m) {
  const Eigen::Vector3f position = PositionOf(target);
  const float range_hypot = std::sqrt(position.x() * position.x() + position.y() * position.y());
  *look_az_deg = oneq::common::numerics::RadToDeg(std::atan2(position.y(), position.x()));
  *look_el_deg = oneq::common::numerics::RadToDeg(std::atan2(position.z(), range_hypot));
  *slant_range_m = target.range_m > 0.0f ? target.range_m : position.norm();
}

tracking::RirMeasurementCovariance RirController::MakeCartesianMeasurementCovariance(
    const session::RirSceneTarget& target, float range_std_m, float angle_std_rad) {
  const Eigen::Vector3f position = PositionOf(target);
  const float range_m = std::max(position.norm(), 1.0f);
  const float range_hypot = std::sqrt(position.x() * position.x() + position.y() * position.y());
  const float angle_variance = angle_std_rad * angle_std_rad;

  Eigen::Matrix3f jacobian = Eigen::Matrix3f::Zero();
  if (range_hypot > 1.0f) {
    const float range_sq = range_m * range_m;
    jacobian(0, 0) = position.x() / range_m;
    jacobian(0, 1) = -position.y();
    jacobian(0, 2) = -position.x() * position.z() / (range_sq * range_hypot);
    jacobian(1, 0) = position.y() / range_m;
    jacobian(1, 1) = position.x();
    jacobian(1, 2) = -position.y() * position.z() / (range_sq * range_hypot);
    jacobian(2, 0) = position.z() / range_m;
    jacobian(2, 1) = 0.0f;
    jacobian(2, 2) = range_hypot / range_sq;
  } else {
    jacobian(0, 0) = position.x() / range_m;
    jacobian(0, 2) = position.x() / range_m;
    jacobian(1, 0) = position.y() / range_m;
    jacobian(1, 2) = position.y() / range_m;
    jacobian(2, 0) = position.z() / range_m;
    jacobian(2, 2) = position.z() / range_m;
  }

  Eigen::Vector3f variances(range_std_m * range_std_m, angle_variance, angle_variance);
  tracking::RirMeasurementCovariance covariance =
      jacobian * variances.asDiagonal() * jacobian.transpose();
  covariance.diagonal().array() += kCovarianceFloorM2;
  return covariance;
}

tracking::RirTrackMeasurement RirController::SampleMeasurementPosition(
    const tracking::RirTrackMeasurement& measurement) {
  tracking::RirTrackMeasurement sampled = measurement;
  Eigen::LLT<tracking::RirMeasurementCovariance> llt(measurement.measurement_covariance);
  if (llt.info() != Eigen::Success) {
    return sampled;
  }
  std::normal_distribution<float> normal(0.0f, 1.0f);
  Eigen::Vector3f noise(normal(measurement_rng_), normal(measurement_rng_),
                        normal(measurement_rng_));
  sampled.position += llt.matrixL() * noise;
  return sampled;
}

bool RirController::TryBuildMeasurement(const session::RirSceneTarget& target,
                                        std::size_t source_index, float propagation_loss_db,
                                        float clutter_power_w, const session::RirCycleInput& input,
                                        const config::RirAzimuthElevationDeg& dwell_center_deg,
                                        tracking::RirTrackMeasurement* measurement, float* snr_db) {
  if (measurement == nullptr || snr_db == nullptr) {
    return false;
  }
  float look_az_deg = 0.0f;
  float look_el_deg = 0.0f;
  float slant_range_m = 0.0f;
  ComputeLookAngles(target, &look_az_deg, &look_el_deg, &slant_range_m);

  const bool has_look_angles = slant_range_m > 0.0f;
  dwell::RirResolvedBeamState beam_state;
  beam_state.one_way_antenna_gain_db = hardware_.antenna.main_beam_gain_db;
  beam_state.effective_beamwidth_deg = dwell::RirResolveEffectiveBeamwidth(hardware_.antenna);
  if (hardware_.antenna.enable_directional_pattern && has_look_angles) {
    const float wavelength_m = static_cast<float>(oneq::common::numerics::kLightSpeed) /
                               hardware_.transmitter.frequency_hz;
    // 库内驻留调度器给定波束中心：目标离轴增益按（目标视线角 - 驻留中心）衰减
    // （与 AR 冻结指向链路同口径；enable_directional_pattern=false 时回退主瓣峰值）。
    beam_state = dwell::RirResolveBeamStateForPointing(hardware_.antenna, dwell_center_deg,
                                                       look_az_deg, look_el_deg, true,
                                                       wavelength_m);
  }

  dwell::RirTargetReturn target_return;
  target_return.rcs_m2 = target.rcs;
  target_return.range_m = slant_range_m;
  target_return.swerling_type = ToInternalSwerling(target.target_swerling_type);

  const bool has_environment = input.environment_snapshot.has_environment_data;
  const bool has_interference = !input.incident_links.empty();
  dwell::RirDetectionResult detection;
  bool resolved_cell = false;
  if ((has_environment || has_interference) && IsValidIdentity(input.own_emission_identity)) {
    oneq::electromagnetics::RfWaveformSchedule own_waveform;
    if (TryBuildOwnWaveform(input, hardware_.transmitter, mission_.recognition_dwell_sec,
                            detection_random_seed_, &own_waveform)) {
      dwell::RirDetectionCellConfig cell_config;
      cell_config.own_transmit_waveform = own_waveform;
      cell_config.receive_window_start_time_s = input.sim_time_sec;
      cell_config.receive_window_duration_s = mission_.recognition_dwell_sec;
      cell_config.matched_filter_bandwidth_hz = hardware_.transmitter.bandwidth_hz;
      cell_config.one_way_antenna_gain_dbi = beam_state.one_way_antenna_gain_db;
      cell_config.receiver_loss_db = hardware_.receiver.receive_loss_db;
      cell_config.receiver_noise_figure_db = hardware_.receiver.noise_figure_db;
      cell_config.signal_processing = hardware_.signal_processing;

      dwell::RirDetectionCellTarget cell_target;
      cell_target.range_m = slant_range_m;
      const Eigen::Vector3f position = PositionOf(target);
      const float range_norm = std::max(position.norm(), 1.0f);
      const float radial_velocity =
          (position.x() * target.velocity_x + position.y() * target.velocity_y +
           position.z() * target.velocity_z) /
          range_norm;
      cell_target.closing_radial_velocity_mps = -radial_velocity;
      cell_target.rcs_m2 = target.rcs;
      cell_target.two_way_additional_propagation_loss_db = propagation_loss_db;
      cell_target.effective_pulse_count =
          static_cast<std::uint32_t>(std::max(1, policy_.detection.pulse_count));

      dwell::RirDetectionCellResult cell;
      if (dwell::TryResolveRirDetectionCell(cell_config, cell_target, input.own_emission_identity,
                                            input.incident_links, clutter_power_w, &cell)) {
        detection = detector_->DetectResolvedCell(target_return, cell);
        resolved_cell = true;
      } else {
        // 中译：detection cell 求解失败，回退效能级检测路径。
        // 标识：数值/输入保护——RF 分解失败时不丢目标，按旧口径继续判决。
        PROJECT_LOG_WARN("[RirController] detection cell resolve failed for target id={}",
                         target.external_target_id);
      }
    }
  }

  if (!resolved_cell) {
    dwell::RirEnvironmentNoise env;
    env.propagation_loss_db = propagation_loss_db;
    env.clutter_noise_w = clutter_power_w;
    env.jam_noise_w = 0.0f;
    detection = detector_->Detect(target_return, env, beam_state.one_way_antenna_gain_db,
                                  std::max(1, policy_.detection.pulse_count));
  }

  *snr_db = detection.snr_db;
  const bool admitted = policy_.detection.gate_mode == config::RirDetectionGateMode::kSnrFallback
                            ? detection.snr_db >= kSnrFallbackGateDb
                            : detection.detected;
  if (!admitted) {
    return false;
  }

  const dwell::RirMeasurementErrorState measurement_error =
      dwell::RirMeasurementErrorModel::Compute(detection.snr_db, beam_state.effective_beamwidth_deg,
                                               hardware_.transmitter.bandwidth_hz);
  tracking::RirTrackMeasurement built;
  built.source_index = source_index;
  built.external_target_id = target.external_target_id;
  built.target_name = target.target_name;
  built.position = PositionOf(target);
  built.velocity = Eigen::Vector3f(target.velocity_x, target.velocity_y, target.velocity_z);
  built.rcs = target.rcs;
  built.measurement_covariance = MakeCartesianMeasurementCovariance(
      target, measurement_error.range_error_std_m, measurement_error.angle_error_std_rad);
  // 6 dB 回退模式保持旧识别门控口径：量测位置取真值，只携带误差协方差；
  // 检测器门控模式按量测误差模型采样位置（自持链路的量测扰动语义）。
  if (policy_.detection.gate_mode == config::RirDetectionGateMode::kSnrFallback) {
    *measurement = built;
  } else {
    *measurement = SampleMeasurementPosition(built);
  }
  return true;
}

recognition::RirObservationContext RirController::MakeObservationContext(
    const session::RirSceneTarget& target, float platform_altitude_m, float snr_db) const {
  recognition::RirObservationContext context;
  context.snr_db = snr_db;
  context.bandwidth_hz = hardware_.transmitter.bandwidth_hz;
  context.dwell_sec = mission_.recognition_dwell_sec;
  float look_az_deg = 0.0f;
  float look_el_deg = 0.0f;
  float slant_range_m = 0.0f;
  ComputeLookAngles(target, &look_az_deg, &look_el_deg, &slant_range_m);
  context.range_m = slant_range_m;
  context.look_az_deg = look_az_deg;
  context.look_el_deg = look_el_deg;
  context.platform_altitude_m = platform_altitude_m;
  context.minimum_aspect_coverage_deg = 0.0f;
  return context;
}

void RirController::RunCycle(const session::RirCycleInput& input,
                             session::RirOutputFrame* output_frame,
                             const config::RirAzimuthElevationDeg& dwell_center_deg) {
  const bool in_identify = work_mode_ == config::RirWorkMode::kIdentify;
  if (recognition_mode_active_ && !in_identify) {
    tracker_.ExitRecognitionMode();
  }
  recognition_mode_active_ = in_identify;
  sim_time_sec_ = input.sim_time_sec;

  latest_summary_ = session::RirRecognitionCycleSummary{};
  has_latest_summary_ = false;

  std::vector<tracking::RirTrackState> track_snapshots;
  if (in_identify) {
    float propagation_loss_db = 0.0f;
    float clutter_power_w = 0.0f;
    ResolveEnvironment(input, &propagation_loss_db, &clutter_power_w);

    std::unordered_map<std::uint64_t, const session::RirSceneTarget*> scene_by_id;
    std::vector<std::size_t> candidate_indices;
    for (std::size_t i = 0U; i < input.scene_targets.size(); ++i) {
      const session::RirSceneTarget& target = input.scene_targets[i];
      if (target.external_target_id == 0U) {
        continue;
      }
      scene_by_id[target.external_target_id] = &target;
      candidate_indices.push_back(i);
    }

    const std::vector<tracking::RirTrackState> prior_tracks = lifecycle_->BuildTrackSnapshots();
    std::unordered_map<std::uint64_t, std::uint32_t> recognition_rank_by_target_id;
    for (const tracking::RirTrackState& track : prior_tracks) {
      const session::RirRecognitionResult* result = tracker_.FindResult(track.association_key);
      const bool recognized =
          result != nullptr && (result->state == session::RirRecognitionState::kCategoryConfirmed ||
                                result->state == session::RirRecognitionState::kModelConfirmed);
      recognition_rank_by_target_id[track.external_target_id] = recognized ? 1U : 0U;
    }
    std::sort(
        candidate_indices.begin(), candidate_indices.end(),
        [&](std::size_t lhs_index, std::size_t rhs_index) {
          const session::RirSceneTarget& lhs = input.scene_targets[lhs_index];
          const session::RirSceneTarget& rhs = input.scene_targets[rhs_index];
          const std::uint32_t lhs_rank = recognition_rank_by_target_id[lhs.external_target_id];
          const std::uint32_t rhs_rank = recognition_rank_by_target_id[rhs.external_target_id];
          if (lhs_rank != rhs_rank) {
            return lhs_rank < rhs_rank;
          }
          const float lhs_range = lhs.range_m > 0.0f ? lhs.range_m : PositionOf(lhs).norm();
          const float rhs_range = rhs.range_m > 0.0f ? rhs.range_m : PositionOf(rhs).norm();
          return lhs_range < rhs_range;
        });

    std::vector<tracking::RirTrackMeasurement> measurements;
    std::unordered_map<std::uint64_t, float> snr_by_target_id;
    const float dwell_sec = mission_.recognition_dwell_sec;
    const std::uint32_t scheduled_count = static_cast<std::uint32_t>(candidate_indices.size());
    std::uint32_t executed_count = 0U;
    for (const std::size_t target_index : candidate_indices) {
      tracking::RirTrackMeasurement measurement;
      float snr_db = 0.0f;
      if (!TryBuildMeasurement(input.scene_targets[target_index], target_index, propagation_loss_db,
                               clutter_power_w, input, dwell_center_deg, &measurement, &snr_db)) {
        continue;
      }
      snr_by_target_id[measurement.external_target_id] = snr_db;
      measurements.push_back(measurement);
      ++executed_count;
    }

    const tracking::RirAssociationResult association = associator_.Associate(
        measurements, lifecycle_->BuildAssociationSeeds(), static_cast<float>(input.dt_sec));
    tracking::RirCycleContext cycle_context;
    cycle_context.cycle_index = input.input_cycle_index;
    cycle_context.batch_id = input.batch_id;
    cycle_context.dt_sec = static_cast<float>(input.dt_sec);
    lifecycle_->Update(cycle_context, association.measurements);

    track_snapshots = lifecycle_->BuildTrackSnapshots();

    std::unordered_map<std::uint64_t, recognition::RirTracker::TrackObservationInput>
        observations_by_key;
    for (const tracking::RirTrackState& track : track_snapshots) {
      const auto scene_found = scene_by_id.find(track.external_target_id);
      if (scene_found == scene_by_id.end() || scene_found->second == nullptr) {
        continue;
      }
      float snr_db = 0.0f;
      const auto snr_found = snr_by_target_id.find(track.external_target_id);
      if (snr_found != snr_by_target_id.end()) {
        snr_db = snr_found->second;
      }
      recognition::RirTracker::TrackObservationInput observation;
      observation.target = scene_found->second;
      observation.context =
          MakeObservationContext(*scene_found->second, input.platform_altitude_m, snr_db);
      observations_by_key[track.association_key] = observation;
    }

    if (database_ != nullptr) {
      tracker_.UpdateCycle(track_snapshots, observations_by_key, *database_,
                           policy_.recognition.feature_weights, sim_time_sec_,
                           input.input_cycle_index, input.batch_id);
    } else {
      tracker_.HoldCycle(track_snapshots, sim_time_sec_);
    }

    latest_summary_ = tracker_.BuildSummary(track_snapshots);
    latest_summary_.dwell_budget.scheduled_dwell_count = scheduled_count;
    latest_summary_.dwell_budget.executed_dwell_count = executed_count;
    latest_summary_.dwell_budget.dwell_budget_sec = static_cast<float>(scheduled_count) * dwell_sec;
    latest_summary_.dwell_budget.dwell_consumed_sec =
        static_cast<float>(executed_count) * dwell_sec;
    has_latest_summary_ = true;
  } else {
    track_snapshots = lifecycle_->BuildTrackSnapshots();
    tracker_.HoldCycle(track_snapshots, sim_time_sec_);
    latest_summary_ = tracker_.BuildSummary(track_snapshots);
    has_latest_summary_ = true;
  }

  last_track_snapshots_ = track_snapshots;
  output_frame->recognition_outputs.clear();
  output_frame->recognition_outputs.reserve(track_snapshots.size());
  for (const tracking::RirTrackState& track : track_snapshots) {
    session::RirTrackRecognitionOutput output;
    output.association_key = track.association_key;
    const session::RirRecognitionResult* result = tracker_.FindResult(track.association_key);
    if (result != nullptr) {
      output.result = *result;
    }
    output_frame->recognition_outputs.push_back(output);
  }
}

bool RirController::IsTargetRecognized(std::uint64_t external_target_id) const {
  if (external_target_id == 0U) {
    return false;
  }
  for (const tracking::RirTrackState& track : last_track_snapshots_) {
    if (track.external_target_id != external_target_id) {
      continue;
    }
    const session::RirRecognitionResult* result = tracker_.FindResult(track.association_key);
    if (result != nullptr &&
        (result->state == session::RirRecognitionState::kCategoryConfirmed ||
         result->state == session::RirRecognitionState::kModelConfirmed)) {
      return true;
    }
  }
  return false;
}

}  // namespace runtime
}  // namespace remote_identification_radar
