/**
 * @file RirController.cpp
 * @brief 远程识别雷达自持链路控制器实现（阶段 2-S S2）。
 */

#include "remote_identification_radar/runtime/RirController.h"

#include <Eigen/Cholesky>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <unordered_map>
#include <utility>
#include <vector>

#include "common/logging/AcceptanceText.h"
#include "common/logging/ProjectLog.h"
#include "common/numerics/Constants.h"
#include "common/radar/VegetationClutterModel.h"
#include "1q/coordinate/position_transform.h"
#include "1q/environment/AtmosphericTypes.h"
#include "1q/remote_identification_radar/session/RirIssueCodes.h"
#include "common/atmosphere/AtmospherePhysics.h"
#include "remote_identification_radar/dwell/RirBeamControl.h"
#include "remote_identification_radar/dwell/RirEffectiveRcs.h"
#include "remote_identification_radar/dwell/RirEmissionFactory.h"
#include "remote_identification_radar/dwell/RirMeasurementErrorModel.h"
#include "remote_identification_radar/dwell/RirReceiverStateBuilder.h"
#include "remote_identification_radar/dwell/RirRfFrontEndResolver.h"
#include "remote_identification_radar/internal/RirScanVolume.h"
#include "remote_identification_radar/recognition/RecognitionObservationBuilder.h"
#include "remote_identification_radar/runtime/RirAcceptanceLog.h"
#include "remote_identification_radar/runtime/RirAcceptanceRecords.h"

namespace remote_identification_radar {
namespace runtime {

namespace {

constexpr float kSnrFallbackGateDb = 6.0f;
constexpr float kCovarianceFloorM2 = 1.0e-6f;
std::string g_acceptance_found_targets;

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

/** @brief 内部航迹生命周期状态 → 公开枚举镜像（归属记录透出，观测投影消费）。 */
session::RirTrackLifecycleStatus ToPublicTrackStatus(tracking::RirTrackStatus status) {
  switch (status) {
    case tracking::RirTrackStatus::kConfirmed:
      return session::RirTrackLifecycleStatus::kConfirmed;
    case tracking::RirTrackStatus::kLost:
      return session::RirTrackLifecycleStatus::kLost;
    case tracking::RirTrackStatus::kTentative:
    default:
      return session::RirTrackLifecycleStatus::kTentative;
  }
}

/** @brief 内部四维观测 → 公开特征量测镜像（字段同值透出，仅命名对齐公开契约）。 */
session::RirFeatureObservations ToPublicFeatureObservations(const recognition::RirFeatureSet& set) {
  session::RirFeatureObservations features;
  features.rcs.valid = set.rcs.valid;
  features.rcs.mean_dbsm = set.rcs.mean_dbsm;
  features.rcs.std_db = set.rcs.std_db;
  features.rcs.azimuth_variation_db = set.rcs.azimuth_variation_db;
  features.rcs.elevation_variation_db = set.rcs.elevation_variation_db;
  features.rcs.peak_to_valley_db = set.rcs.peak_to_valley_db;
  features.rcs.aspect_coverage_deg = set.rcs.aspect_coverage_deg;
  features.rcs.quality = set.rcs.quality;
  features.motion.valid = set.motion.valid;
  features.motion.speed_m_per_s = set.motion.speed_mps;
  features.motion.altitude_m = set.motion.altitude_m;
  features.motion.acceleration_m_per_s2 = set.motion.acceleration_mps2;
  features.motion.turn_radius_m = set.motion.turn_radius_m;
  features.motion.is_straight = set.motion.is_straight;
  features.motion.quality = set.motion.quality;
  features.polarization.valid = set.polarization.valid;
  features.polarization.energy_difference_db = set.polarization.energy_difference_db;
  features.polarization.relative_difference_db = set.polarization.relative_difference_db;
  features.polarization.energy_sum_db = set.polarization.energy_sum_db;
  features.polarization.quality = set.polarization.quality;
  features.range_profile.valid = set.range_profile.valid;
  features.range_profile.length_m = set.range_profile.length_m;
  features.range_profile.peak_count = set.range_profile.peak_count;
  features.range_profile.peak_energy_concentration = set.range_profile.peak_energy_concentration;
  features.range_profile.resolution_m = set.range_profile.resolution_m;
  features.range_profile.quality = set.range_profile.quality;
  return features;
}

/// 格式化量值为两位小数（消息文本稳定，不承诺解析稳定性；规则 13b message 约定）。
std::string RirFormatFloat(float value) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.2f", static_cast<double>(value));
  return buffer;
}

/// 构造 kInfo 级按目标排除诊断（不属于三写，仅承载排查信息；规则 13b）。
/// @param cause 门内归因；聚合门排除须给出主因，具体门可 kNone。
/// @param target_index 场景目标下标；写入 `location = {kSceneEntity, index}` 供
///                     跨周期差分记录器按实体关联消费。
session::RirIssue RirMakeExclusionIssue(const char* code, const std::string& message,
                                        session::RirIssueCause cause,
                                        std::size_t target_index) {
  session::RirIssue issue;
  issue.severity = session::RirIssueSeverity::kInfo;
  issue.phase = session::RirIssuePhase::kExecution;
  issue.code = code;
  issue.message = message;
  issue.cause = cause;
  issue.location.kind = oneq::foundation::ValidationLocationKind::kSceneEntity;
  issue.location.entity_index = target_index;
  return issue;
}

// 规则 13b 门内归因：检测准入门（detector 判决/SNR 回退门）折入距离/波束/噪声底/
// RCS 多种物理因素，按各因素相对参考状态的损失 dB 判定主因（损失最大者）。
// 参考态与 AR ClassifySnrExclusionCause 同口径：1 km 距离、主瓣中心增益、
// 1 m² RCS、热噪声底、零传播损耗；传播损耗并入距离项。
session::RirIssueCause ClassifyRirDetectionExclusionCause(
    float range_m, float effective_rcs_m2, float one_way_antenna_gain_db,
    float main_beam_gain_db, float propagation_loss_db, float total_noise_w,
    float thermal_noise_w) {
  constexpr float kReferenceRangeM = 1000.0f;
  constexpr float kMinRcsM2 = 1.0e-12f;
  constexpr float kMinNoiseW = 1.0e-30f;

  const float distance_loss_db =
      4.0f * 10.0f * std::log10(std::max(range_m, 1.0e-3f) / kReferenceRangeM) +
      std::max(propagation_loss_db, 0.0f);
  const float beam_loss_db =
      2.0f * std::max(main_beam_gain_db - one_way_antenna_gain_db, 0.0f);
  const float rcs_loss_db = -10.0f * std::log10(std::max(effective_rcs_m2, kMinRcsM2));
  const float noise_loss_db =
      thermal_noise_w > 0.0f
          ? 10.0f * std::log10(std::max(total_noise_w, kMinNoiseW) /
                               std::max(thermal_noise_w, kMinNoiseW))
          : 0.0f;

  float max_loss_db = 0.0f;
  session::RirIssueCause cause = session::RirIssueCause::kUnknown;
  const struct {
    float loss_db;
    session::RirIssueCause cause;
  } kCandidates[] = {
      {distance_loss_db, session::RirIssueCause::kDistanceLimited},
      {beam_loss_db, session::RirIssueCause::kBeamLimited},
      {noise_loss_db, session::RirIssueCause::kNoiseLimited},
      {rcs_loss_db, session::RirIssueCause::kRcsLimited},
  };
  for (const auto& candidate : kCandidates) {
    if (candidate.loss_db > max_loss_db) {
      max_loss_db = candidate.loss_db;
      cause = candidate.cause;
    }
  }
  return max_loss_db > 0.0f ? cause : session::RirIssueCause::kUnknown;
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

void RirController::SetSensorPlatformId(std::uint64_t sensor_platform_id) {
  sensor_platform_id_ = sensor_platform_id;
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
    const std::chrono::steady_clock::time_point load_begin = std::chrono::steady_clock::now();
    const bool loaded = recognition::RirFeatureDatabase::Load(
        policy_.recognition.database_path, candidate.get(), &error);
    last_database_load_ms_ = std::chrono::duration<double, std::milli>(
                                 std::chrono::steady_clock::now() - load_begin)
                                 .count();
    if (loaded) {
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

void RirController::UpdateEnvironment(const config::RirEnvironmentConfig& environment) {
  environment_ = environment;
  // k 因子随气象观测派生（对标 AR EnvironmentService 冻结快照口径；非法输入内部兜底 4/3）。
  effective_k_factor_ = oneq::environment::ResolveEffectiveKFactor(
      environment.atmospheric_physics);
}

void RirController::ResolveEnvironment(float* propagation_loss_db, float* clutter_power_w) const {
  *propagation_loss_db = 0.0f;
  *clutter_power_w = 0.0f;
  if (!environment_.enable_environment_effects) {
    return;
  }
  internal::RirEnvironmentSceneState scene_state;
  scene_state.vegetation_scatter_physics = environment_.vegetation_scatter_physics;
  const internal::RirPropagationResult propagation = propagation_model_.Evaluate(scene_state);
  *propagation_loss_db =
      propagation.propagation_loss_db + environment_.weather_attenuation_db;
  // clutter_power_db 是相对热噪底的 dB（CNR 口径，与 AR 同口径单源换算），
  // 不得解释为绝对 dBW。
  const float thermal_noise_w = internal::RirRadarEquations::ComputeThermalNoisePower_W(
      hardware_.transmitter, hardware_.receiver);
  *clutter_power_w = oneq::common::radar::ComputeEquivalentClutterNoiseW(
      thermal_noise_w, propagation.clutter_power_db);
}

float RirController::ComputeTargetAtmosphericLossDb(float carrier_hz, float platform_altitude_m,
                                                    float look_el_deg, bool has_look_angles,
                                                    float slant_range_m,
                                                    float target_position_z_m) const {
  if (!environment_.atmospheric_physics.enable_physical_model) {
    return 0.0f;
  }
  // 与 AR DetectionExecution 同口径：common 标量胶水 + 模块侧 enable/k_factor。
  oneq::common::atmosphere::AtmosphericObservationRef obs;
  obs.pressure_hpa = environment_.atmospheric_physics.pressure_hpa;
  obs.temperature_k = environment_.atmospheric_physics.temperature_k;
  obs.relative_humidity = environment_.atmospheric_physics.relative_humidity;
  obs.k_factor = effective_k_factor_;
  return oneq::common::atmosphere::ComputeTargetAtmosphericPhysicsLossDb(
      carrier_hz, slant_range_m, platform_altitude_m,
      std::max(platform_altitude_m + target_position_z_m, 0.0f), look_el_deg, has_look_angles, obs);
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

RirController::RirResolvedRfCycle RirController::ResolveRfCycle(
    const session::RirCycleInput& input,
    const config::RirAzimuthElevationDeg& dwell_center_deg) const {
  RirResolvedRfCycle resolved;
  if (sensor_platform_id_ == 0U || hardware_.transmitter.prf_hz <= 0.0f) {
    return resolved;
  }

  dwell::RirRfCycleInput rf_input;
  rf_input.platform_id = sensor_platform_id_;
  rf_input.platform_position_ecef_m = input.platform_position;
  rf_input.window_start_time_s = static_cast<double>(input.sim_time_sec);
  rf_input.window_duration_s = static_cast<double>(mission_.recognition_dwell_sec);
  rf_input.beam_pointing_deg = dwell_center_deg;

  const double carrier_hz =
      dwell::RirEmissionFactory::ResolveCarrierHz(hardware_.transmitter, input.input_cycle_index);
  const double pri_s = 1.0 / static_cast<double>(hardware_.transmitter.prf_hz);
  // 与 AR PrepareRfCycle 同口径：驻留窗内脉冲数按 ceil 计（首沿落入窗口的脉冲
  // 全部计入积累增益），下限 1 保持驻留窗短于一个 PRI 时仍有单脉冲。
  const double pulse_count_value =
      std::ceil(static_cast<double>(mission_.recognition_dwell_sec) / pri_s);
  const std::uint32_t pulse_count =
      static_cast<std::uint32_t>(std::max(1.0, pulse_count_value));

  oneq::electromagnetics::RfSceneEmission own_emission;
  if (!dwell::RirEmissionFactory::TryBuildEmission(
          rf_input, hardware_, static_cast<std::uint64_t>(input.input_cycle_index), carrier_hz,
          pri_s, pulse_count, static_cast<std::uint64_t>(detection_random_seed_),
          static_cast<std::uint64_t>(input.input_cycle_index), &own_emission)) {
    return resolved;
  }

  resolved.own_emission_identity = own_emission.identity;
  resolved.own_emission = own_emission;
  resolved.own_transmit_waveform = own_emission.waveform;
  resolved.carrier_hz = static_cast<float>(carrier_hz);

  const dwell::RirReceiverOperatingState receiver_state =
      dwell::RirReceiverStateBuilder::Build(rf_input, own_emission, hardware_, carrier_hz);

  oneq::electromagnetics::RfSceneFrame resolved_scene = input.rf_scene;
  resolved_scene.world_cycle_index = input.input_cycle_index;
  resolved_scene.window_start_time_s = rf_input.window_start_time_s;
  resolved_scene.window_duration_s = rf_input.window_duration_s;
  resolved_scene.emissions.push_back(own_emission);

  dwell::RirRfFrontEndResult front_end;
  if (!dwell::TryResolveRirRfFrontEnd(
          resolved_scene, receiver_state.rf_receiver,
          receiver_state.maximum_linear_input_power_w,
          oneq::electromagnetics::RfIncidentLinkConfig{}, &front_end)) {
    return resolved;
  }

  resolved.incident_links = std::move(front_end.incident_links);
  resolved.receiver_saturated = front_end.receiver_saturated;
  resolved.resolved = true;
  return resolved;
}

bool RirController::TryBuildMeasurement(
    const session::RirSceneTarget& target, std::size_t source_index, float platform_altitude_m,
    float propagation_loss_db, float clutter_power_w, const session::RirCycleInput& input,
    const config::RirAzimuthElevationDeg& dwell_center_deg, const RirResolvedRfCycle& rf_cycle,
    tracking::RirTrackMeasurement* measurement, float* snr_db) {
  if (measurement == nullptr || snr_db == nullptr) {
    return false;
  }
  float look_az_deg = 0.0f;
  float look_el_deg = 0.0f;
  float slant_range_m = 0.0f;
  ComputeLookAngles(target, &look_az_deg, &look_el_deg, &slant_range_m);

  // 与 AR TargetLookResolver 同口径：位置范数 ≤0.1 m 时视线角无效（range_m 兜底
  // 得到的 az=0/el=0 不是真实视线）→ 方向图增益回退主瓣峰值，而非按兜底角离轴衰减。
  const Eigen::Vector3f raw_position(target.position_x, target.position_y, target.position_z);
  const bool has_look_angles = raw_position.norm() > 0.1f;
  const float carrier_hz =
      rf_cycle.carrier_hz > 0.0f ? rf_cycle.carrier_hz : hardware_.transmitter.frequency_hz;
  // 逐目标大气物理附加损耗（与 AR 同口径：全局植被/天气损耗之外的按几何分量）。
  const float atmospheric_loss_db = ComputeTargetAtmosphericLossDb(
      carrier_hz, platform_altitude_m, look_el_deg, has_look_angles, slant_range_m,
      PositionOf(target).z());
  const float total_propagation_loss_db = propagation_loss_db + atmospheric_loss_db;
  // 与 AR DetectionExecution 同口径：主路径与回退分支波长同源——回退分支的有效
  // 波束宽度同样经 λ/L 物理推导（结果喂量测误差模型）；载频非正时不启用物理推导。
  const float wavelength_m = carrier_hz > 0.0f
                                 ? static_cast<float>(oneq::common::numerics::kLightSpeed) /
                                       carrier_hz
                                 : 0.0f;
  dwell::RirResolvedBeamState beam_state;
  beam_state.one_way_antenna_gain_db = hardware_.antenna.main_beam_gain_db;
  beam_state.effective_beamwidth_deg =
      dwell::RirResolveEffectiveBeamwidth(hardware_.antenna, wavelength_m);
  if (hardware_.antenna.enable_directional_pattern && has_look_angles) {
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

  dwell::RirDetectionResult detection;
  dwell::RirDetectionCellResult cell;
  bool resolved_cell = false;
  if (rf_cycle.resolved) {
    if (rf_cycle.receiver_saturated) {
      // 中译：接收前端饱和，detection cell 可能失真，仍尝试求解并在失败时回退。
      // 标识：RF 前端线性区越界——饱和标志为 true 时记录 WARN。
      PROJECT_LOG_WARN("[RirController] receiver front-end saturated at cycle={}",
                       input.input_cycle_index);
    }
    dwell::RirDetectionCellConfig cell_config;
    cell_config.own_transmit_waveform = rf_cycle.own_transmit_waveform;
    cell_config.receive_window_start_time_s = static_cast<double>(input.sim_time_sec);
    cell_config.receive_window_duration_s = static_cast<double>(mission_.recognition_dwell_sec);
    // 与 AR DetectionExecution 同口径：匹配滤波带宽取本周期波形实际占用带宽
    // （当前发射工厂与 hardware bandwidth_hz 恒等；波形级带宽解耦后两侧同源）。
    cell_config.matched_filter_bandwidth_hz =
        rf_cycle.own_transmit_waveform.occupied_bandwidth_hz;
    cell_config.one_way_antenna_gain_dbi = static_cast<double>(beam_state.one_way_antenna_gain_db);
    cell_config.receiver_loss_db = static_cast<double>(hardware_.receiver.receive_loss_db);
    cell_config.receiver_noise_figure_db = static_cast<double>(hardware_.receiver.noise_figure_db);
    cell_config.signal_processing = hardware_.signal_processing;

    dwell::RirDetectionCellTarget cell_target;
    cell_target.range_m = static_cast<double>(slant_range_m);
    const Eigen::Vector3f position = PositionOf(target);
    const float range_norm = std::max(position.norm(), 1.0f);
    const float radial_velocity =
        (position.x() * target.velocity_x + position.y() * target.velocity_y +
         position.z() * target.velocity_z) /
        range_norm;
    cell_target.closing_radial_velocity_mps = static_cast<double>(-radial_velocity);
    dwell::RirTargetLookAngles look_angles;
    look_angles.look_az_deg = look_az_deg;
    look_angles.look_el_deg = look_el_deg;
    look_angles.has_look_angles = has_look_angles;
    const float carrier_hz =
        rf_cycle.carrier_hz > 0.0f ? rf_cycle.carrier_hz : hardware_.transmitter.frequency_hz;
    cell_target.rcs_m2 = static_cast<double>(dwell::ComputeEffectiveTargetRcsM2(
        target, look_angles, hardware_.rcs_physics, carrier_hz));
    cell_target.two_way_additional_propagation_loss_db =
        static_cast<double>(total_propagation_loss_db);
    cell_target.effective_pulse_count =
        static_cast<std::uint32_t>(std::max(1, policy_.detection.pulse_count));

    if (dwell::TryResolveRirDetectionCell(cell_config, cell_target, rf_cycle.own_emission_identity,
                                        rf_cycle.incident_links, static_cast<double>(clutter_power_w),
                                        &cell)) {
      target_return.rcs_m2 = static_cast<float>(cell_target.rcs_m2);
      detection = detector_->DetectResolvedCell(target_return, cell);
      resolved_cell = true;
    } else {
      // 中译：detection cell 求解失败，回退效能级检测路径。
      // 标识：数值/输入保护——RF 分解失败时不丢目标，按旧口径继续判决。
      PROJECT_LOG_WARN("[RirController] detection cell resolve failed for target id={}",
                       target.external_target_id);
    }
  }

  if (!resolved_cell) {
    dwell::RirEnvironmentNoise env;
    env.propagation_loss_db = total_propagation_loss_db;
    env.clutter_noise_w = clutter_power_w;
    env.jam_noise_w = 0.0f;
    detection = detector_->Detect(target_return, env, beam_state.one_way_antenna_gain_db,
                                  std::max(1, policy_.detection.pulse_count));
  }

  *snr_db = detection.snr_db;
  const bool admitted = policy_.detection.gate_mode == config::RirDetectionGateMode::kSnrFallback
                            ? detection.snr_db >= kSnrFallbackGateDb
                            : detection.detected;
  // 验收事件 detection_cell（3.2.2.1.1.x/3.2.2.1.2）：逐目标检测链物理量——方向图
  // 离轴增益/有效波束宽度/离轴角、回波/热噪/干扰/杂波功率、脉压增益、SINR/SNR、
  // Pd 与判决。has_cell=0 为 v1 效能级回退路径（cell 分项功率不可得，只有 SNR/Pd）。
  if (RIR_ACCEPTANCE_LOG_ENABLED()) {
    RirDetectionAcceptInput snap;
    snap.sim_time_sec = input.sim_time_sec;
    snap.cycle = input.input_cycle_index;
    snap.target_id = target.external_target_id;
    snap.range_m = slant_range_m;
    snap.look_az_deg = look_az_deg;
    snap.look_el_deg = look_el_deg;
    snap.rcs_m2 = target_return.rcs_m2;
    snap.snr_db = detection.snr_db;
    snap.pd = detection.detection_prob;
    snap.detected = detection.detected;
    snap.has_cell = resolved_cell;
    snap.peak_gain_dbi = hardware_.antenna.main_beam_gain_db;
    snap.bw_az_deg = beam_state.effective_beamwidth_deg.az_beamwidth_deg;
    snap.bw_el_deg = beam_state.effective_beamwidth_deg.el_beamwidth_deg;
    snap.echo_power_dbw = detection.echo_power_dbw;
    snap.cell = cell;
    snap.gains = hardware_.signal_processing;
    snap.cfar_pfa = static_cast<double>(policy_.detection.cfar_pfa);
    snap.prf_hz = static_cast<double>(hardware_.transmitter.prf_hz);
    snap.center_frequency_hz =
        rf_cycle.own_transmit_waveform.center_frequency_hz > 0.0
            ? rf_cycle.own_transmit_waveform.center_frequency_hz
            : static_cast<double>(hardware_.transmitter.frequency_hz);
    snap.incident_links.reserve(rf_cycle.incident_links.size());
    for (const auto& link : rf_cycle.incident_links) {
      if (link.identity.platform_id == rf_cycle.own_emission_identity.platform_id &&
          link.identity.equipment_id == rf_cycle.own_emission_identity.equipment_id &&
          link.identity.emission_id == rf_cycle.own_emission_identity.emission_id) {
        continue;
      }
      snap.incident_links.push_back(link);
    }
    WriteRirDetectionChain(snap);
  }

  if (!admitted) {
    // 规则 13b：检测准入门排除 → kInfo 诊断（不属于三写，仅承载排查信息）。
    // 检测门为聚合门（距离/波束/噪声底/RCS 折入单一判决），按各因素相对参考态
    // 损失 dB 判定主因（与 AR ClassifySnrExclusionCause 同口径）。
    const float thermal_noise_w = internal::RirRadarEquations::ComputeThermalNoisePower_W(
        hardware_.transmitter, hardware_.receiver);
    float total_noise_w = thermal_noise_w + clutter_power_w;
    if (resolved_cell) {
      total_noise_w = static_cast<float>(cell.thermal_noise_power_w + cell.clutter_power_w +
                                         cell.interference_power_w);
    }
    const session::RirIssueCause cause = ClassifyRirDetectionExclusionCause(
        slant_range_m, target_return.rcs_m2, beam_state.one_way_antenna_gain_db,
        hardware_.antenna.main_beam_gain_db, total_propagation_loss_db, total_noise_w,
        thermal_noise_w);
    const char* gate_mode_text =
        policy_.detection.gate_mode == config::RirDetectionGateMode::kSnrFallback
            ? "snr_fallback"
            : "detector";
    last_execution_issues_.push_back(RirMakeExclusionIssue(
        session::codes::kTargetDetectionGate,
        "target_id=" + std::to_string(target.external_target_id) + "; snr_db=" +
            RirFormatFloat(detection.snr_db) + " range_m=" + RirFormatFloat(slant_range_m) +
            " rcs_m2=" + RirFormatFloat(target_return.rcs_m2) +
            " rejected by detection gate (mode=" + gate_mode_text + ")",
        cause, source_index));
    return false;
  }
  if (RIR_ACCEPTANCE_LOG_ENABLED()) {
    if (!g_acceptance_found_targets.empty()) {
      g_acceptance_found_targets += "; ";
    }
    g_acceptance_found_targets +=
        "ID=" + std::to_string(target.external_target_id) + " 斜距=" +
        std::to_string(static_cast<int>(slant_range_m)) + "m Pd=" +
        oneq::logging::FormatF(detection.detection_prob, 5);
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
  // 标量速度观测（与 AR filtered_feature.observed_speed 同位）：速度向量为零时
  // 生命周期速度种子按 (observed_speed, 0, 0) 回退的基准。
  built.observed_speed = built.velocity.norm();
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
                             session::RirOutputFrame* output_frame, std::uint64_t batch_id,
                             const config::RirAzimuthElevationDeg& dwell_center_deg,
                             const config::RirAzimuthElevationLimitsDeg& steerable_volume_deg,
                             const config::RirAzimuthElevationDeg& scan_center_deg) {
  const bool in_identify = work_mode_ == config::RirWorkMode::kIdentify;
  if (recognition_mode_active_ && !in_identify) {
    tracker_.ExitRecognitionMode();
  }
  recognition_mode_active_ = in_identify;
  sim_time_sec_ = input.sim_time_sec;

  float platform_altitude_m = 0.0f;
  oneq::coordinate::LlaPositionDegM platform_lla;
  if (oneq::coordinate::TryEcefToLla(input.platform_position, &platform_lla)) {
    platform_altitude_m = static_cast<float>(platform_lla.altitude_m);
  }

  latest_summary_ = session::RirRecognitionCycleSummary{};
  has_latest_summary_ = false;
  // 出口①与归属视图按周期重置（透出原则：识别链未构建观测的周期为空）。
  output_frame->feature_measurements.clear();
  last_track_attributions_.clear();
  last_execution_issues_.clear();
  last_emission_frame_ = {};

  std::vector<tracking::RirTrackState> track_snapshots;
  if (in_identify) {
    float propagation_loss_db = 0.0f;
    float clutter_power_w = 0.0f;
    ResolveEnvironment(&propagation_loss_db, &clutter_power_w);

    const RirResolvedRfCycle rf_cycle = ResolveRfCycle(input, dwell_center_deg);
    if (rf_cycle.resolved) {
      last_emission_frame_.world_cycle_index = input.input_cycle_index;
      last_emission_frame_.window_start_time_s = static_cast<double>(input.sim_time_sec);
      last_emission_frame_.window_duration_s =
          static_cast<double>(mission_.recognition_dwell_sec);
      last_emission_frame_.emissions.push_back(rf_cycle.own_emission);
    }

    // 验收事件 interference_link（3.2.2.1.1.3）：逐干扰源到达接收端的入射功率
    // （RF 前端口径，未做 detection cell 内时频重叠聚合；聚合后进入 SINR 分母的
    // 总量见 detection_cell 事件 interference_w 字段）。
    if (RIR_ACCEPTANCE_LOG_ENABLED()) {
      g_acceptance_found_targets.clear();
      WriteRirInterferenceLinks(input.sim_time_sec, input.input_cycle_index, rf_cycle.incident_links);
    }

    std::unordered_map<std::uint64_t, const session::RirSceneTarget*> scene_by_id;
    std::vector<std::size_t> candidate_indices;
    for (std::size_t i = 0U; i < input.scene_targets.size(); ++i) {
      const session::RirSceneTarget& target = input.scene_targets[i];
      if (target.external_target_id == 0U) {
        continue;
      }
      scene_by_id[target.external_target_id] = &target;
      // 2026-08-22 甲方批注「设定方位俯仰进行扫描」：搜索候选集按可扫描体积
      // 裁剪（az 相对 scan_center、el 绝对；与指定识别目标驻留门同口径）——
      // 视线角出角域的目标不入检测候选，不再依赖方向图衰减软门控。
      float look_az_deg = 0.0f;
      float look_el_deg = 0.0f;
      float slant_range_m = 0.0f;
      ComputeLookAngles(target, &look_az_deg, &look_el_deg, &slant_range_m);
      const config::RirAzimuthElevationDeg look{look_az_deg, look_el_deg};
      if (!internal::TargetWithinSteerableVolume(look, steerable_volume_deg, scan_center_deg)) {
        // 规则 13b：角域裁剪排除 → kInfo 诊断（不属于三写，仅承载排查信息）。
        // 出界目标不入检测候选、航迹按失跟语义自然消退，发码使"目标消失"可归因
        // （此前为静默 continue）。具体门 cause=kNone，az 相对 scan_center、el 绝对。
        last_execution_issues_.push_back(RirMakeExclusionIssue(
            session::codes::kTargetOutsideSearchVolume,
            "target_id=" + std::to_string(target.external_target_id) +
                "; look_az_deg=" + RirFormatFloat(look.az_deg) +
                " look_el_deg=" + RirFormatFloat(look.el_deg) +
                " outside steerable volume az_rel_deg=[" +
                RirFormatFloat(steerable_volume_deg.az_min_deg) + "," +
                RirFormatFloat(steerable_volume_deg.az_max_deg) + "] el_deg=[" +
                RirFormatFloat(steerable_volume_deg.el_min_deg) + "," +
                RirFormatFloat(steerable_volume_deg.el_max_deg) + "]",
            session::RirIssueCause::kNone, i));
        continue;
      }
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
      if (!TryBuildMeasurement(input.scene_targets[target_index], target_index, platform_altitude_m,
                               propagation_loss_db, clutter_power_w, input, dwell_center_deg,
                               rf_cycle, &measurement, &snr_db)) {
        continue;
      }
      // 规则 13b：检测通过后的识别链门控诊断（每目标至多一条，链上第一门优先；
      // 检测门失败已在 TryBuildMeasurement 内落诊断）。
      if (database_ == nullptr) {
        last_execution_issues_.push_back(RirMakeExclusionIssue(
            session::codes::kTargetNoFeatureDatabase,
            "target_id=" + std::to_string(input.scene_targets[target_index].external_target_id) +
                "; feature database unavailable, recognition held",
            session::RirIssueCause::kNone, target_index));
      } else {
        float look_az_deg = 0.0f;
        float look_el_deg = 0.0f;
        float slant_range_m = 0.0f;
        ComputeLookAngles(input.scene_targets[target_index], &look_az_deg, &look_el_deg,
                          &slant_range_m);
        if (slant_range_m > mission_.max_range_m) {
          last_execution_issues_.push_back(RirMakeExclusionIssue(
              session::codes::kTargetBeyondRecognitionRange,
              "target_id=" + std::to_string(input.scene_targets[target_index].external_target_id) +
                  "; slant_range_m=" + RirFormatFloat(slant_range_m) + " beyond max_range_m=" +
                  RirFormatFloat(mission_.max_range_m),
              session::RirIssueCause::kNone, target_index));
        }
      }
      snr_by_target_id[measurement.external_target_id] = snr_db;
      measurements.push_back(measurement);
      ++executed_count;
    }

    const tracking::RirAssociationResult association = associator_.Associate(
        measurements, lifecycle_->BuildAssociationSeeds(), static_cast<float>(input.dt_sec));
    tracking::RirCycleContext cycle_context;
    cycle_context.cycle_index = input.input_cycle_index;
    cycle_context.batch_id = batch_id;
    cycle_context.dt_sec = static_cast<float>(input.dt_sec);
    lifecycle_->Update(cycle_context, association.measurements);

    // 验收事件 association（3.2.2.3.1.1）：全局最优关联结果——命中对（键/量测
    // 索引/马氏代价）与漏检航迹键（量测清单经 detection_cell 事件逐目标留痕）。
    if (RIR_ACCEPTANCE_LOG_ENABLED()) {
      WriteRirSearchDetections(input.sim_time_sec, input.input_cycle_index, dwell_center_deg.az_deg,
                               dwell_center_deg.el_deg, steerable_volume_deg, scan_center_deg,
                               g_acceptance_found_targets);
      WriteRirAssociation(input.sim_time_sec, input.input_cycle_index, association);
    }

    track_snapshots = lifecycle_->BuildTrackSnapshots();

    // 验收事件 cluster（2026-08-22 甲方批注）：集群目标数量 = 确认航迹数
    // （计数口径；非检测条数）。
    if (RIR_ACCEPTANCE_LOG_ENABLED()) {
      WriteRirClusterCount(input.sim_time_sec, input.input_cycle_index, track_snapshots);
    }

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
          MakeObservationContext(*scene_found->second, platform_altitude_m, snr_db);
      observations_by_key[track.association_key] = observation;
    }

    if (database_ != nullptr) {
      // 出口①：采集本周期实际构建的有效特征观测（透出来源），组装公开量测记录。
      std::vector<recognition::RirTracker::CycleFeatureObservation> cycle_observations;
      tracker_.UpdateCycle(track_snapshots, observations_by_key, *database_,
                           policy_.recognition.feature_weights, sim_time_sec_,
                           input.input_cycle_index, batch_id, &cycle_observations);
      output_frame->feature_measurements.reserve(cycle_observations.size());
      for (const recognition::RirTracker::CycleFeatureObservation& observation :
           cycle_observations) {
        const auto context_found = observations_by_key.find(observation.association_key);
        if (context_found == observations_by_key.end()) {
          continue;  // UpdateCycle 只透出来自本表的键，此分支不可达（防御）。
        }
        const recognition::RirObservationContext& context = context_found->second.context;
        session::RirFeatureMeasurementRecord record;
        record.association_key = observation.association_key;
        record.features = ToPublicFeatureObservations(observation.features);
        record.valid_feature_mask = observation.features.valid_feature_mask;
        record.look_az_deg = context.look_az_deg;
        record.look_el_deg = context.look_el_deg;
        record.range_m = context.range_m;
        record.snr_db = context.snr_db;
        record.dwell_sec = context.dwell_sec;
        record.bandwidth_hz = context.bandwidth_hz;
        record.has_platform_position = true;
        record.platform_position = input.platform_position;
        record.cycle_index = input.input_cycle_index;
        record.batch_id = batch_id;
        output_frame->feature_measurements.push_back(record);
      }
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

    // 验收事件 schedule（3.2.2.4.2.2）：驻留执行计数与识别效能摘要。口径：
    // scheduled=非零 ID 场景目标数、executed=检测准入门通过数（事后统计，
    // 非调度器准入）；事件类型分类计数为既有口径所能提供的粒度。
    if (RIR_ACCEPTANCE_LOG_ENABLED()) {
      std::uint32_t confirmed = 0U;
      for (const tracking::RirTrackState& track : track_snapshots) {
        if (track.status == tracking::RirTrackStatus::kConfirmed) {
          ++confirmed;
        }
      }
      WriteRirSchedule(input.sim_time_sec, input.input_cycle_index, scheduled_count, executed_count,
                       latest_summary_.dwell_budget.dwell_budget_sec,
                       latest_summary_.dwell_budget.dwell_consumed_sec, 1U, confirmed,
                       latest_summary_.participating_track_count);
    }
  } else {
    track_snapshots = lifecycle_->BuildTrackSnapshots();
    tracker_.HoldCycle(track_snapshots, sim_time_sec_);
    latest_summary_ = tracker_.BuildSummary(track_snapshots);
    has_latest_summary_ = true;
    // 规则 13b：非识别工作模式 → 全局模式门，逐目标落 kInfo 诊断（不建识别观测，
    // 检测/跟踪不受影响的 STBY 保持周期）。
    for (std::size_t i = 0U; i < input.scene_targets.size(); ++i) {
      if (input.scene_targets[i].external_target_id == 0U) {
        continue;
      }
      last_execution_issues_.push_back(RirMakeExclusionIssue(
          session::codes::kTargetModeNotIdentify,
          "target_id=" + std::to_string(input.scene_targets[i].external_target_id) +
              "; work_mode=stby, no recognition observation this cycle",
          session::RirIssueCause::kNone, i));
    }
  }

  last_track_snapshots_ = track_snapshots;
  output_frame->recognition_outputs.clear();
  output_frame->recognition_outputs.reserve(track_snapshots.size());
  last_track_attributions_.reserve(track_snapshots.size());
  for (const tracking::RirTrackState& track : track_snapshots) {
    session::RirTrackRecognitionOutput output;
    output.association_key = track.association_key;
    const session::RirRecognitionResult* result = tracker_.FindResult(track.association_key);
    if (result != nullptr) {
      output.result = *result;
    }
    output_frame->recognition_outputs.push_back(output);
    if (RIR_ACCEPTANCE_LOG_ENABLED()) {
      const session::RirFeatureMeasurementRecord* features = nullptr;
      for (const session::RirFeatureMeasurementRecord& record : output_frame->feature_measurements) {
        if (record.association_key == track.association_key) {
          features = &record;
          break;
        }
      }
      const std::vector<session::RirPolarizationRcsSample>* polarization_samples = nullptr;
      for (const session::RirSceneTarget& target : input.scene_targets) {
        if (target.external_target_id == track.external_target_id) {
          polarization_samples = &target.polarization_rcs_samples;
          break;
        }
      }
      std::vector<float> imm_weights;
      if (lifecycle_ != nullptr) {
        const tracking::RirImmFilter* imm = lifecycle_->FindImmFilter(track.association_key);
        if (imm != nullptr && imm->IsValid()) {
          const Eigen::VectorXf weights = imm->GetModelWeights();
          imm_weights.reserve(static_cast<std::size_t>(weights.size()));
          for (Eigen::Index i = 0; i < weights.size(); ++i) {
            imm_weights.push_back(weights(i));
          }
        }
      }
      WriteRirTrackAndId(input.sim_time_sec, input.input_cycle_index, track, result, features,
                         polarization_samples, latest_summary_.has_ground_truth,
                         static_cast<double>(latest_summary_.category_accuracy), &imm_weights);
    }
    // 归属视图与出口②同循环产出（全部航迹快照：tentative/confirmed/lost）。
    session::RirTrackAttributionRecord attribution;
    attribution.association_key = track.association_key;
    attribution.external_target_id = track.external_target_id;
    attribution.target_name = track.target_name;
    attribution.track_status = ToPublicTrackStatus(track.status);
    attribution.hit_count = track.hit_count;
    attribution.position_enu_x_m = static_cast<double>(track.position.x());
    attribution.position_enu_y_m = static_cast<double>(track.position.y());
    attribution.position_enu_z_m = static_cast<double>(track.position.z());
    attribution.speed_m_per_s = static_cast<double>(track.speed);
    last_track_attributions_.push_back(attribution);
  }
  if (RIR_ACCEPTANCE_LOG_ENABLED()) {
    std::string multi = "本周期航迹数=" + std::to_string(track_snapshots.size());
    std::size_t index = 1U;
    for (const tracking::RirTrackState& track : track_snapshots) {
      multi += " 航迹" + std::to_string(index) + "→目标" + std::to_string(track.external_target_id);
      ++index;
    }
    RIR_ACCEPTANCE_ITEM(input.sim_time_sec, input.input_cycle_index, "多目标跟踪", multi);
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
