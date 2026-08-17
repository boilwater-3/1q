#include "sbirs_sensor/pipeline/SbirsPipeline.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <set>
#include <string>

#include "common/logging/ProjectLog.h"
#include "common/numerics/Constants.h"
#include "1q/coordinate/inertial_transform.h"
#include "1q/sbirs_sensor/session/SbirsIssueCodes.h"
#include "sbirs_sensor/environment/SbirsEnvironmentModel.h"
#include "sbirs_sensor/foundation/SbirsErrorModel.h"
#include "sbirs_sensor/foundation/SbirsGeometry.h"
#include "sbirs_sensor/foundation/SbirsNoiseModel.h"
#include "sbirs_sensor/foundation/SbirsRadiometry.h"
#include "sbirs_sensor/pipeline/SbirsBoresightChain.h"
#include "sbirs_sensor/pipeline/SbirsEciScene.h"
#include "sbirs_sensor/pipeline/SbirsNfovAcquisition.h"

namespace sbirs_sensor {
namespace pipeline {
namespace {

const double kEarthRadiusM = 6371000.0;

// 规则 13b：正常执行周期按目标门控排除的 kInfo 诊断码（不属于三写，仅承载排查信息）。
// code 引用 SbirsIssueCodes.h 注册表常量。

/// 构造 kInfo 级按目标排除诊断（不属于三写，仅承载排查信息；规则 13b）。
/// @param cause 门内归因（规则 13b 门内归因条款）；聚合门排除须给出主因，具体门可 kNone。
/// @param target_index 场景目标索引；非负时写入 `location = {kSceneEntity, target_index}`，
///                     供跨周期差分记录器按实体关联消费（默认 -1 保持 kGlobal，向后兼容）。
session::SbirsIssue MakeExclusionIssue(const char* code, const std::string& message,
                                       session::SbirsIssueCause cause =
                                           session::SbirsIssueCause::kNone,
                                       std::ptrdiff_t target_index = -1) {
  session::SbirsIssue issue;
  issue.severity = session::SbirsIssueSeverity::kInfo;
  issue.code = code;
  issue.message = message;
  issue.cause = cause;
  if (target_index >= 0) {
    issue.location.kind = oneq::foundation::ValidationLocationKind::kSceneEntity;
    issue.location.entity_index = static_cast<std::size_t>(target_index);
  }
  return issue;
}

// 规则 13b 门内归因：WFOV SNR 门失败主因分类。SNR 门为聚合门（距离²/大气透过率/
// 目标签名折入单一门限），反事实判定主因——各因子取"达标参考值"后 SNR 提升（dB）
// 最大者：距离参考 1000 km、大气全透过、目标签名取"使 SNR 恰达门限的签名"
//（signature_required 仅依赖硬件/门限配置，调用方在循环外计算）；目标签名即
// 输入辐射强度 I_t（W/sr）。噪声为硬件常数，不参与单目标归因。全部损失 <= 0 时
// 返回 kUnknown。
session::SbirsIssueCause ClassifyWfovSnrExclusionCause(double range_m, float transmittance,
                                                       double signature_actual,
                                                       double signature_required) {
  constexpr double kReferenceRangeM = 1.0e6;
  const double distance_loss_db =
      20.0 * std::log10(std::max(range_m, 1.0) / kReferenceRangeM);
  const double attenuation_loss_db = -20.0 * std::log10(std::max(transmittance, 1.0e-6f));
  const double signature_loss_db =
      signature_actual > 0.0 && signature_required > 0.0
          ? 10.0 * std::log10(signature_required / signature_actual)
          : 0.0;
  session::SbirsIssueCause cause = session::SbirsIssueCause::kUnknown;
  double max_loss_db = 0.0;
  const struct {
    double loss_db;
    session::SbirsIssueCause cause;
  } kCandidates[] = {
      {distance_loss_db, session::SbirsIssueCause::kDistanceLimited},
      {attenuation_loss_db, session::SbirsIssueCause::kAttenuationLimited},
      {signature_loss_db, session::SbirsIssueCause::kSignatureLimited},
  };
  for (const auto& candidate : kCandidates) {
    if (candidate.loss_db > max_loss_db) {
      max_loss_db = candidate.loss_db;
      cause = candidate.cause;
    }
  }
  return max_loss_db > 0.0 ? cause : session::SbirsIssueCause::kUnknown;
}

std::string FormatFloat(float value) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.1f", static_cast<double>(value));
  return buffer;
}

std::string FormatSnr(double value) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.3e", value);
  return buffer;
}

std::uint32_t DeriveMeasurementSeed(std::uint32_t base_seed, std::uint32_t domain_tag) {
  std::uint32_t value = base_seed ^ domain_tag;
  value ^= value >> 16U;
  value *= 0x7feb352dU;
  value ^= value >> 15U;
  value *= 0x846ca68bU;
  value ^= value >> 16U;
  return value == 0U ? 1U : value;
}

const std::uint32_t kWfovMeasurementDomain = UINT32_C(0x57464f56);
const std::uint32_t kEstimatedMeasurementDomain = UINT32_C(0x4553544d);
const std::uint32_t kSensorLikeOutputDomain = UINT32_C(0x534c4f55);

bool IsTruthTrackingState(SbirsTargetState state) {
  return state == SbirsTargetState::kStrictTruthAssistedTracking ||
         state == SbirsTargetState::kSensorLikeTruthAssistedTracking;
}

attribution::SbirsTrackingSource TrackingSourceForState(SbirsTargetState state) {
  switch (state) {
    case SbirsTargetState::kEstimatedTracking:
      return attribution::SbirsTrackingSource::kEstimated;
    case SbirsTargetState::kStrictTruthAssistedTracking:
      return attribution::SbirsTrackingSource::kStrictTruthAssisted;
    case SbirsTargetState::kSensorLikeTruthAssistedTracking:
      return attribution::SbirsTrackingSource::kSensorLikeTruthAssisted;
    default:
      return attribution::SbirsTrackingSource::kNotApplicable;
  }
}

attribution::SbirsTrackingSource TrackingSourceForMode(config::SbirsTrackingMode mode) {
  switch (mode) {
    case config::SbirsTrackingMode::kEstimated:
      return attribution::SbirsTrackingSource::kEstimated;
    case config::SbirsTrackingMode::kStrictTruthAssisted:
      return attribution::SbirsTrackingSource::kStrictTruthAssisted;
    case config::SbirsTrackingMode::kSensorLikeTruthAssisted:
      return attribution::SbirsTrackingSource::kSensorLikeTruthAssisted;
    default:
      return attribution::SbirsTrackingSource::kNotApplicable;
  }
}

float NormalizeAzimuth(float azimuth_deg) {
  float result = std::fmod(azimuth_deg + 180.0f, 360.0f);
  if (result < 0.0f) {
    result += 360.0f;
  }
  return result - 180.0f;
}

float AzimuthDelta(float lhs_deg, float rhs_deg) { return NormalizeAzimuth(lhs_deg - rhs_deg); }

float PositiveModulo(float value, float period) {
  float result = std::fmod(value, period);
  if (result < 0.0f) {
    result += period;
  }
  return result;
}

// 2026-08 正式变更（ECI 输出）：内部角度量纲保持 deg、方位约定为对称
// (-180, 180]（AzimuthDelta 最短角差语义）；输出边界统一转换为 ECI 极坐标
// 弧度——az ∈ [0, 2π)、el ∈ [-π/2, π/2]（客户契约，见 session_contract.md
// §传感器方位坐标系约定）。el 越界由误差注入引起时钳制到有效域。
float WrapAzimuthPositive(float azimuth_deg) { return PositiveModulo(azimuth_deg, 360.0f); }

float ToEciAzimuthRad(float azimuth_deg) {
  return oneq::common::numerics::DegToRad(WrapAzimuthPositive(azimuth_deg));
}

float ToEciElevationRad(float elevation_deg) {
  const float clamped = std::max(-90.0f, std::min(90.0f, elevation_deg));
  return oneq::common::numerics::DegToRad(clamped);
}

float ScanAzimuth(const config::SbirsMissionConfig& mission, float phase_deg) {
  const float direction =
      mission.scan_direction == config::SbirsScanDirection::kIncreasingAzimuth ? 1.0f : -1.0f;
  return NormalizeAzimuth(mission.scan_start_az_deg + direction * phase_deg);
}

float ScanPhaseForAzimuth(const config::SbirsMissionConfig& mission, float azimuth_deg) {
  if (mission.scan_direction == config::SbirsScanDirection::kIncreasingAzimuth) {
    return PositiveModulo(azimuth_deg - mission.scan_start_az_deg, 360.0f);
  }
  return PositiveModulo(mission.scan_start_az_deg - azimuth_deg, 360.0f);
}

// 2-D 俯仰栅格（阶段 4）：span=0 默认单行模式（行数=1，行中心恒为 scan_center_el_deg，
// 既有行为逐位不变）；span>0 时行数 = 1 + floor(span/step)，行中心 = el_start + row·step。
// 配置校验已保证 span>=0、step>0。
int ScanRowCount(const config::SbirsMissionConfig& mission) {
  if (mission.scan_el_span_deg <= 0.0f) {
    return 1;
  }
  return 1 + static_cast<int>(
                 std::floor(mission.scan_el_span_deg / std::max(1.0e-6f, mission.scan_el_step_deg)));
}

float RowCenterEl(const config::SbirsMissionConfig& mission, int row_index) {
  if (mission.scan_el_span_deg <= 0.0f) {
    return mission.scan_center_el_deg;
  }
  return mission.scan_el_start_deg +
         static_cast<float>(row_index) * std::max(1.0e-6f, mission.scan_el_step_deg);
}

session::SbirsVector3M LosFromAzimuthElevation(float azimuth_deg, float elevation_deg) {
  const double azimuth_rad =
      oneq::common::numerics::DegToRad(static_cast<double>(azimuth_deg));
  const double elevation_rad =
      oneq::common::numerics::DegToRad(static_cast<double>(elevation_deg));
  const double horizontal = std::cos(elevation_rad);
  session::SbirsVector3M los;
  los.x = horizontal * std::cos(azimuth_rad);
  los.y = horizontal * std::sin(azimuth_rad);
  los.z = std::sin(elevation_rad);
  return los;
}

bool InRectangularFov(float target_az_deg, float target_el_deg, float center_az_deg,
                      float center_el_deg, float fov_az_deg, float fov_el_deg) {
  return std::fabs(AzimuthDelta(target_az_deg, center_az_deg)) <= 0.5f * fov_az_deg &&
         std::fabs(target_el_deg - center_el_deg) <= 0.5f * fov_el_deg;
}

SbirsPointingDisturbanceParameters DisturbanceParameters(
    const config::SbirsPointingDisturbanceConfig& config) {
  SbirsPointingDisturbanceParameters parameters;
  parameters.common_attitude_sigma_deg = static_cast<double>(config.common_attitude_sigma_deg);
  parameters.common_attitude_correlation_time_s =
      static_cast<double>(config.common_attitude_correlation_time_s);
  parameters.channel_pointing_sigma_deg = static_cast<double>(config.channel_pointing_sigma_deg);
  parameters.channel_pointing_correlation_time_s =
      static_cast<double>(config.channel_pointing_correlation_time_s);
  parameters.channel_vibration_amplitude_deg =
      static_cast<double>(config.channel_vibration_amplitude_deg);
  parameters.channel_vibration_frequency_hz =
      static_cast<double>(config.channel_vibration_frequency_hz);
  return parameters;
}

bool EffectiveNfovPointing(const SbirsPointingCoordinator& coordinator, int channel_id,
                           const SbirsPointingDisturbanceParameters& parameters,
                           const session::SbirsVector3M& nominal_los,
                           const SbirsBoresightChain& boresight_chain, float static_error_deg,
                           float* azimuth_deg, float* elevation_deg) {
  if (azimuth_deg == nullptr || elevation_deg == nullptr) {
    return false;
  }
  SbirsPointingDisturbanceSample disturbance;
  if (!coordinator.DisturbanceSample(channel_id, parameters, &disturbance)) {
    return false;
  }
  // 阶段 2：实际指向 = 名义 LOS（传感器系 az/el）+ 共模/通道扰动 + 静态 settle 误差，
  // 全部在传感器系叠加（identity 链下与历史逐位一致）；输出为传感器系 az/el。
  float sensor_azimuth_deg = 0.0f;
  float sensor_elevation_deg = 0.0f;
  boresight_chain.SensorAzElOfEciVector(nominal_los, &sensor_azimuth_deg, &sensor_elevation_deg);
  *azimuth_deg = sensor_azimuth_deg +
                 static_cast<float>(disturbance.common.azimuth_deg + disturbance.channel.azimuth_deg) +
                 static_error_deg;
  *elevation_deg = sensor_elevation_deg +
                   static_cast<float>(disturbance.common.elevation_deg + disturbance.channel.elevation_deg);
  return true;
}

bool IsFiniteGaussianState(const tracking::SbirsGaussianState& state) {
  return state.mean.allFinite() && state.covariance.allFinite();
}

bool IsValidTrackingSnapshot(const SbirsPipelineSnapshot& snapshot,
                             const config::SbirsTrackingConfig& tracking_config) {
  if (!std::isfinite(snapshot.scan_phase_deg) || snapshot.scan_row_index < 0 ||
      snapshot.next_detection_id == 0U ||
      snapshot.wfov_measurement_random_state == 0U ||
      snapshot.estimated_measurement_random_state == 0U ||
      snapshot.sensor_like_output_random_state == 0U ||
      !std::isfinite(snapshot.misalignment_yaw_deg) ||
      !std::isfinite(snapshot.misalignment_pitch_deg) ||
      !std::isfinite(snapshot.misalignment_roll_deg)) {
    return false;
  }
  for (const auto& entry : snapshot.cue_predictor.targets) {
    if (!std::isfinite(entry.second.measured_azimuth_deg) ||
        !std::isfinite(entry.second.measured_elevation_deg)) {
      return false;
    }
  }

  std::set<std::uint64_t> estimated_target_ids;
  for (const auto& entry : snapshot.target_states) {
    switch (entry.second) {
      case SbirsTargetState::kUndetected:
      case SbirsTargetState::kWideCandidate:
      case SbirsTargetState::kAwaitingNfovAcquisition:
      case SbirsTargetState::kStrictTruthAssistedTracking:
      case SbirsTargetState::kSensorLikeTruthAssistedTracking:
      case SbirsTargetState::kLost:
        break;
      case SbirsTargetState::kEstimatedTracking:
        estimated_target_ids.insert(entry.first);
        break;
      default:
        return false;
    }
  }
  if (snapshot.filter_states.size() != estimated_target_ids.size() ||
      snapshot.nis_gate_exceeded_counts.size() != estimated_target_ids.size()) {
    return false;
  }
  const std::size_t expected_model_count = tracking_config.imm_model_noise_diff_coeffs.empty()
                                               ? 2U
                                               : tracking_config.imm_model_noise_diff_coeffs.size();
  for (const std::uint64_t target_id : estimated_target_ids) {
    const auto filter = snapshot.filter_states.find(target_id);
    if (filter == snapshot.filter_states.end() || !IsFiniteGaussianState(filter->second) ||
        snapshot.nis_gate_exceeded_counts.count(target_id) == 0U) {
      return false;
    }
    if (tracking_config.estimated_backend != config::SbirsEstimatedTrackingBackend::kImm) {
      continue;
    }
    const auto imm = snapshot.imm_snapshots.find(target_id);
    if (imm == snapshot.imm_snapshots.end() ||
        imm->second.model_states.size() != expected_model_count ||
        static_cast<std::size_t>(imm->second.model_weights.size()) != expected_model_count ||
        !imm->second.model_weights.allFinite()) {
      return false;
    }
    for (std::size_t index = 0U; index < expected_model_count; ++index) {
      const tracking::SbirsImmModelState& model = imm->second.model_states[index];
      if (!IsFiniteGaussianState(model.state) || !std::isfinite(model.weight) ||
          model.weight != imm->second.model_weights(static_cast<Eigen::Index>(index))) {
        return false;
      }
    }
  }
  const std::size_t expected_imm_count =
      tracking_config.estimated_backend == config::SbirsEstimatedTrackingBackend::kImm
          ? estimated_target_ids.size()
          : 0U;
  return snapshot.imm_snapshots.size() == expected_imm_count &&
         snapshot.imm_active == (expected_imm_count != 0U);
}

double ComputeSnr(const config::SbirsInternalExecutionConfig& config,
                  const SbirsEciSceneTarget& target, double range_m, float transmittance) {
  const config::SbirsHardwareConfig& hardware = config.session.hardware;
  const double received_power = foundation::ComputeReceivedPowerW(
      target.radiant_intensity_w_per_sr, range_m, hardware.optical_aperture_m,
      hardware.optical_transmission, transmittance, hardware.detector_quantum_efficiency);
  // 2.8 噪声分解：背景/热/读出三项 RMS 合成；默认全 0 时回退到 NEP 标量。
  const foundation::SbirsNoiseStatistics noise =
      foundation::ComputeBackgroundNoiseStatistics(hardware);
  const double effective_noise = foundation::ResolveEffectiveNoiseW(hardware, noise);
  const double signal_energy =
      std::max(0.0, received_power) * std::max(0.0f, hardware.integration_time_sec);
  return signal_energy / effective_noise;
}

// 阶段 3：由安装失准配置抽取运行期一次常值失准角总量（常值偏置 + 随机微扰）。
// sigma==0 时直接返回 bias（不消耗随机流）；否则用配置种子构造独立流、每轴一次
// N(0,σ) 抽取（3 次 Box-Muller 采样，弃样语义与量测域 SbirsRandomSource 一致）。
// 每次调用用同种子确定性重抽产生同值——pipeline 构造与 ApplyConfig 均调用，
// 配置不变时运行内不变化，保证 replay 可复现与确定性 continuation。
session::SbirsEulerAnglesDeg DrawMisalignmentTotal(
    const config::SbirsMisalignmentModel& misalignment) {
  session::SbirsEulerAnglesDeg total;
  total.yaw_deg = misalignment.bias_deg.yaw_deg;
  total.pitch_deg = misalignment.bias_deg.pitch_deg;
  total.roll_deg = misalignment.bias_deg.roll_deg;
  if (misalignment.random_sigma_deg <= 0.0f) {
    return total;
  }
  foundation::SbirsRandomSource random(misalignment.random_seed);
  const double sigma = static_cast<double>(misalignment.random_sigma_deg);
  total.yaw_deg += sigma * random.NextStandardNormal();
  total.pitch_deg += sigma * random.NextStandardNormal();
  total.roll_deg += sigma * random.NextStandardNormal();
  return total;
}

}  // namespace

SbirsPipeline::SbirsPipeline(const config::SbirsInternalExecutionConfig& config)
    : config_(config),
      misalignment_total_deg_(DrawMisalignmentTotal(config.session.orientation.misalignment)),
      nfov_scheduler_(config.session.policy.scheduler.max_concurrent_nfov_locks),
      pointing_coordinator_(config.session.policy.scheduler.max_concurrent_nfov_locks,
                            config.session.policy.pointing_disturbance.random_seed),
      wfov_measurement_random_source_(DeriveMeasurementSeed(
          config.session.policy.error_model.random_seed, kWfovMeasurementDomain)),
      estimated_measurement_random_source_(DeriveMeasurementSeed(
          config.session.policy.error_model.random_seed, kEstimatedMeasurementDomain)),
      sensor_like_output_random_source_(DeriveMeasurementSeed(
          config.session.policy.error_model.random_seed, kSensorLikeOutputDomain)) {}

void SbirsPipeline::ApplyConfig(const config::SbirsInternalExecutionConfig& config,
                                const runtime::SbirsRuntimeConfigImpact& impact) {
  const float previous_scan_azimuth_deg = ScanAzimuth(config_.session.mission, scan_phase_deg_);
  // 阶段 4：记录旧栅格当前行中心 el，供新栅格重锚行索引。
  const float previous_row_center_el_deg = RowCenterEl(config_.session.mission, scan_row_index_);
  config_ = config;
  // 阶段 3：安装失准为静态配置（不进运行期 patch），每次应用配置时确定性重抽
  // 运行期失准角总量——配置不变时同种子重抽产生同值（行为不变），配置变时即刻生效。
  misalignment_total_deg_ = DrawMisalignmentTotal(config_.session.orientation.misalignment);
  if (impact.scan_sector_changed) {
    const float candidate_phase =
        ScanPhaseForAzimuth(config_.session.mission, previous_scan_azimuth_deg);
    scan_phase_deg_ = config_.session.mission.scan_span_deg == 360.0f ||
                              candidate_phase < config_.session.mission.scan_span_deg
                          ? candidate_phase
                          : 0.0f;
    // 行重锚（阶段 4）：旧行中心 el 映射到新栅格最近行；新栅格为单行模式
    // （span=0）或旧 el 不在新栅格 [el_start, el_start+span] 内时归零行。
    const int new_row_count = ScanRowCount(config_.session.mission);
    scan_row_index_ = 0;
    if (new_row_count > 1 && config_.session.mission.scan_el_span_deg > 0.0f &&
        previous_row_center_el_deg >= config_.session.mission.scan_el_start_deg &&
        previous_row_center_el_deg <= config_.session.mission.scan_el_start_deg +
                                          config_.session.mission.scan_el_span_deg) {
      const double relative_row =
          (static_cast<double>(previous_row_center_el_deg) -
           static_cast<double>(config_.session.mission.scan_el_start_deg)) /
          static_cast<double>(std::max(1.0e-6f, config_.session.mission.scan_el_step_deg));
      const int nearest_row = static_cast<int>(std::floor(relative_row + 0.5));
      scan_row_index_ =
          std::max(0, std::min(new_row_count - 1, nearest_row));
    }
  }
  if (impact.reset_measurement_random_stream) {
    const std::uint32_t seed = config.session.policy.error_model.random_seed;
    wfov_measurement_random_source_ = foundation::SbirsRandomSource(
        DeriveMeasurementSeed(seed, kWfovMeasurementDomain));
    estimated_measurement_random_source_ = foundation::SbirsRandomSource(
        DeriveMeasurementSeed(seed, kEstimatedMeasurementDomain));
    sensor_like_output_random_source_ = foundation::SbirsRandomSource(
        DeriveMeasurementSeed(seed, kSensorLikeOutputDomain));
  }
  if (impact.nfov_channel_count_changed || impact.restart_pointing_disturbance) {
    SbirsNfovScheduler next_scheduler(impact.next_nfov_channel_count);
    SbirsNfovSchedulerSnapshot next_scheduler_snapshot;
    const SbirsNfovSchedulerSnapshot previous_scheduler_snapshot = nfov_scheduler_.Capture();
    for (const auto& entry : previous_scheduler_snapshot.target_to_channel) {
      if (entry.second < impact.next_nfov_channel_count) {
        next_scheduler_snapshot.target_to_channel.insert(entry);
      }
    }
    next_scheduler.Restore(next_scheduler_snapshot);

    SbirsPointingCoordinator next_pointing = pointing_coordinator_;
    std::vector<std::uint64_t> released_target_ids;
    if (next_pointing.Reconfigure(impact.next_nfov_channel_count,
                                  config.session.policy.pointing_disturbance.random_seed,
                                  impact.restart_pointing_disturbance, &released_target_ids)) {
      bool state_is_consistent = true;
      const SbirsPointingCoordinatorSnapshot next_pointing_snapshot = next_pointing.Capture();
      for (const auto& entry : next_scheduler_snapshot.target_to_channel) {
        const std::size_t channel_index = static_cast<std::size_t>(entry.second);
        state_is_consistent =
            state_is_consistent && channel_index < next_pointing_snapshot.channels.size() &&
            next_pointing_snapshot.channels[channel_index].has_bound_target &&
            next_pointing_snapshot.channels[channel_index].target_id == entry.first;
      }
      if (state_is_consistent) {
        nfov_scheduler_ = next_scheduler;
        pointing_coordinator_ = next_pointing;
        for (const std::uint64_t target_id : released_target_ids) {
          target_states_[target_id] = SbirsTargetState::kWideCandidate;
          tracking_coordinator_.ReleaseTarget(target_id);
        }
      }
    }
  }
  if (impact.reset_nis_gate_counts) {
    tracking_coordinator_.ResetNisGateCounts();
  }
  if (impact.reset_nfov_gate_failure_counts) {
    pointing_coordinator_.ResetTrackingGateFailureCounts();
  }
  std::vector<std::uint64_t> released_tracking_targets;
  for (const auto& entry : target_states_) {
    const bool release_for_backend =
        impact.release_estimated_tracks && entry.second == SbirsTargetState::kEstimatedTracking;
    const bool release_for_mode =
        impact.release_incompatible_tracks &&
        (entry.second == SbirsTargetState::kEstimatedTracking ||
         IsTruthTrackingState(entry.second));
    if (release_for_backend || release_for_mode) {
      released_tracking_targets.push_back(entry.first);
    }
  }
  for (const std::uint64_t target_id : released_tracking_targets) {
    target_states_[target_id] = SbirsTargetState::kWideCandidate;
    nfov_scheduler_.Release(target_id);
    pointing_coordinator_.ReleaseTarget(target_id);
    tracking_coordinator_.ReleaseTarget(target_id);
  }
  if (impact.retag_truth_tracks) {
    const SbirsTargetState next_state =
        impact.next_tracking_mode == config::SbirsTrackingMode::kStrictTruthAssisted
            ? SbirsTargetState::kStrictTruthAssistedTracking
            : SbirsTargetState::kSensorLikeTruthAssistedTracking;
    for (auto& entry : target_states_) {
      if (IsTruthTrackingState(entry.second)) {
        entry.second = next_state;
      }
    }
  }
  if (impact.clear_for_inactive || impact.clear_for_wide_search) {
    target_states_.clear();
    tracking_coordinator_.ClearForStandby();
    nfov_scheduler_.Clear();
    pointing_coordinator_.Clear();
    if (impact.clear_for_inactive) {
      cue_predictor_.Clear();
    }
  }
}

SbirsPipelineResult SbirsPipeline::RunCycle(const session::SbirsCycleInput& input) {
  SbirsPipelineResult result;
  const config::SbirsMissionConfig& mission = config_.session.mission;
  const config::SbirsPolicyConfig& policy = config_.session.policy;
  const config::SbirsEnvironmentConfig& environment_config = config_.session.environment;

  // 指向合成链（阶段 2/3）：每周期由卫星姿态（Body->ECI）与安装角（Body->Sensor）
  // 及运行期安装失准角总量构建。默认零姿态 + 零安装角 + 零失准下为恒等变换
  // （IsIdentity），全部门控与输出与历史逐位一致。
  const oneq::foundation::EulerAnglesDeg misalignment_angles_deg{
      misalignment_total_deg_.yaw_deg, misalignment_total_deg_.pitch_deg,
      misalignment_total_deg_.roll_deg};
  const SbirsBoresightChain boresight_chain(input.satellite_attitude_eci_body_deg,
                                            config_.session.orientation.mount_angles_deg,
                                            misalignment_angles_deg);
  // 名义扫描中心（扰动前）合成光轴的 ECI 方位/俯仰：输出参考保持 ECI 极坐标，
  // 不随姿态/安装变化；2-D 栅格（阶段 4）下俯仰为当前行中心 el。
  const auto nominal_scan_eci_angles_rad = [&]() -> std::pair<float, float> {
    const float scan_elevation_deg = RowCenterEl(mission, scan_row_index_);
    if (boresight_chain.IsIdentity()) {
      return std::make_pair(ToEciAzimuthRad(ScanAzimuth(mission, scan_phase_deg_)),
                            ToEciElevationRad(scan_elevation_deg));
    }
    const session::SbirsVector3M eci_los =
        boresight_chain.EciLosOfSensorPointing(ScanAzimuth(mission, scan_phase_deg_),
                                               scan_elevation_deg);
    return std::make_pair(ToEciAzimuthRad(foundation::ComputeAzimuthDeg(eci_los)),
                          ToEciElevationRad(foundation::ComputeElevationDeg(eci_los)));
  };

  if (mission.work_mode == config::SbirsWorkMode::kStandby || !config_.session.sensor_enabled) {
    target_states_.clear();
    tracking_coordinator_.ClearForStandby();
    nfov_scheduler_.Clear();
    pointing_coordinator_.Clear();
    cue_predictor_.Clear();
    const std::pair<float, float> nominal_angles = nominal_scan_eci_angles_rad();
    result.scan_azimuth_rad = nominal_angles.first;
    result.scan_elevation_rad = nominal_angles.second;
    return result;
  }

  result.executed = true;

  // ECI 输出参考系（2026-08 正式变更）：本周期时刻的 GMST 把卫星与目标位置/速度
  // 从 ECEF 旋转到 ECI（J2000 平赤道面），下游 LOS/az/el/遮挡/SNR/EKF 全部在 ECI
  // 中执行；速度含 ω×r 输运项（见 1q/coordinate/inertial_transform.h）。目标侧
  // 旋转结果存入诚实命名的 SbirsEciSceneTarget（position_eci_m 等）。
  // 输入校验已保证 utc_julian_day 正有限；防御性回退 GMST=0 仅防不可达路径。
  double gmst_rad = 0.0;
  if (!oneq::coordinate::TryComputeGmstRad(input.utc_julian_day, &gmst_rad)) {
    gmst_rad = 0.0;
  }
  std::vector<SbirsEciSceneTarget> eci_scene;
  eci_scene.reserve(input.scene.size());
  for (const session::SbirsSceneTarget& target : input.scene) {
    eci_scene.push_back(RotateSceneTargetToEci(target, gmst_rad));
  }
  const oneq::coordinate::EcefPositionM satellite_ecef(
      input.satellite_position_ecef_m.x, input.satellite_position_ecef_m.y,
      input.satellite_position_ecef_m.z);
  oneq::coordinate::EciPositionM satellite_eci;
  if (!oneq::coordinate::TryEcefToEci(satellite_ecef, gmst_rad, &satellite_eci)) {
    satellite_eci = oneq::coordinate::EciPositionM(satellite_ecef.x_m, satellite_ecef.y_m,
                                                   satellite_ecef.z_m);
  }
  const session::SbirsVector3M satellite_position_eci_m{
      satellite_eci.x_m, satellite_eci.y_m, satellite_eci.z_m};
  // 卫星速度旋入 ECI（与目标侧 RotateSceneTargetToEci 同法，含 ω×r 输运项）；
  // 旋换失败走与位置相同的防御性回退（保留原分量）。
  const oneq::coordinate::EcefVelocityMps satellite_velocity_ecef(
      input.satellite_velocity_ecef_m_per_s.x, input.satellite_velocity_ecef_m_per_s.y,
      input.satellite_velocity_ecef_m_per_s.z);
  oneq::coordinate::EciVelocityMps satellite_velocity_eci;
  if (!oneq::coordinate::TryEcefVelocityToEci(satellite_ecef, satellite_velocity_ecef, gmst_rad,
                                              &satellite_velocity_eci)) {
    satellite_velocity_eci = oneq::coordinate::EciVelocityMps(
        satellite_velocity_ecef.x_mps, satellite_velocity_ecef.y_mps,
        satellite_velocity_ecef.z_mps);
  }
  const session::SbirsVector3M satellite_velocity_eci_m{
      satellite_velocity_eci.x_mps, satellite_velocity_eci.y_mps,
      satellite_velocity_eci.z_mps};

  // 规则 13a：周期执行摘要所需四类门控排除计数（按目标循环内累加）。
  std::size_t excluded_occulted = 0U;
  std::size_t excluded_out_of_range = 0U;
  std::size_t excluded_out_of_wfov = 0U;
  std::size_t excluded_snr_below = 0U;
  const auto log_cycle_summary = [&]() {
    // 中译：周期执行摘要（周期号、扫描方位角、探测成功数/目标总数、记录总数、
    //       四类门控排除计数）。
    // 标识：规则 13a 周期级执行摘要日志——每周期探测概况与几何/SNR 排除分布，
    //       供宏观核对与"零探测"排查；仅人读，不用于状态判断（规则 3）。
    //       detected 只统计 record.detected == true 的成功探测；records 为本周期
    //       全部检测记录数（含捕获失败/门失败等未过门限记录），两数分离避免把
    //       失败记录误读为探测（2026-08-08 修正）。
    const std::size_t detected_count = static_cast<std::size_t>(std::count_if(
        result.detections.begin(), result.detections.end(),
        [](const SbirsPipelineDetection& detection) { return detection.record.detected; }));
    PROJECT_LOG_INFO("[SbirsPipeline] cycle_index={} scan_az_rad={:.4f} detected={}/{} records={} "
                     "excluded={{occulted={} range={} wfov={} snr={}}}",
                     input.cycle_index, result.scan_azimuth_rad, detected_count,
                     input.scene.size(), result.detections.size(), excluded_occulted,
                     excluded_out_of_range, excluded_out_of_wfov, excluded_snr_below);
  };

  const SbirsPointingDisturbanceParameters disturbance_parameters =
      DisturbanceParameters(policy.pointing_disturbance);
  if (!pointing_coordinator_.AdvanceDisturbance(static_cast<double>(input.dt_sec),
                                                disturbance_parameters)) {
    const std::pair<float, float> nominal_angles = nominal_scan_eci_angles_rad();
    result.scan_azimuth_rad = nominal_angles.first;
    result.scan_elevation_rad = nominal_angles.second;
    log_cycle_summary();
    return result;
  }
  SbirsPointingDisturbanceSample frame_disturbance;
  if (!pointing_coordinator_.DisturbanceSample(0, disturbance_parameters, &frame_disturbance)) {
    const std::pair<float, float> nominal_angles = nominal_scan_eci_angles_rad();
    result.scan_azimuth_rad = nominal_angles.first;
    result.scan_elevation_rad = nominal_angles.second;
    log_cycle_summary();
    return result;
  }

  // 2-D 栅格推进（阶段 4）：行内方位相位推进与既有完全同式（float 域
  // phase + rate·dt 后 PositiveModulo 到 [0, span)，逐位不变）；相位跨过 span 时
  // 行索引按 floor(advance/span) 步进并回绕（row_count=1 单行模式下 row_advance
  // 恒 0，行为与既有逐位一致）。
  const int row_count = ScanRowCount(mission);
  const float advanced_phase =
      scan_phase_deg_ + mission.scan_rate_deg_per_sec * std::max(0.0f, input.dt_sec);
  const int row_advance =
      static_cast<int>(std::floor(static_cast<double>(advanced_phase) /
                                  static_cast<double>(mission.scan_span_deg)));
  scan_phase_deg_ = PositiveModulo(advanced_phase, mission.scan_span_deg);
  if (row_advance > 0 && row_count > 1) {
    scan_row_index_ = (scan_row_index_ + row_advance) % row_count;
  }
  const float scan_azimuth_deg = ScanAzimuth(mission, scan_phase_deg_);
  const std::pair<float, float> nominal_angles = nominal_scan_eci_angles_rad();
  result.scan_azimuth_rad = nominal_angles.first;
  result.scan_elevation_rad = nominal_angles.second;
  // 实际扫描中心（传感器系，扰动前相位扫描角 + 共模扰动，随后按扫描限位钳制）：
  // 体稳定 = 扫描参数直接为传感器系角度；惯性稳定 = 扫描参数为 ECI 参考方向，经链
  // 反解到传感器系（物理上保持惯性方向稳定）。实际光轴足迹 = 链旋转该传感器系指向。
  float scan_azimuth_sensor_deg = scan_azimuth_deg;
  float scan_elevation_sensor_deg = RowCenterEl(mission, scan_row_index_);
  if (config_.session.orientation.stabilization_mode ==
      config::SbirsStabilizationMode::kInertialStabilized) {
    const session::SbirsVector3M desired_eci_los =
        LosFromAzimuthElevation(scan_azimuth_deg, scan_elevation_sensor_deg);
    const double desired_norm = foundation::Norm(desired_eci_los);
    session::SbirsVector3M desired_unit;
    desired_unit.x = desired_norm > 0.0 ? desired_eci_los.x / desired_norm : desired_eci_los.x;
    desired_unit.y = desired_norm > 0.0 ? desired_eci_los.y / desired_norm : desired_eci_los.y;
    desired_unit.z = desired_norm > 0.0 ? desired_eci_los.z / desired_norm : desired_eci_los.z;
    boresight_chain.SensorPointingForDesiredEciLos(desired_unit, &scan_azimuth_sensor_deg,
                                                   &scan_elevation_sensor_deg);
  }
  // 名义（扰动前）扫描中心传感器系角度：NFOV 通道初始 LOS 用（历史语义：Reserve 初值
  // 取扰动前扫描指向，不携带本周期共模扰动）。
  const float nominal_scan_azimuth_sensor_deg = scan_azimuth_sensor_deg;
  const float nominal_scan_elevation_sensor_deg = scan_elevation_sensor_deg;
  scan_azimuth_sensor_deg += static_cast<float>(frame_disturbance.common.azimuth_deg);
  scan_elevation_sensor_deg += static_cast<float>(frame_disturbance.common.elevation_deg);
  SbirsBoresightChain::ClampToScanLimits(config_.session.orientation.sensor_scan_limits_deg,
                                         &scan_azimuth_sensor_deg, &scan_elevation_sensor_deg);
  const float actual_scan_azimuth_sensor_deg = scan_azimuth_sensor_deg;
  const float actual_scan_elevation_sensor_deg = scan_elevation_sensor_deg;

  const float transmittance = environment::ResolveEffectiveTransmittance(environment_config);
  // 门内归因目标无关量（仅依赖硬件/门限配置，循环外计算一次）：有效噪声与达标所需签名 =
  // wide_min·噪声/积分时间（见 ClassifyWfovSnrExclusionCause）。
  const config::SbirsHardwareConfig& hw = config_.session.hardware;
  const double effective_noise_w =
      foundation::ResolveEffectiveNoiseW(hw, foundation::ComputeBackgroundNoiseStatistics(hw));
  const double snr_signature_required =
      policy.detection.wide_min_snr_linear * effective_noise_w /
      std::max(0.0f, hw.integration_time_sec);
  std::vector<SbirsCandidate> candidates;
  std::set<std::uint64_t> present_target_ids;
  for (const session::SbirsSceneTarget& target : input.scene) {
    present_target_ids.insert(target.target_id);
  }
  for (std::map<std::uint64_t, SbirsTargetState>::value_type& target_state : target_states_) {
    if (present_target_ids.count(target_state.first) == 0U) {
      target_state.second = SbirsTargetState::kLost;
      nfov_scheduler_.Release(target_state.first);
      pointing_coordinator_.ReleaseTarget(target_state.first);
      cue_predictor_.Release(target_state.first);
      tracking_coordinator_.ReleaseTarget(target_state.first);
    }
  }

  for (std::size_t target_idx = 0U; target_idx < eci_scene.size(); ++target_idx) {
    // ECI 场景副本（真值已在周期入口旋转到 ECI，字段名即 ECI 语义）。
    const SbirsEciSceneTarget& target = eci_scene[target_idx];
    if (!target.active) {
      target_states_[target.target_id] = SbirsTargetState::kLost;
      nfov_scheduler_.Release(target.target_id);
      pointing_coordinator_.ReleaseTarget(target.target_id);
      cue_predictor_.Release(target.target_id);
      tracking_coordinator_.ReleaseTarget(target.target_id);
      continue;
    }

    if (foundation::IsEarthOcculted(satellite_position_eci_m, target.position_eci_m,
                                    kEarthRadiusM)) {
      // 规则 13b：遮挡排除 → kInfo 诊断（不属于三写，仅承载排查信息）。
      // 具体门（遮挡判定本身可定位）：cause 保持 kNone，遮挡余量（负值 = 深度）进 message。
      const double occultation_margin_m = foundation::ComputeEarthOccultationMarginM(
          satellite_position_eci_m, target.position_eci_m, kEarthRadiusM);
      result.issues.push_back(MakeExclusionIssue(
          session::codes::kTargetOcculted,
          "target_id=" + std::to_string(target.target_id) +
              "; LOS from satellite occulted by Earth; occultation_margin_m=" +
              std::to_string(static_cast<std::int64_t>(occultation_margin_m)),
          session::SbirsIssueCause::kNone,
          static_cast<std::ptrdiff_t>(target_idx)));
      ++excluded_occulted;
      target_states_[target.target_id] = SbirsTargetState::kUndetected;
      nfov_scheduler_.Release(target.target_id);
      pointing_coordinator_.ReleaseTarget(target.target_id);
      cue_predictor_.Release(target.target_id);
      tracking_coordinator_.ReleaseTarget(target.target_id);
      continue;
    }

    const session::SbirsVector3M los =
        foundation::Subtract(target.position_eci_m, satellite_position_eci_m);
    const double range_m = foundation::Norm(los);
    if (range_m < mission.min_range_m || range_m > mission.max_range_m) {
      // 规则 13b：距离门排除 → kInfo 诊断（不属于三写，仅承载排查信息）。
      // 具体门（距离带本身可定位）：cause 保持 kNone，距带边余量进 message。
      const double range_margin_m =
          range_m < mission.min_range_m ? mission.min_range_m - range_m : range_m - mission.max_range_m;
      result.issues.push_back(MakeExclusionIssue(
          session::codes::kTargetOutOfRange,
          "target_id=" + std::to_string(target.target_id) + "; range_m=" +
              std::to_string(static_cast<std::int64_t>(range_m)) + " outside [" +
              std::to_string(static_cast<std::int64_t>(mission.min_range_m)) + "," +
              std::to_string(static_cast<std::int64_t>(mission.max_range_m)) + "] range_margin_m=" +
              std::to_string(static_cast<std::int64_t>(range_margin_m)),
          session::SbirsIssueCause::kNone,
          static_cast<std::ptrdiff_t>(target_idx)));
      ++excluded_out_of_range;
      target_states_[target.target_id] = SbirsTargetState::kUndetected;
      nfov_scheduler_.Release(target.target_id);
      pointing_coordinator_.ReleaseTarget(target.target_id);
      cue_predictor_.Release(target.target_id);
      tracking_coordinator_.ReleaseTarget(target.target_id);
      continue;
    }

    const float azimuth_deg = foundation::ComputeAzimuthDeg(los);
    const float elevation_deg = foundation::ComputeElevationDeg(los);
    // 目标 ECI 视线旋入传感器系（指向合成链）：WFOV 门与越界诊断改在传感器系执行
    //（输出 az/el 仍保持 ECI 参考）。identity 链下与传感器系差值逐位一致。
    float sensor_azimuth_deg = 0.0f;
    float sensor_elevation_deg = 0.0f;
    boresight_chain.SensorAzElOfEciVector(los, &sensor_azimuth_deg, &sensor_elevation_deg);
    const double snr = ComputeSnr(config_, target, range_m, transmittance);
    // 当前时刻最大探测距离（WFOV 门限反解）：随本周期 τ_eff/噪声快照与目标辐射强度
    // 变化，进归属层诊断（不进 raw output）；SNR 门失败目标写入 issue 消息。
    const double max_detection_range_m = foundation::ComputeMaxDetectionRangeM(
        target.radiant_intensity_w_per_sr, hw.optical_aperture_m, hw.optical_transmission,
        transmittance, hw.detector_quantum_efficiency, hw.integration_time_sec,
        effective_noise_w, policy.detection.wide_min_snr_linear);

    // 相对视线角速度：用于动态滞后误差与 R 矩阵（design 2.10）。相对速度 =
    // v_target（未提供时取 0）− v_satellite；卫星速度必填，因此目标静止时 ω 不为 0
    // （卫星运动本身扫过视场）。
    session::SbirsVector3M relative_velocity_eci_m_per_s{
        -satellite_velocity_eci_m.x, -satellite_velocity_eci_m.y, -satellite_velocity_eci_m.z};
    if (target.has_velocity_eci_m_per_s) {
      relative_velocity_eci_m_per_s.x += target.velocity_eci_m_per_s.x;
      relative_velocity_eci_m_per_s.y += target.velocity_eci_m_per_s.y;
      relative_velocity_eci_m_per_s.z += target.velocity_eci_m_per_s.z;
    }
    const float omega_deg_per_sec_cached = foundation::ComputeRelativeAngularRateDegPerSec(
        los, relative_velocity_eci_m_per_s);

    const bool in_wfov = InRectangularFov(sensor_azimuth_deg, sensor_elevation_deg,
                                          actual_scan_azimuth_sensor_deg,
                                          actual_scan_elevation_sensor_deg, mission.wide_field_fov_az_deg,
                                          mission.wide_field_fov_el_deg);

    const SbirsTargetState state = target_states_[target.target_id];
    const bool is_locked = nfov_scheduler_.IsLocked(target.target_id) &&
                           (state == SbirsTargetState::kStrictTruthAssistedTracking ||
                            state == SbirsTargetState::kSensorLikeTruthAssistedTracking ||
                            state == SbirsTargetState::kEstimatedTracking);
    if (is_locked) {
      const int channel_id = nfov_scheduler_.ChannelOf(target.target_id);
      const bool estimated_tracking = state == SbirsTargetState::kEstimatedTracking;
      float command_azimuth_deg = azimuth_deg;
      float command_elevation_deg = elevation_deg;
      if (estimated_tracking) {
        const SbirsTrackingPredictionResult prediction = tracking_coordinator_.PredictTarget(
            target.target_id, policy, input.dt_sec, satellite_position_eci_m);
        command_azimuth_deg = prediction.output_azimuth_deg;
        command_elevation_deg = prediction.output_elevation_deg;
      }
      SbirsPointingActuatorConfig tracking_pointing_config;
      tracking_pointing_config.max_slew_rate_deg_per_sec = mission.narrow_pointing_max_slew_rate_deg_per_sec;
      tracking_pointing_config.settle_tolerance_deg = mission.narrow_pointing_settle_tolerance_deg;
      // NFOV 命令（ECI az/el）旋入传感器系并限位钳制，再经链合成 ECI 单位向量驱动
      // actuator（actuator 限速转向在 ECI 单位向量域，参考系无关，快照不变）。
      float sensor_command_azimuth_deg = 0.0f;
      float sensor_command_elevation_deg = 0.0f;
      boresight_chain.SensorAzElOfEciVector(
          LosFromAzimuthElevation(command_azimuth_deg, command_elevation_deg),
          &sensor_command_azimuth_deg, &sensor_command_elevation_deg);
      SbirsBoresightChain::ClampToScanLimits(config_.session.orientation.sensor_scan_limits_deg,
                                             &sensor_command_azimuth_deg,
                                             &sensor_command_elevation_deg);
      const SbirsPointingAdvanceResult pointing_result = pointing_coordinator_.AdvanceTracking(
          channel_id, target.target_id,
          boresight_chain.EciLosOfSensorPointing(sensor_command_azimuth_deg,
                                                 sensor_command_elevation_deg),
          input.dt_sec, tracking_pointing_config);
      float actual_pointing_azimuth_deg = 0.0f;
      float actual_pointing_elevation_deg = 0.0f;
      const bool pointing_available = EffectiveNfovPointing(
          pointing_coordinator_, channel_id, disturbance_parameters, pointing_result.current_los,
          boresight_chain, mission.narrow_pointing_settle_error_deg, &actual_pointing_azimuth_deg,
          &actual_pointing_elevation_deg);
      const bool geometry_gate_passed =
          pointing_result.status != SbirsPointingAdvanceStatus::kRejected && pointing_available &&
          InRectangularFov(sensor_azimuth_deg, sensor_elevation_deg, actual_pointing_azimuth_deg,
                           actual_pointing_elevation_deg, mission.narrow_field_fov_az_deg,
                           mission.narrow_field_fov_el_deg);
      const bool snr_gate_passed = snr >= policy.detection.narrow_min_snr_linear;
      const bool tracking_gate_passed = geometry_gate_passed && snr_gate_passed;
      const unsigned int gate_failure_count = pointing_coordinator_.RecordTrackingGateResult(
          target.target_id, tracking_gate_passed);
      const bool lost_due_to_tracking_gate =
          !tracking_gate_passed &&
          gate_failure_count >= policy.tracking.nfov_tracking_gate_loss_cycles;

      SbirsPipelineDetection detection;
      detection.record.detection_id = next_detection_id_++;
      detection.record.azimuth_rad = ToEciAzimuthRad(command_azimuth_deg);
      detection.record.elevation_rad = ToEciElevationRad(command_elevation_deg);
      detection.record.infrared_snr_linear = static_cast<float>(snr);
      detection.record.observation_stage = output::SbirsObservationStage::kNarrowFieldTrack;
      detection.record.detected = tracking_gate_passed;
      detection.attribution.detection_id = detection.record.detection_id;
      detection.attribution.target_id = target.target_id;
      detection.attribution.target_name = target.target_name;
      detection.attribution.estimated_range_m = static_cast<float>(range_m);
      detection.attribution.max_detection_range_m = static_cast<float>(max_detection_range_m);
      detection.attribution.tracking_source = TrackingSourceForState(state);
      detection.attribution.nfov_channel_id = channel_id;
      detection.attribution.has_nfov_tracking_diagnostics = true;
      detection.attribution.nfov_pointing_error_deg = foundation::AngularSeparationDeg(
          azimuth_deg, elevation_deg, actual_pointing_azimuth_deg,
          actual_pointing_elevation_deg);
      detection.attribution.nfov_geometry_gate_passed = geometry_gate_passed;
      detection.attribution.nfov_snr_gate_passed = snr_gate_passed;
      detection.attribution.nfov_tracking_gate_failure_count = gate_failure_count;
      detection.attribution.nfov_tracking_coasting =
          !tracking_gate_passed && !lost_due_to_tracking_gate;

      bool lost_due_to_estimation_nis = false;
      if (tracking_gate_passed && estimated_tracking) {
        const SbirsTrackingUpdateResult tracking_result = tracking_coordinator_.CorrectTarget(
            target.target_id, policy, &estimated_measurement_random_source_, azimuth_deg,
            elevation_deg, range_m,
            omega_deg_per_sec_cached, satellite_position_eci_m);
        detection.record.azimuth_rad = ToEciAzimuthRad(tracking_result.output_azimuth_deg);
        detection.record.elevation_rad = ToEciElevationRad(tracking_result.output_elevation_deg);
        detection.attribution.has_estimation_nis = tracking_result.has_estimation_nis;
        detection.attribution.estimation_nis = tracking_result.estimation_nis;
        detection.attribution.estimation_nis_gate_exceeded =
            tracking_result.estimation_nis_gate_exceeded;
        lost_due_to_estimation_nis = tracking_result.lost_due_to_estimation_nis;
        detection.record.detected = !lost_due_to_estimation_nis;
      } else if (tracking_gate_passed &&
                 state == SbirsTargetState::kSensorLikeTruthAssistedTracking) {
        const foundation::SbirsErrorBearing bearing = foundation::ApplyAngularErrorModel(
            policy.error_model, &sensor_like_output_random_source_, azimuth_deg, elevation_deg,
            range_m, omega_deg_per_sec_cached);
        detection.record.azimuth_rad = ToEciAzimuthRad(bearing.azimuth_deg);
        detection.record.elevation_rad = ToEciElevationRad(bearing.elevation_deg);
        detection.attribution.estimated_range_m = static_cast<float>(bearing.range_m);
      } else if (!tracking_gate_passed && estimated_tracking) {
        tracking_coordinator_.MarkMeasurementUnavailable(target.target_id);
      }
      if (lost_due_to_estimation_nis) {
        detection.attribution.capture_failure_reason =
            attribution::SbirsCaptureFailureReason::kEstimationNisGateLost;
        target_states_[target.target_id] = SbirsTargetState::kWideCandidate;
        nfov_scheduler_.Release(target.target_id);
        pointing_coordinator_.ReleaseTarget(target.target_id);
        tracking_coordinator_.ReleaseTarget(target.target_id);
      } else if (lost_due_to_tracking_gate) {
        detection.attribution.capture_failure_reason =
            attribution::SbirsCaptureFailureReason::kNfovTrackingGateLost;
        target_states_[target.target_id] = SbirsTargetState::kWideCandidate;
        nfov_scheduler_.Release(target.target_id);
        pointing_coordinator_.ReleaseTarget(target.target_id);
        tracking_coordinator_.ReleaseTarget(target.target_id);
      }
      result.detections.push_back(detection);
      continue;
    }

    if (!in_wfov) {
      // 规则 13b：视场排除 → kInfo 诊断（不属于三写，仅承载排查信息）。
      // 门内归因：视场门按越界轴细分（az/el/both），并补相对扫描中心的差值
      //（与 InRectangularFov 同基准、同参考系：传感器系；半视场为门限）。
      const float az_delta_deg =
          std::fabs(AzimuthDelta(sensor_azimuth_deg, actual_scan_azimuth_sensor_deg));
      const float el_delta_deg = std::fabs(sensor_elevation_deg - actual_scan_elevation_sensor_deg);
      const float half_wfov_az_deg = 0.5f * mission.wide_field_fov_az_deg;
      const float half_wfov_el_deg = 0.5f * mission.wide_field_fov_el_deg;
      const bool az_out = az_delta_deg > half_wfov_az_deg;
      const bool el_out = el_delta_deg > half_wfov_el_deg;
      const session::SbirsIssueCause wfov_cause =
          az_out && el_out
              ? session::SbirsIssueCause::kBothAxesOutside
              : (az_out ? session::SbirsIssueCause::kAzOutside
                        : session::SbirsIssueCause::kElOutside);
      result.issues.push_back(MakeExclusionIssue(
          session::codes::kTargetOutOfWfov,
          "target_id=" + std::to_string(target.target_id) + "; sensor_az/el (" +
              FormatFloat(sensor_azimuth_deg) + "," + FormatFloat(sensor_elevation_deg) +
              ") outside scan center (" + FormatFloat(actual_scan_azimuth_sensor_deg) + "," +
              FormatFloat(actual_scan_elevation_sensor_deg) + ") fov " +
              FormatFloat(mission.wide_field_fov_az_deg) + "x" +
              FormatFloat(mission.wide_field_fov_el_deg) + " az_delta_deg=" +
              FormatFloat(az_delta_deg) + " el_delta_deg=" + FormatFloat(el_delta_deg),
          wfov_cause,
          static_cast<std::ptrdiff_t>(target_idx)));
      ++excluded_out_of_wfov;
      target_states_[target.target_id] = SbirsTargetState::kUndetected;
      nfov_scheduler_.Release(target.target_id);
      pointing_coordinator_.ReleaseTarget(target.target_id);
      cue_predictor_.Release(target.target_id);
      tracking_coordinator_.ReleaseTarget(target.target_id);
      continue;
    }
    if (snr < policy.detection.wide_min_snr_linear) {
      // 规则 13b：WFOV SNR 门排除 → kInfo 诊断（不属于三写，仅承载排查信息）。
      // 门内归因：SNR 门为聚合门，反事实判定主因（见 ClassifyWfovSnrExclusionCause；
      // signature_required 已在循环外计算）。
      const double signature_actual = std::max(0.0, target.radiant_intensity_w_per_sr);
      const session::SbirsIssueCause snr_cause = ClassifyWfovSnrExclusionCause(
          range_m, transmittance, signature_actual, snr_signature_required);
      result.issues.push_back(MakeExclusionIssue(
          session::codes::kTargetSnrBelowThreshold,
          "target_id=" + std::to_string(target.target_id) +
              "; snr_linear=" + FormatSnr(snr) + " below wide_min=" +
              FormatSnr(policy.detection.wide_min_snr_linear) + " range_m=" +
              std::to_string(static_cast<std::int64_t>(range_m)) + " transmittance=" +
              FormatSnr(transmittance) + " d_max_m=" +
              std::to_string(static_cast<std::int64_t>(max_detection_range_m)),
          snr_cause,
          static_cast<std::ptrdiff_t>(target_idx)));
      ++excluded_snr_below;
      target_states_[target.target_id] = SbirsTargetState::kUndetected;
      nfov_scheduler_.Release(target.target_id);
      pointing_coordinator_.ReleaseTarget(target.target_id);
      cue_predictor_.Release(target.target_id);
      tracking_coordinator_.ReleaseTarget(target.target_id);
      continue;
    }

    SbirsCandidate candidate;
    candidate.target = &target;
    candidate.azimuth_deg = azimuth_deg;
    candidate.elevation_deg = elevation_deg;
    candidate.range_m = range_m;  // 真值距离：调度优先级用（design 2.6）
    candidate.max_detection_range_m = max_detection_range_m;
    // 2.10 WFOV 带误差位置：施加 5 类物理误差（高斯随机 + 折射 + 滞后）。
    // 复用上方已算的目标角速度 omega_deg_per_sec_cached。
    const foundation::SbirsErrorBearing bearing = foundation::ApplyAngularErrorModel(
        policy.error_model, &wfov_measurement_random_source_, azimuth_deg, elevation_deg, range_m,
        /*relative_angular_rate_deg_per_sec=*/omega_deg_per_sec_cached);
    candidate.measured_azimuth_deg = bearing.azimuth_deg;
    candidate.measured_elevation_deg = bearing.elevation_deg;
    candidate.measured_range_m = bearing.range_m;
    candidate.relative_angular_rate_deg_per_sec = omega_deg_per_sec_cached;
    const SbirsCuePrediction cue_prediction =
        cue_predictor_.Update(target.target_id, bearing.azimuth_deg, bearing.elevation_deg,
                              input.dt_sec, mission.narrow_cue_latency_s);
    candidate.command_azimuth_deg = cue_prediction.command_azimuth_deg;
    candidate.command_elevation_deg = cue_prediction.command_elevation_deg;
    // cue 延迟外推：narrow_cue_latency_s 期间目标与卫星都继续运动，真值 az/el 需按
    // 延迟后相对几何重算：(p_target + v_target·τ) − (p_satellite + v_satellite·τ)。
    const float cue_latency_s = mission.narrow_cue_latency_s;
    if (cue_latency_s > 0.0f) {
      session::SbirsVector3M predicted_position;
      predicted_position.x = target.position_eci_m.x;
      predicted_position.y = target.position_eci_m.y;
      predicted_position.z = target.position_eci_m.z;
      if (target.has_velocity_eci_m_per_s) {
        predicted_position.x += target.velocity_eci_m_per_s.x * cue_latency_s;
        predicted_position.y += target.velocity_eci_m_per_s.y * cue_latency_s;
        predicted_position.z += target.velocity_eci_m_per_s.z * cue_latency_s;
      }
      session::SbirsVector3M predicted_satellite_position;
      predicted_satellite_position.x =
          satellite_position_eci_m.x + satellite_velocity_eci_m.x * cue_latency_s;
      predicted_satellite_position.y =
          satellite_position_eci_m.y + satellite_velocity_eci_m.y * cue_latency_s;
      predicted_satellite_position.z =
          satellite_position_eci_m.z + satellite_velocity_eci_m.z * cue_latency_s;
      const session::SbirsVector3M predicted_los =
          foundation::Subtract(predicted_position, predicted_satellite_position);
      candidate.delayed_truth_azimuth_deg = foundation::ComputeAzimuthDeg(predicted_los);
      candidate.delayed_truth_elevation_deg = foundation::ComputeElevationDeg(predicted_los);
    } else {
      candidate.delayed_truth_azimuth_deg = azimuth_deg;
      candidate.delayed_truth_elevation_deg = elevation_deg;
    }
    candidate.snr = snr;
    candidates.push_back(candidate);
    if (target_states_[target.target_id] != SbirsTargetState::kAwaitingNfovAcquisition) {
      target_states_[target.target_id] = SbirsTargetState::kWideCandidate;
    }
  }

  if (mission.work_mode == config::SbirsWorkMode::kWideSearch) {
    for (const SbirsCandidate& candidate : candidates) {
      SbirsPipelineDetection detection;
      detection.record.detection_id = next_detection_id_++;
      detection.record.azimuth_rad = ToEciAzimuthRad(candidate.measured_azimuth_deg);
      detection.record.elevation_rad = ToEciElevationRad(candidate.measured_elevation_deg);
      detection.record.infrared_snr_linear = static_cast<float>(candidate.snr);
      detection.record.observation_stage = output::SbirsObservationStage::kWideFieldSearch;
      detection.record.detected = true;
      detection.attribution.detection_id = detection.record.detection_id;
      detection.attribution.target_id = candidate.target->target_id;
      detection.attribution.target_name = candidate.target->target_name;
      detection.attribution.estimated_range_m = static_cast<float>(candidate.range_m);
      detection.attribution.max_detection_range_m =
          static_cast<float>(candidate.max_detection_range_m);
      detection.attribution.nfov_channel_id = -1;
      result.detections.push_back(detection);
    }
    log_cycle_summary();
    return result;
  }

  SbirsPointingActuatorConfig pointing_config;
  pointing_config.max_slew_rate_deg_per_sec = mission.narrow_pointing_max_slew_rate_deg_per_sec;
  pointing_config.settle_tolerance_deg = mission.narrow_pointing_settle_tolerance_deg;
  std::set<std::uint64_t> processed_target_ids;
  std::set<std::uint64_t> blocked_target_ids;

  const auto append_wfov_detection = [&](const SbirsCandidate& candidate, int channel_id) {
    SbirsPipelineDetection detection;
    detection.record.detection_id = next_detection_id_++;
    detection.record.azimuth_rad = ToEciAzimuthRad(candidate.measured_azimuth_deg);
    detection.record.elevation_rad = ToEciElevationRad(candidate.measured_elevation_deg);
    detection.record.infrared_snr_linear = static_cast<float>(candidate.snr);
    detection.record.observation_stage = output::SbirsObservationStage::kWideFieldSearch;
    detection.record.detected = true;
    detection.attribution.detection_id = detection.record.detection_id;
    detection.attribution.target_id = candidate.target->target_id;
    detection.attribution.target_name = candidate.target->target_name;
    detection.attribution.estimated_range_m = static_cast<float>(candidate.range_m);
    detection.attribution.nfov_channel_id = channel_id;
    result.detections.push_back(detection);
  };
  const auto append_acquisition_failure = [&](const SbirsCandidate& candidate, int channel_id,
                                              attribution::SbirsCaptureFailureReason reason) {
    SbirsPipelineDetection detection;
    detection.record.detection_id = next_detection_id_++;
    detection.record.azimuth_rad = ToEciAzimuthRad(candidate.measured_azimuth_deg);
    detection.record.elevation_rad = ToEciElevationRad(candidate.measured_elevation_deg);
    detection.record.infrared_snr_linear = static_cast<float>(candidate.snr);
    detection.record.observation_stage = output::SbirsObservationStage::kNarrowFieldAcquisition;
    detection.record.detected = false;
    detection.attribution.detection_id = detection.record.detection_id;
    detection.attribution.target_id = candidate.target->target_id;
    detection.attribution.target_name = candidate.target->target_name;
    detection.attribution.estimated_range_m = static_cast<float>(candidate.range_m);
    detection.attribution.nfov_channel_id = channel_id;
    detection.attribution.capture_failure_reason = reason;
    result.detections.push_back(detection);
  };
  const auto advance_pointing = [&](const SbirsCandidate& selected, int channel_id) {
    const std::uint64_t target_id = selected.target->target_id;
    // NFOV 命令（cue 预测 ECI az/el）旋入传感器系并限位钳制，再经链合成 ECI 单位向量
    // 驱动 actuator；限位够不到时 actuator 停在限位边缘（AR 同款静默钳制语义）。
    float sensor_command_azimuth_deg = 0.0f;
    float sensor_command_elevation_deg = 0.0f;
    boresight_chain.SensorAzElOfEciVector(
        LosFromAzimuthElevation(selected.command_azimuth_deg, selected.command_elevation_deg),
        &sensor_command_azimuth_deg, &sensor_command_elevation_deg);
    SbirsBoresightChain::ClampToScanLimits(config_.session.orientation.sensor_scan_limits_deg,
                                           &sensor_command_azimuth_deg,
                                           &sensor_command_elevation_deg);
    const SbirsPointingAdvanceResult pointing_result = pointing_coordinator_.Advance(
        channel_id, target_id,
        boresight_chain.EciLosOfSensorPointing(sensor_command_azimuth_deg,
                                               sensor_command_elevation_deg),
        input.dt_sec, pointing_config);
    processed_target_ids.insert(target_id);
    if (pointing_result.status == SbirsPointingAdvanceStatus::kSlewing) {
      append_wfov_detection(selected, channel_id);
      return;
    }
    if (pointing_result.status == SbirsPointingAdvanceStatus::kTimedOut) {
      nfov_scheduler_.Release(target_id);
      target_states_[target_id] = SbirsTargetState::kWideCandidate;
      blocked_target_ids.insert(target_id);
      append_acquisition_failure(selected, channel_id,
                                 attribution::SbirsCaptureFailureReason::kNfovPointingTimeout);
      return;
    }
    if (pointing_result.status == SbirsPointingAdvanceStatus::kRejected) {
      nfov_scheduler_.Release(target_id);
      pointing_coordinator_.ReleaseTarget(target_id);
      target_states_[target_id] = SbirsTargetState::kWideCandidate;
      blocked_target_ids.insert(target_id);
      append_acquisition_failure(selected, channel_id,
                                 attribution::SbirsCaptureFailureReason::kNfovAcquisitionFailed);
      return;
    }

    SbirsNfovAcquisitionRequest acquisition_request;
    // 首捕窗口判定改在传感器系：delayed truth（延迟真值 ECI los）经链转换；
    // 命令 = 实际指向（名义 LOS 传感器系 + 扰动）的传感器系 az/el。
    boresight_chain.SensorAzElOfEciVector(
        LosFromAzimuthElevation(selected.delayed_truth_azimuth_deg,
                                selected.delayed_truth_elevation_deg),
        &acquisition_request.delayed_truth_azimuth_deg,
        &acquisition_request.delayed_truth_elevation_deg);
    if (!EffectiveNfovPointing(pointing_coordinator_, channel_id, disturbance_parameters,
                               pointing_result.current_los, boresight_chain, 0.0f,
                               &acquisition_request.command_azimuth_deg,
                               &acquisition_request.command_elevation_deg)) {
      nfov_scheduler_.Release(target_id);
      pointing_coordinator_.ReleaseTarget(target_id);
      target_states_[target_id] = SbirsTargetState::kWideCandidate;
      blocked_target_ids.insert(target_id);
      append_acquisition_failure(selected, channel_id,
                                 attribution::SbirsCaptureFailureReason::kNfovAcquisitionFailed);
      return;
    }
    acquisition_request.pointing_settle_error_deg = mission.narrow_pointing_settle_error_deg;
    acquisition_request.field_of_view_azimuth_deg = mission.narrow_field_fov_az_deg;
    acquisition_request.field_of_view_elevation_deg = mission.narrow_field_fov_el_deg;
    acquisition_request.snr = selected.snr;
    acquisition_request.minimum_snr_linear = policy.detection.narrow_min_snr_linear;
    const bool captured = IsNfovAcquisitionEligible(acquisition_request);
    if (captured) {
      if (!pointing_coordinator_.PromoteToTracking(target_id)) {
        nfov_scheduler_.Release(target_id);
        pointing_coordinator_.ReleaseTarget(target_id);
        target_states_[target_id] = SbirsTargetState::kWideCandidate;
        blocked_target_ids.insert(target_id);
        append_acquisition_failure(
            selected, channel_id,
            attribution::SbirsCaptureFailureReason::kNfovAcquisitionFailed);
        return;
      }
      cue_predictor_.Release(target_id);
      const bool use_estimated =
          policy.tracking.tracking_mode == config::SbirsTrackingMode::kEstimated;
      if (use_estimated) {
        target_states_[target_id] = SbirsTargetState::kEstimatedTracking;
      } else if (policy.tracking.tracking_mode ==
                 config::SbirsTrackingMode::kStrictTruthAssisted) {
        target_states_[target_id] = SbirsTargetState::kStrictTruthAssistedTracking;
      } else {
        target_states_[target_id] = SbirsTargetState::kSensorLikeTruthAssistedTracking;
      }
      if (use_estimated) {
        tracking_coordinator_.InitializeTarget(target_id, *selected.target, policy.tracking);
      }
      SbirsPipelineDetection detection;
      detection.record.detection_id = next_detection_id_++;
      detection.record.azimuth_rad = ToEciAzimuthRad(selected.measured_azimuth_deg);
      detection.record.elevation_rad = ToEciElevationRad(selected.measured_elevation_deg);
      detection.record.infrared_snr_linear = static_cast<float>(selected.snr);
      detection.record.observation_stage = output::SbirsObservationStage::kNarrowFieldAcquisition;
      detection.record.detected = true;
      detection.attribution.detection_id = detection.record.detection_id;
      detection.attribution.target_id = selected.target->target_id;
      detection.attribution.target_name = selected.target->target_name;
      detection.attribution.tracking_source = TrackingSourceForMode(policy.tracking.tracking_mode);
      detection.attribution.max_detection_range_m =
          static_cast<float>(selected.max_detection_range_m);
      if (policy.tracking.tracking_mode == config::SbirsTrackingMode::kStrictTruthAssisted) {
        detection.record.azimuth_rad = ToEciAzimuthRad(selected.azimuth_deg);
        detection.record.elevation_rad = ToEciElevationRad(selected.elevation_deg);
        detection.attribution.estimated_range_m = static_cast<float>(selected.range_m);
      } else if (policy.tracking.tracking_mode ==
                 config::SbirsTrackingMode::kSensorLikeTruthAssisted) {
        const foundation::SbirsErrorBearing bearing = foundation::ApplyAngularErrorModel(
            policy.error_model, &sensor_like_output_random_source_, selected.azimuth_deg,
            selected.elevation_deg, selected.range_m, selected.relative_angular_rate_deg_per_sec);
        detection.record.azimuth_rad = ToEciAzimuthRad(bearing.azimuth_deg);
        detection.record.elevation_rad = ToEciElevationRad(bearing.elevation_deg);
        detection.attribution.estimated_range_m = static_cast<float>(bearing.range_m);
      } else {
        detection.attribution.estimated_range_m = static_cast<float>(selected.range_m);
      }
      detection.attribution.nfov_channel_id = channel_id;
      result.detections.push_back(detection);
    } else {
      nfov_scheduler_.Release(target_id);
      pointing_coordinator_.ReleaseTarget(target_id);
      target_states_[target_id] = SbirsTargetState::kWideCandidate;
      blocked_target_ids.insert(target_id);
      append_acquisition_failure(selected, channel_id,
                                 attribution::SbirsCaptureFailureReason::kNfovAcquisitionFailed);
    }
  };

  std::vector<const SbirsCandidate*> awaiting_candidates;
  for (const SbirsCandidate& candidate : candidates) {
    const std::uint64_t target_id = candidate.target->target_id;
    if (target_states_[target_id] == SbirsTargetState::kAwaitingNfovAcquisition &&
        nfov_scheduler_.IsLocked(target_id)) {
      awaiting_candidates.push_back(&candidate);
    }
  }
  std::sort(awaiting_candidates.begin(), awaiting_candidates.end(),
            [this](const SbirsCandidate* lhs, const SbirsCandidate* rhs) {
              const int lhs_channel = nfov_scheduler_.ChannelOf(lhs->target->target_id);
              const int rhs_channel = nfov_scheduler_.ChannelOf(rhs->target->target_id);
              return lhs_channel != rhs_channel ? lhs_channel < rhs_channel
                                                : lhs->target->target_id < rhs->target->target_id;
            });
  for (const SbirsCandidate* candidate : awaiting_candidates) {
    advance_pointing(*candidate, nfov_scheduler_.ChannelOf(candidate->target->target_id));
  }

  std::vector<SbirsCandidate> new_candidates;
  for (const SbirsCandidate& candidate : candidates) {
    const std::uint64_t target_id = candidate.target->target_id;
    if (processed_target_ids.count(target_id) == 0U && blocked_target_ids.count(target_id) == 0U &&
        !nfov_scheduler_.IsLocked(target_id)) {
      new_candidates.push_back(candidate);
    }
  }
  const std::vector<const SbirsCandidate*> selected_candidates =
      nfov_scheduler_.SelectForAcquisition(new_candidates);
  for (const SbirsCandidate* selected : selected_candidates) {
    const std::uint64_t target_id = selected->target->target_id;
    const int channel_id = nfov_scheduler_.Acquire(target_id);
    if (channel_id < 0 ||
        !pointing_coordinator_.Reserve(
            channel_id, target_id,
            boresight_chain.EciLosOfSensorPointing(nominal_scan_azimuth_sensor_deg,
                                                   nominal_scan_elevation_sensor_deg))) {
      nfov_scheduler_.Release(target_id);
      pointing_coordinator_.ReleaseTarget(target_id);
      target_states_[target_id] = SbirsTargetState::kWideCandidate;
      processed_target_ids.insert(target_id);
      blocked_target_ids.insert(target_id);
      append_acquisition_failure(*selected, channel_id,
                                 attribution::SbirsCaptureFailureReason::kNfovAcquisitionFailed);
      continue;
    }
    target_states_[target_id] = SbirsTargetState::kAwaitingNfovAcquisition;
    advance_pointing(*selected, channel_id);
  }

  // 通道已满（无并发余量）时，未被选中的 WFOV 候选标记为调度跳过。
  const bool resources_full =
      static_cast<int>(nfov_scheduler_.LockedCount()) >= nfov_scheduler_.max_locks();
  for (const SbirsCandidate& candidate : candidates) {
    const std::uint64_t target_id = candidate.target->target_id;
    if (processed_target_ids.count(target_id) != 0U || nfov_scheduler_.IsLocked(target_id)) {
      continue;
    }
    SbirsPipelineDetection detection;
    detection.record.detection_id = next_detection_id_++;
    detection.record.azimuth_rad = ToEciAzimuthRad(candidate.measured_azimuth_deg);
    detection.record.elevation_rad = ToEciElevationRad(candidate.measured_elevation_deg);
    detection.record.infrared_snr_linear = static_cast<float>(candidate.snr);
    detection.record.observation_stage = output::SbirsObservationStage::kWideFieldSearch;
    detection.record.detected = true;
    detection.attribution.detection_id = detection.record.detection_id;
    detection.attribution.target_id = candidate.target->target_id;
    detection.attribution.target_name = candidate.target->target_name;
    detection.attribution.estimated_range_m = static_cast<float>(candidate.range_m);
    // 进入 WFOV 候选但未被调度器选中（资源被占用或排序靠后）：标记为调度跳过。
    if (resources_full) {
      detection.attribution.capture_failure_reason =
          attribution::SbirsCaptureFailureReason::kSchedulerSkipped;
    }
    result.detections.push_back(detection);
  }

  log_cycle_summary();
  return result;
}

SbirsPipelineSnapshot SbirsPipeline::CaptureRuntimeState() const {
  SbirsPipelineSnapshot snapshot;
  snapshot.scan_phase_deg = scan_phase_deg_;
  snapshot.scan_row_index = scan_row_index_;
  snapshot.misalignment_yaw_deg = static_cast<float>(misalignment_total_deg_.yaw_deg);
  snapshot.misalignment_pitch_deg = static_cast<float>(misalignment_total_deg_.pitch_deg);
  snapshot.misalignment_roll_deg = static_cast<float>(misalignment_total_deg_.roll_deg);
  snapshot.next_detection_id = next_detection_id_;
  snapshot.target_states = target_states_;
  snapshot.nfov_scheduler = nfov_scheduler_.Capture();
  snapshot.pointing_coordinator = pointing_coordinator_.Capture();
  snapshot.wfov_measurement_random_state = wfov_measurement_random_source_.Capture();
  snapshot.estimated_measurement_random_state = estimated_measurement_random_source_.Capture();
  snapshot.sensor_like_output_random_state = sensor_like_output_random_source_.Capture();
  snapshot.cue_predictor = cue_predictor_.Capture();
  const SbirsTrackingRuntimeState tracking_state = tracking_coordinator_.CaptureRuntimeState();
  snapshot.filter_states = tracking_state.filter_states;
  snapshot.nis_gate_exceeded_counts = tracking_state.nis_gate_exceeded_counts;
  snapshot.imm_active = tracking_state.imm_active;
  snapshot.imm_snapshots = tracking_state.imm_snapshots;
  return snapshot;
}

bool SbirsPipeline::RestoreRuntimeState(const SbirsPipelineSnapshot& snapshot) {
  if (!IsValidTrackingSnapshot(snapshot, config_.session.policy.tracking) ||
      snapshot.scan_phase_deg < 0.0f ||
      snapshot.scan_phase_deg >= config_.session.mission.scan_span_deg ||
      snapshot.scan_row_index < 0 ||
      snapshot.scan_row_index >= ScanRowCount(config_.session.mission)) {
    return false;
  }
  SbirsPointingCoordinator restored_pointing(
      nfov_scheduler_.max_locks(), config_.session.policy.pointing_disturbance.random_seed);
  if (!restored_pointing.Restore(snapshot.pointing_coordinator) ||
      snapshot.nfov_scheduler.target_to_channel.size() >
          static_cast<std::size_t>(nfov_scheduler_.max_locks())) {
    return false;
  }
  std::set<int> assigned_channels;
  for (const std::map<std::uint64_t, int>::value_type& assignment :
       snapshot.nfov_scheduler.target_to_channel) {
    const std::map<std::uint64_t, SbirsTargetState>::const_iterator state =
        snapshot.target_states.find(assignment.first);
    if (assignment.second < 0 || assignment.second >= nfov_scheduler_.max_locks() ||
        !assigned_channels.insert(assignment.second).second ||
        state == snapshot.target_states.end() ||
        (state->second != SbirsTargetState::kAwaitingNfovAcquisition &&
         state->second != SbirsTargetState::kEstimatedTracking &&
         state->second != SbirsTargetState::kStrictTruthAssistedTracking &&
         state->second != SbirsTargetState::kSensorLikeTruthAssistedTracking)) {
      return false;
    }
    const int pointing_channel = restored_pointing.ChannelOf(assignment.first);
    if (pointing_channel != assignment.second) {
      return false;
    }
  }
  for (const std::map<std::uint64_t, SbirsTargetState>::value_type& state :
       snapshot.target_states) {
    if ((state.second == SbirsTargetState::kAwaitingNfovAcquisition ||
         state.second == SbirsTargetState::kEstimatedTracking ||
         state.second == SbirsTargetState::kStrictTruthAssistedTracking ||
         state.second == SbirsTargetState::kSensorLikeTruthAssistedTracking) &&
        snapshot.nfov_scheduler.target_to_channel.find(state.first) ==
            snapshot.nfov_scheduler.target_to_channel.end()) {
      return false;
    }
  }
  const SbirsPointingCoordinatorSnapshot restored_snapshot = restored_pointing.Capture();
  for (const SbirsPointingChannelSnapshot& channel : restored_snapshot.channels) {
    if (!channel.has_bound_target) {
      continue;
    }
    const std::map<std::uint64_t, int>::const_iterator assignment =
        snapshot.nfov_scheduler.target_to_channel.find(channel.target_id);
    if (assignment == snapshot.nfov_scheduler.target_to_channel.end() ||
        assignment->second != channel.channel_id ||
        snapshot.target_states.find(channel.target_id) == snapshot.target_states.end() ||
        (snapshot.target_states.find(channel.target_id)->second !=
             SbirsTargetState::kAwaitingNfovAcquisition &&
         snapshot.target_states.find(channel.target_id)->second !=
             SbirsTargetState::kEstimatedTracking &&
         snapshot.target_states.find(channel.target_id)->second !=
             SbirsTargetState::kStrictTruthAssistedTracking &&
         snapshot.target_states.find(channel.target_id)->second !=
             SbirsTargetState::kSensorLikeTruthAssistedTracking)) {
      return false;
    }
    const SbirsTargetState channel_state = snapshot.target_states.find(channel.target_id)->second;
    if ((channel_state == SbirsTargetState::kAwaitingNfovAcquisition &&
         channel.tracking_gate_failure_count != 0U) ||
        (channel_state != SbirsTargetState::kAwaitingNfovAcquisition &&
         (channel.elapsed_wait_sec != 0.0 ||
          channel.tracking_gate_failure_count >=
              config_.session.policy.tracking.nfov_tracking_gate_loss_cycles))) {
      return false;
    }
  }
  SbirsTrackingRuntimeState tracking_state;
  tracking_state.filter_states = snapshot.filter_states;
  tracking_state.nis_gate_exceeded_counts = snapshot.nis_gate_exceeded_counts;
  tracking_state.imm_active = snapshot.imm_active;
  tracking_state.imm_snapshots = snapshot.imm_snapshots;
  scan_phase_deg_ = snapshot.scan_phase_deg;
  scan_row_index_ = snapshot.scan_row_index;
  misalignment_total_deg_.yaw_deg = snapshot.misalignment_yaw_deg;
  misalignment_total_deg_.pitch_deg = snapshot.misalignment_pitch_deg;
  misalignment_total_deg_.roll_deg = snapshot.misalignment_roll_deg;
  next_detection_id_ = snapshot.next_detection_id;
  target_states_ = snapshot.target_states;
  nfov_scheduler_.Restore(snapshot.nfov_scheduler);
  pointing_coordinator_ = restored_pointing;
  wfov_measurement_random_source_.Restore(snapshot.wfov_measurement_random_state);
  estimated_measurement_random_source_.Restore(snapshot.estimated_measurement_random_state);
  sensor_like_output_random_source_.Restore(snapshot.sensor_like_output_random_state);
  cue_predictor_.Restore(snapshot.cue_predictor);
  tracking_coordinator_.RestoreRuntimeState(tracking_state);
  return true;
}

}  // namespace pipeline
}  // namespace sbirs_sensor
