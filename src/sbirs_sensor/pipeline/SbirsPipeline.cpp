#include "sbirs_sensor/pipeline/SbirsPipeline.h"

#include <algorithm>
#include <cmath>
#include <set>

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

session::SbirsVector3M LosFromAzimuthElevation(float azimuth_deg, float elevation_deg) {
  const double kDegreesToRadians = 0.017453292519943295;
  const double azimuth_rad = static_cast<double>(azimuth_deg) * kDegreesToRadians;
  const double elevation_rad = static_cast<double>(elevation_deg) * kDegreesToRadians;
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
                           const session::SbirsVector3M& nominal_los, float static_error_deg,
                           float* azimuth_deg, float* elevation_deg) {
  if (azimuth_deg == nullptr || elevation_deg == nullptr) {
    return false;
  }
  SbirsPointingDisturbanceSample disturbance;
  if (!coordinator.DisturbanceSample(channel_id, parameters, &disturbance)) {
    return false;
  }
  *azimuth_deg =
      foundation::ComputeAzimuthDeg(nominal_los) +
      static_cast<float>(disturbance.common.azimuth_deg + disturbance.channel.azimuth_deg) +
      static_error_deg;
  *elevation_deg =
      foundation::ComputeElevationDeg(nominal_los) +
      static_cast<float>(disturbance.common.elevation_deg + disturbance.channel.elevation_deg);
  return true;
}

bool IsFiniteGaussianState(const tracking::SbirsGaussianState& state) {
  return state.mean.allFinite() && state.covariance.allFinite();
}

bool IsValidTrackingSnapshot(const SbirsPipelineSnapshot& snapshot,
                             const config::SbirsTrackingConfig& tracking_config) {
  if (!std::isfinite(snapshot.scan_phase_deg) || snapshot.next_detection_id == 0U ||
      snapshot.wfov_measurement_random_state == 0U ||
      snapshot.estimated_measurement_random_state == 0U ||
      snapshot.sensor_like_output_random_state == 0U) {
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
  config_ = config;
  if (impact.scan_sector_changed) {
    const float candidate_phase =
        ScanPhaseForAzimuth(config_.session.mission, previous_scan_azimuth_deg);
    scan_phase_deg_ = config_.session.mission.scan_span_deg == 360.0f ||
                              candidate_phase < config_.session.mission.scan_span_deg
                          ? candidate_phase
                          : 0.0f;
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

  if (mission.work_mode == config::SbirsWorkMode::kStandby || !config_.session.sensor_enabled) {
    target_states_.clear();
    tracking_coordinator_.ClearForStandby();
    nfov_scheduler_.Clear();
    pointing_coordinator_.Clear();
    cue_predictor_.Clear();
    result.scan_azimuth_deg = ScanAzimuth(mission, scan_phase_deg_);
    return result;
  }

  const SbirsPointingDisturbanceParameters disturbance_parameters =
      DisturbanceParameters(policy.pointing_disturbance);
  if (!pointing_coordinator_.AdvanceDisturbance(static_cast<double>(input.dt_sec),
                                                disturbance_parameters)) {
    result.scan_azimuth_deg = ScanAzimuth(mission, scan_phase_deg_);
    return result;
  }
  SbirsPointingDisturbanceSample frame_disturbance;
  if (!pointing_coordinator_.DisturbanceSample(0, disturbance_parameters, &frame_disturbance)) {
    result.scan_azimuth_deg = ScanAzimuth(mission, scan_phase_deg_);
    return result;
  }

  scan_phase_deg_ =
      PositiveModulo(scan_phase_deg_ + mission.scan_rate_deg_per_sec * std::max(0.0f, input.dt_sec),
                     mission.scan_span_deg);
  const float scan_azimuth_deg = ScanAzimuth(mission, scan_phase_deg_);
  result.scan_azimuth_deg = scan_azimuth_deg;
  const float actual_scan_azimuth_deg =
      NormalizeAzimuth(scan_azimuth_deg + static_cast<float>(frame_disturbance.common.azimuth_deg));
  const float actual_scan_elevation_deg =
      mission.scan_center_el_deg + static_cast<float>(frame_disturbance.common.elevation_deg);

  const float transmittance = environment::ResolveEffectiveTransmittance(environment_config);
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

  for (const session::SbirsSceneTarget& target : input.scene) {
    if (!target.active) {
      target_states_[target.target_id] = SbirsTargetState::kLost;
      nfov_scheduler_.Release(target.target_id);
      pointing_coordinator_.ReleaseTarget(target.target_id);
      cue_predictor_.Release(target.target_id);
      tracking_coordinator_.ReleaseTarget(target.target_id);
      continue;
    }

    if (foundation::IsEarthOcculted(input.satellite_position_ecef_m, target.position_ecef_m,
                                    kEarthRadiusM)) {
      target_states_[target.target_id] = SbirsTargetState::kUndetected;
      nfov_scheduler_.Release(target.target_id);
      pointing_coordinator_.ReleaseTarget(target.target_id);
      cue_predictor_.Release(target.target_id);
      tracking_coordinator_.ReleaseTarget(target.target_id);
      continue;
    }

    const session::SbirsVector3M los =
        foundation::Subtract(target.position_ecef_m, input.satellite_position_ecef_m);
    const double range_m = foundation::Norm(los);
    if (range_m < mission.min_range_m || range_m > mission.max_range_m) {
      target_states_[target.target_id] = SbirsTargetState::kUndetected;
      nfov_scheduler_.Release(target.target_id);
      pointing_coordinator_.ReleaseTarget(target.target_id);
      cue_predictor_.Release(target.target_id);
      tracking_coordinator_.ReleaseTarget(target.target_id);
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

    const bool in_wfov = InRectangularFov(azimuth_deg, elevation_deg, actual_scan_azimuth_deg,
                                          actual_scan_elevation_deg, mission.wide_field_fov_az_deg,
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
            target.target_id, policy, input.dt_sec, input.satellite_position_ecef_m);
        command_azimuth_deg = prediction.output_azimuth_deg;
        command_elevation_deg = prediction.output_elevation_deg;
      }
      SbirsPointingActuatorConfig tracking_pointing_config;
      tracking_pointing_config.max_slew_rate_deg_per_sec = mission.narrow_pointing_max_slew_rate_deg_per_sec;
      tracking_pointing_config.settle_tolerance_deg = mission.narrow_pointing_settle_tolerance_deg;
      const SbirsPointingAdvanceResult pointing_result = pointing_coordinator_.AdvanceTracking(
          channel_id, target.target_id,
          LosFromAzimuthElevation(command_azimuth_deg, command_elevation_deg), input.dt_sec,
          tracking_pointing_config);
      float actual_pointing_azimuth_deg = 0.0f;
      float actual_pointing_elevation_deg = 0.0f;
      const bool pointing_available = EffectiveNfovPointing(
          pointing_coordinator_, channel_id, disturbance_parameters, pointing_result.current_los,
          mission.narrow_pointing_settle_error_deg, &actual_pointing_azimuth_deg,
          &actual_pointing_elevation_deg);
      const bool geometry_gate_passed =
          pointing_result.status != SbirsPointingAdvanceStatus::kRejected && pointing_available &&
          InRectangularFov(azimuth_deg, elevation_deg, actual_pointing_azimuth_deg,
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
      detection.record.azimuth_deg = command_azimuth_deg;
      detection.record.elevation_deg = command_elevation_deg;
      detection.record.infrared_snr_linear = static_cast<float>(snr);
      detection.record.observation_stage = output::SbirsObservationStage::kNarrowFieldTrack;
      detection.record.detected = tracking_gate_passed;
      detection.attribution.detection_id = detection.record.detection_id;
      detection.attribution.target_id = target.target_id;
      detection.attribution.target_name = target.target_name;
      detection.attribution.estimated_range_m = static_cast<float>(range_m);
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
            omega_deg_per_sec_cached, input.satellite_position_ecef_m);
        detection.record.azimuth_deg = tracking_result.output_azimuth_deg;
        detection.record.elevation_deg = tracking_result.output_elevation_deg;
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
        detection.record.azimuth_deg = bearing.azimuth_deg;
        detection.record.elevation_deg = bearing.elevation_deg;
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

    if (!in_wfov || snr < policy.detection.wide_min_snr_linear) {
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
    // 2.10 WFOV 带误差位置：施加 5 类物理误差（高斯随机 + 折射 + 滞后）。
    // 复用上方已算的目标角速度 omega_deg_per_sec_cached。
    const foundation::SbirsErrorBearing bearing = foundation::ApplyAngularErrorModel(
        policy.error_model, &wfov_measurement_random_source_, azimuth_deg, elevation_deg, range_m,
        /*target_angular_rate_deg_per_sec=*/omega_deg_per_sec_cached);
    candidate.measured_azimuth_deg = bearing.azimuth_deg;
    candidate.measured_elevation_deg = bearing.elevation_deg;
    candidate.measured_range_m = bearing.range_m;
    candidate.angular_rate_deg_per_sec = omega_deg_per_sec_cached;
    const SbirsCuePrediction cue_prediction =
        cue_predictor_.Update(target.target_id, bearing.azimuth_deg, bearing.elevation_deg,
                              input.dt_sec, mission.narrow_cue_latency_s);
    candidate.command_azimuth_deg = cue_prediction.command_azimuth_deg;
    candidate.command_elevation_deg = cue_prediction.command_elevation_deg;
    // cue 延迟外推：narrow_cue_latency_s 期间目标继续运动，真值 az/el 需按延迟后位置重算。
    const float cue_latency_s = mission.narrow_cue_latency_s;
    if (cue_latency_s > 0.0f && target.has_velocity_ecef_m_per_s) {
      session::SbirsVector3M predicted_position;
      predicted_position.x =
          target.position_ecef_m.x + target.velocity_ecef_m_per_s.x * cue_latency_s;
      predicted_position.y =
          target.position_ecef_m.y + target.velocity_ecef_m_per_s.y * cue_latency_s;
      predicted_position.z =
          target.position_ecef_m.z + target.velocity_ecef_m_per_s.z * cue_latency_s;
      const session::SbirsVector3M predicted_los =
          foundation::Subtract(predicted_position, input.satellite_position_ecef_m);
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
      detection.record.azimuth_deg = candidate.measured_azimuth_deg;
      detection.record.elevation_deg = candidate.measured_elevation_deg;
      detection.record.infrared_snr_linear = static_cast<float>(candidate.snr);
      detection.record.observation_stage = output::SbirsObservationStage::kWideFieldSearch;
      detection.record.detected = true;
      detection.attribution.detection_id = detection.record.detection_id;
      detection.attribution.target_id = candidate.target->target_id;
      detection.attribution.target_name = candidate.target->target_name;
      detection.attribution.estimated_range_m = static_cast<float>(candidate.range_m);
      detection.attribution.nfov_channel_id = -1;
      result.detections.push_back(detection);
    }
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
    detection.record.azimuth_deg = candidate.measured_azimuth_deg;
    detection.record.elevation_deg = candidate.measured_elevation_deg;
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
    detection.record.azimuth_deg = candidate.measured_azimuth_deg;
    detection.record.elevation_deg = candidate.measured_elevation_deg;
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
    const SbirsPointingAdvanceResult pointing_result = pointing_coordinator_.Advance(
        channel_id, target_id,
        LosFromAzimuthElevation(selected.command_azimuth_deg, selected.command_elevation_deg),
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
    acquisition_request.delayed_truth_azimuth_deg = selected.delayed_truth_azimuth_deg;
    acquisition_request.delayed_truth_elevation_deg = selected.delayed_truth_elevation_deg;
    if (!EffectiveNfovPointing(pointing_coordinator_, channel_id, disturbance_parameters,
                               pointing_result.current_los, 0.0f,
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
      detection.record.azimuth_deg = selected.measured_azimuth_deg;
      detection.record.elevation_deg = selected.measured_elevation_deg;
      detection.record.infrared_snr_linear = static_cast<float>(selected.snr);
      detection.record.observation_stage = output::SbirsObservationStage::kNarrowFieldAcquisition;
      detection.record.detected = true;
      detection.attribution.detection_id = detection.record.detection_id;
      detection.attribution.target_id = selected.target->target_id;
      detection.attribution.target_name = selected.target->target_name;
      detection.attribution.tracking_source = TrackingSourceForMode(policy.tracking.tracking_mode);
      if (policy.tracking.tracking_mode == config::SbirsTrackingMode::kStrictTruthAssisted) {
        detection.record.azimuth_deg = selected.azimuth_deg;
        detection.record.elevation_deg = selected.elevation_deg;
        detection.attribution.estimated_range_m = static_cast<float>(selected.range_m);
      } else if (policy.tracking.tracking_mode ==
                 config::SbirsTrackingMode::kSensorLikeTruthAssisted) {
        const foundation::SbirsErrorBearing bearing = foundation::ApplyAngularErrorModel(
            policy.error_model, &sensor_like_output_random_source_, selected.azimuth_deg,
            selected.elevation_deg, selected.range_m, selected.angular_rate_deg_per_sec);
        detection.record.azimuth_deg = bearing.azimuth_deg;
        detection.record.elevation_deg = bearing.elevation_deg;
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
            LosFromAzimuthElevation(scan_azimuth_deg, mission.scan_center_el_deg))) {
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
    detection.record.azimuth_deg = candidate.measured_azimuth_deg;
    detection.record.elevation_deg = candidate.measured_elevation_deg;
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

  return result;
}

SbirsPipelineSnapshot SbirsPipeline::CaptureRuntimeState() const {
  SbirsPipelineSnapshot snapshot;
  snapshot.scan_phase_deg = scan_phase_deg_;
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
      snapshot.scan_phase_deg >= config_.session.mission.scan_span_deg) {
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
