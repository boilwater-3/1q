#include "airborne_radar/signal/pipeline/DetectionExecution.h"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>

#include "airborne_radar/signal/detection/BeamControlResolver.h"
#include "airborne_radar/signal/detection/MeasurementErrorModel.h"
#include "airborne_radar/signal/detection/SignalDetector.h"
#include "airborne_radar/signal/pipeline/ControlProfileEffects.h"
#include "airborne_radar/signal/pipeline/JammingEffects.h"
#include "airborne_radar/signal/pipeline/PipelineTargetUtils.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

namespace {

/**
 * @brief 根据目标几何与测量误差参数构建测量协方差矩阵
 * @param[in] geometry 已解析的目标几何信息（距离、位置等）
 * @param[in] range_error_std 距离测量误差标准差，单位为 m
 * @param[in] angle_error_std 角度测量误差标准差，单位为 rad
 * @param[in] default_measurement_noise_std 当输入误差参数无效时使用的默认测量噪声标准差
 * @return 构建好的 MeasurementCovariance 矩阵；当输入误差参数非正时退化为各向同性默认协方差
 */
tracking::MeasurementCovariance BuildMeasurementCovariance(
    const detection::ResolvedTargetGeometry& geometry, float range_error_std, float angle_error_std,
    float default_measurement_noise_std) {
  if (range_error_std <= 0.0f || angle_error_std <= 0.0f) {
    return tracking::MeasurementCovariance::Identity() * default_measurement_noise_std *
           default_measurement_noise_std;
  }

  const float range_m = std::max(geometry.range_m, 0.1f);
  const float var_r = range_error_std * range_error_std;
  const float var_theta = angle_error_std * angle_error_std;
  Eigen::Vector3f pos =
      geometry.has_cartesian_position ? geometry.position_m : Eigen::Vector3f(range_m, 0.0f, 0.0f);
  const float pos_norm = pos.norm();
  if (pos_norm > 0.1f) {
    const Eigen::Vector3f u = pos / pos_norm;
    const Eigen::Matrix3f identity = Eigen::Matrix3f::Identity();
    const Eigen::Matrix3f uu_t = u * u.transpose();
    return var_r * uu_t + (range_m * range_m * var_theta) * (identity - uu_t);
  }

  return tracking::MeasurementCovariance::Identity() * var_r;
}

/**
 * @brief 检测 DetectionExecutionBuffers 中所有必需指针是否均已挂载
 * @param[in] buffers 待检查的检测执行缓冲区
 * @return 所有指针均非空时返回 true，否则返回 false
 */
bool HasValidBuffers(const DetectionExecutionBuffers& buffers) {
  return buffers.target_geometry != nullptr && buffers.signal_term_db != nullptr &&
         buffers.speed_penalty_db != nullptr && buffers.detection_margin_db != nullptr &&
         buffers.detection_succeeded != nullptr && buffers.measurement_covariances != nullptr;
}

}  // namespace

void RunHeuristicDetectionPass(const common::model::TargetFeatureList& input,
                               const SignalPipelineConfig& runtime_config,
                               const InternalSignalPipelineConfig& internal_config,
                               const common::control::RadarControlProfile& control_profile,
                               const environment::EnvironmentSnapshot& environment_snapshot,
                               DetectionExecutionBuffers* buffers) {
  if (buffers == nullptr || !HasValidBuffers(*buffers)) {
    return;
  }

  const std::size_t count = input.size();
  const float signal_adjustment_db =
      ComputeHeuristicSignalAdjustmentDb(internal_config.control_profile_effects, control_profile);
  for (std::size_t i = 0; i < count; ++i) {
    (*buffers->target_geometry)[i] = detection::TargetGeometryResolver::Resolve(input[i]);
    (*buffers->signal_term_db)[i] = input[i].current_track_rcs * 6.0f + signal_adjustment_db;
    (*buffers->speed_penalty_db)[i] = ResolveSpeedMagnitude(input[i]) * 0.002f;
  }

  const float jamming_penalty_db =
      ComputeHeuristicJammingPenaltyDb(internal_config.jamming_effects, environment_snapshot);
  const float environment_penalty_db = std::max(
      0.0f, environment_snapshot.propagation_loss_db * 0.2f +
                environment_snapshot.clutter_power_db * 0.3f + jamming_penalty_db -
                ComputeHeuristicEnvironmentReliefDb(internal_config.jamming_effects,
                                                    control_profile, environment_snapshot));
  for (std::size_t i = 0; i < count; ++i) {
    const float margin =
        (*buffers->signal_term_db)[i] - (*buffers->speed_penalty_db)[i] - environment_penalty_db;
    (*buffers->detection_margin_db)[i] = margin;
    (*buffers->detection_succeeded)[i] =
        static_cast<std::uint8_t>(margin >= runtime_config.detection.min_detection_margin_db);
  }
}

void RunPhysicalDetectionPass(const common::model::TargetFeatureList& input,
                              const SignalPipelineConfig& runtime_config,
                              const InternalSignalPipelineConfig& internal_config,
                              const common::control::RadarControlProfile& control_profile,
                              const environment::EnvironmentSnapshot& environment_snapshot,
                              detection::SignalDetector* signal_detector,
                              DetectionExecutionBuffers* buffers) {
  if (signal_detector == nullptr || buffers == nullptr || !HasValidBuffers(*buffers)) {
    return;
  }

  const std::size_t count = input.size();

  float clutter_w = std::pow(10.0f, environment_snapshot.clutter_power_db / 10.0f);
  if (control_profile.enable_sidelobe_canceller &&
      HasMultiSourceJammingFacts(environment_snapshot)) {
    const bool has_sidelobe_source = std::find_if(environment_snapshot.jammer_sources.begin(),
                                                  environment_snapshot.jammer_sources.end(),
                                                  [](const environment::JammerSourceFact& source) {
                                                    return source.in_sidelobe;
                                                  }) != environment_snapshot.jammer_sources.end();
    clutter_w *= has_sidelobe_source ? 0.55f : 0.80f;
  }

  float jam_w = 0.0f;
  if (HasMultiSourceJammingFacts(environment_snapshot)) {
    for (std::size_t i = 0; i < environment_snapshot.jammer_sources.size(); ++i) {
      const environment::JammerSourceFact& source = environment_snapshot.jammer_sources[i];
      jam_w += ComputePhysicalSourceJamContributionW(internal_config.jamming_effects, source) *
               ComputeResidualJammerFactor(control_profile, source);
    }
  }

  detection::EnvironmentState env;
  env.propagation_loss_db = environment_snapshot.propagation_loss_db;
  env.clutter_noise_w = clutter_w;
  env.jam_noise_w = jam_w;

  signal_detector->UpdateConfig(runtime_config.detection.radar_system);

  for (std::size_t i = 0; i < count; ++i) {
    (*buffers->target_geometry)[i] = detection::TargetGeometryResolver::Resolve(input[i]);
    detection::TargetReturn target;
    target.rcs_m2 = input[i].current_track_rcs;
    target.range_m = (*buffers->target_geometry)[i].range_m;
    target.swerling_type =
        static_cast<common::config::SwerlingModel>(input[i].target_swerling_type);

    const detection::ResolvedBeamState beam_state =
        detection::BeamControlResolver::Resolve(runtime_config.detection.radar_system.antenna,
                                                runtime_config.beam_control.radar_orientation,
                                                runtime_config.beam_control.platform_attitude_deg,
                                                (*buffers->target_geometry)[i].look_angles_deg);
    const detection::DetectionResult detection_result = signal_detector->Detect(
        target, env, beam_state.one_way_antenna_gain_db, runtime_config.detection.pulse_count);
    const detection::MeasurementErrorState measurement_error =
        detection::MeasurementErrorModel::Compute(
            detection_result.snr_db, beam_state.effective_beamwidth_deg,
            runtime_config.detection.radar_system.transmitter.bandwidth_hz);

    (*buffers->signal_term_db)[i] = detection_result.snr_db;
    (*buffers->speed_penalty_db)[i] = 0.0f;
    (*buffers->detection_margin_db)[i] = detection_result.snr_db;
    (*buffers->detection_succeeded)[i] =
        static_cast<std::uint8_t>(detection_result.detected ? 1U : 0U);
    (*buffers->measurement_covariances)[i] = BuildMeasurementCovariance(
        (*buffers->target_geometry)[i], measurement_error.range_error_std_m,
        measurement_error.angle_error_std_rad,
        runtime_config.tracking.kalman_measurement_noise_std);
    (*buffers->measurement_covariances)[i] *= ComputeMeasurementCovarianceInflation(
        internal_config.jamming_effects, control_profile, environment_snapshot);
  }
}

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
