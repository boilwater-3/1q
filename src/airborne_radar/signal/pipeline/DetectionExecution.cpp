#include "airborne_radar/signal/pipeline/DetectionExecution.h"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>

#include "1q/airborne_radar/common/utils/MathUtils.h"
#include "airborne_radar/signal/detection/BeamControlResolver.h"
#include "airborne_radar/signal/detection/MeasurementErrorModel.h"
#include "airborne_radar/signal/detection/SignalDetector.h"
#include "airborne_radar/signal/pipeline/ControlProfileEffects.h"
#include "airborne_radar/signal/pipeline/JammingEffects.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

namespace {

using common::utils::ClampFloat;
using common::utils::DbToLinearPower;

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
 * @brief 从目标特征中解析速度标量
 * @param[in] target 目标特征数据，包含航迹速度分量和标量速度
 * @return 速度矢量的范数；当速度分量全为零时回退到 current_track_speed 字段
 */
float ResolveSpeedMagnitude(const common::model::TargetFeature& target) {
  const Eigen::Vector3f velocity(target.current_track_velocity_x, target.current_track_velocity_y,
                                 target.current_track_velocity_z);
  if (velocity.squaredNorm() > 0.0f) {
    return velocity.norm();
  }
  return target.current_track_speed;
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
                               const common::control::RadarControlProfile& control_profile,
                               const environment::EnvironmentSnapshot& environment_snapshot,
                               DetectionExecutionBuffers* buffers) {
  if (buffers == nullptr || !HasValidBuffers(*buffers)) {
    return;
  }

  const std::size_t count = input.size();
  const float signal_adjustment_db =
      ComputeHeuristicSignalAdjustmentDb(runtime_config.control_profile_effects, control_profile);
  for (std::size_t i = 0; i < count; ++i) {
    (*buffers->target_geometry)[i] = detection::TargetGeometryResolver::Resolve(input[i]);
    (*buffers->signal_term_db)[i] = input[i].current_track_rcs * 6.0f + signal_adjustment_db;
    (*buffers->speed_penalty_db)[i] = ResolveSpeedMagnitude(input[i]) * 0.002f;
  }

  const float jamming_penalty_db =
      ComputeHeuristicJammingPenaltyDb(runtime_config.jamming_effects, environment_snapshot);
  const float environment_penalty_db = std::max(
      0.0f, environment_snapshot.propagation_loss_db * 0.2f +
                environment_snapshot.clutter_power_db * 0.3f + jamming_penalty_db -
                ComputeHeuristicEnvironmentReliefDb(runtime_config.jamming_effects, control_profile,
                                                    environment_snapshot));
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
                              const common::control::RadarControlProfile& control_profile,
                              const environment::EnvironmentSnapshot& environment_snapshot,
                              detection::SignalDetector* signal_detector,
                              DetectionExecutionBuffers* buffers) {
  if (signal_detector == nullptr || buffers == nullptr || !HasValidBuffers(*buffers)) {
    return;
  }

  const std::size_t count = input.size();

  float clutter_w = std::pow(10.0f, environment_snapshot.clutter_power_db / 10.0f);
  if (control_profile.enable_sidelobe_canceller) {
    clutter_w *= environment_snapshot.jammer_in_sidelobe ? 0.55f : 0.80f;
  }

  float jam_w = 0.0f;
  if (HasMultiSourceJammingFacts(environment_snapshot)) {
    for (std::size_t i = 0; i < environment_snapshot.jammer_sources.size(); ++i) {
      const environment::JammerSourceFact& source = environment_snapshot.jammer_sources[i];
      jam_w += ComputePhysicalSourceJamContributionW(runtime_config.jamming_effects, source) *
               ComputeResidualJammerFactor(control_profile, source);
    }
  } else if (environment_snapshot.jamming_detected) {
    jam_w = DbToLinearPower(environment_snapshot.jammer_power_db);
    if (control_profile.enable_sidelobe_canceller) {
      jam_w *= environment_snapshot.jammer_in_sidelobe ? 0.35f : 0.80f;
    }
    if (control_profile.enable_agility_frequency) {
      const float overlap_ratio =
          ClampFloat(environment_snapshot.jammer_frequency_overlap_ratio, 0.0f, 1.0f);
      jam_w *= ClampFloat(1.0f - 0.60f * overlap_ratio, 0.25f, 1.0f);
    }
    if (control_profile.enable_eccm_rejitter) {
      const float prf_lock_risk = ClampFloat(environment_snapshot.jammer_prf_lock_risk, 0.0f, 1.0f);
      jam_w *= ClampFloat(1.0f - 0.50f * prf_lock_risk, 0.35f, 1.0f);
    }
    if (control_profile.enable_adaptive_beamforming) {
      jam_w *= environment_snapshot.jammer_in_sidelobe ? 0.85f : 0.75f;
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
    target.swerling_type = static_cast<detection::SwerlingModel>(input[i].target_swerling_type);

    const detection::ResolvedBeamState beam_state =
        detection::BeamControlResolver::Resolve(runtime_config.detection.radar_system.antenna,
                                                runtime_config.beam_control.radar_orientation,
                                                runtime_config.beam_control.platform_attitude_deg,
                                                (*buffers->target_geometry)[i].look_angles_deg);
    const detection::DetectionResult detection_result = signal_detector->Detect(
        target, env, beam_state.one_way_antenna_gain_db, runtime_config.detection.pulse_count,
        runtime_config.detection.coherent_integration);
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
        runtime_config.jamming_effects, control_profile, environment_snapshot);
  }
}

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
