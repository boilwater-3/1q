#include "airborne_radar/signal/pipeline/DetectionExecution.h"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>

#include "airborne_radar/config/SignalEngineeringConfig.h"
#include "airborne_radar/signal/detection/BeamControlResolver.h"
#include "airborne_radar/signal/detection/MeasurementErrorModel.h"
#include "airborne_radar/signal/detection/SignalDetector.h"
#include "airborne_radar/signal/pipeline/ControlProfileEffects.h"
#include "airborne_radar/signal/pipeline/JammingEffects.h"
#include "airborne_radar/signal/pipeline/PipelineTargetUtils.h"
#include "common/atmosphere/AtmospherePhysics.h"
#include "common/rcs/RcsPhysics.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

namespace {


float ResolveRcsPhysicsFrequencyHz(const ExecutionConfig& exec_config) {
  const float config_frequency_hz = exec_config.detection.engineering.rcs_physics.frequency_hz;
  if (config_frequency_hz > 0.0f) {
    return config_frequency_hz;
  }
  return exec_config.detection.engineering.transmitter.frequency_hz;
}

float ComputeEquivalentClutterNoiseW(
    const config::engineering::DetectionConfig& detection_config, float clutter_power_db) {
  if (!std::isfinite(clutter_power_db)) {
    return 0.0f;
  }

  const float thermal_noise_w = detection::RadarEquations::ComputeThermalNoisePower_W(
      detection_config.transmitter, detection_config.receiver);
  if (!std::isfinite(thermal_noise_w) || thermal_noise_w <= 0.0f) {
    return 0.0f;
  }

  const float kMinRelativeClutterDb = -120.0f;
  const float kMaxRelativeClutterDb = 120.0f;
  const float relative_clutter_db =
      oneq::common::numerics::Clamp(clutter_power_db, kMinRelativeClutterDb, kMaxRelativeClutterDb);
  return thermal_noise_w * std::pow(10.0f, relative_clutter_db / 10.0f);
}

float ComputeTargetSpecificAtmosphericLossDb(
    const ExecutionConfig& exec_config,
    const session::EnvironmentSnapshot& environment_snapshot, float platform_altitude_m,
    const detection::ResolvedTargetGeometry& geometry) {
  if (!environment_snapshot.atmospheric_physics.enable_physical_model) {
    return 0.0f;
  }

  oneq::common::atmosphere::AtmosphericObservationRef obs;
  obs.pressure_hpa = environment_snapshot.atmospheric_physics.pressure_hpa;
  obs.temperature_k = environment_snapshot.atmospheric_physics.temperature_k;
  obs.relative_humidity = environment_snapshot.atmospheric_physics.relative_humidity;
  obs.k_factor = environment_snapshot.effective_k_factor;
  obs.day_of_year = environment_snapshot.effective_day_of_year;
  obs.solar_flux_f107a = environment_snapshot.atmospheric_context.solar_flux_f107a;
  obs.solar_flux_f107 = environment_snapshot.atmospheric_context.solar_flux_f107;
  obs.geomagnetic_ap = environment_snapshot.atmospheric_context.geomagnetic_ap;
  const auto inputs = oneq::common::atmosphere::BuildPropagationInputs(
      exec_config.detection.engineering.transmitter.frequency_hz,
      std::max(geometry.range_m, 0.1f), platform_altitude_m,
      std::max(platform_altitude_m + geometry.position_m.z(), 0.0f),
      geometry.look_angles_deg.has_look_angles ? geometry.look_angles_deg.look_el_deg : 0.0f, obs);
  return oneq::common::atmosphere::EvaluateAtmosphericPropagation(inputs).total_physics_loss_db;
}

float ComputeEquivalentRadiusM(float input_rcs_m2,
                               const config::engineering::RcsPhysicsConfig& rcs_config) {
  const float min_radius_m = std::max(rcs_config.min_equivalent_radius_m, 1.0e-3f);
  const float max_radius_m = std::max(rcs_config.max_equivalent_radius_m, min_radius_m);
  const float safe_input_rcs_m2 = std::max(input_rcs_m2, 0.0f);
  const float equivalent_radius_m = std::sqrt(safe_input_rcs_m2 / static_cast<float>(oneq::common::numerics::kPi));
  return oneq::common::numerics::Clamp(equivalent_radius_m, min_radius_m, max_radius_m);
}

float ComputeEffectiveTargetRcsM2(const session::ArSceneTarget& target,
                                  const detection::ResolvedTargetGeometry& geometry,
                                  const ExecutionConfig& exec_config) {
  const float input_rcs_m2 = std::max(target.rcs, 0.0f);
  const config::engineering::RcsPhysicsConfig& rcs_config =
      exec_config.detection.engineering.rcs_physics;
  if (!rcs_config.enable_physical_rcs) {
    return input_rcs_m2;
  }

  const float mix_ratio = oneq::common::numerics::Clamp(rcs_config.physics_mix_ratio, 0.0f, 1.0f);
  if (mix_ratio <= 0.0f) {
    return input_rcs_m2;
  }

  const float frequency_hz = ResolveRcsPhysicsFrequencyHz(exec_config);
  if (frequency_hz <= 0.0f) {
    return input_rcs_m2;
  }

  const float wavenumber_k0 = 2.0f * static_cast<float>(oneq::common::numerics::kPi) * frequency_hz / static_cast<float>(oneq::common::numerics::kLightSpeed);
  if (wavenumber_k0 <= 0.0f) {
    return input_rcs_m2;
  }

  const float equivalent_radius_m = ComputeEquivalentRadiusM(input_rcs_m2, rcs_config);
  const float azimuth_deg = geometry.look_angles_deg.has_look_angles
                                ? std::fabs(geometry.look_angles_deg.look_az_deg)
                                : 0.0f;
  const float elevation_deg = geometry.look_angles_deg.has_look_angles
                                  ? std::fabs(geometry.look_angles_deg.look_el_deg)
                                  : 0.0f;
  const float psi_i_deg = oneq::common::numerics::Clamp(elevation_deg, 0.0f, 89.0f);
  const float psi_s_deg =
      oneq::common::numerics::Clamp(psi_i_deg + std::fabs(rcs_config.bistatic_psi_offset_deg), 0.0f, 89.0f);

  const float cylinder_rcs_m2 =
      oneq::common::rcs::ComputeCylinderRcs(equivalent_radius_m, wavenumber_k0);
  const float bistatic_rcs_m2 = oneq::common::rcs::ComputeBistaticCylinderRcs(
      wavenumber_k0, equivalent_radius_m, psi_i_deg, psi_s_deg, azimuth_deg);
  const float planar_rcs_m2 =
      oneq::common::rcs::ComputePlanarPlateRcs(wavenumber_k0, equivalent_radius_m, elevation_deg);

  const float cylinder_weight = oneq::common::numerics::Clamp(rcs_config.cylinder_weight, 0.0f, 1.0f);
  const float physical_rcs_m2 = cylinder_weight * (0.5f * (cylinder_rcs_m2 + bistatic_rcs_m2)) +
                                (1.0f - cylinder_weight) * planar_rcs_m2;

  const float min_rcs_m2 = std::max(rcs_config.min_rcs_m2, 0.0f);
  const float max_rcs_m2 = std::max(rcs_config.max_rcs_m2, min_rcs_m2);
  const float clamped_physical_rcs_m2 = oneq::common::numerics::Clamp(physical_rcs_m2, min_rcs_m2, max_rcs_m2);
  return input_rcs_m2 * (1.0f - mix_ratio) + clamped_physical_rcs_m2 * mix_ratio;
}

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
  Eigen::Vector3f pos = geometry.position_m;
  const float pos_norm = pos.norm();
  if (pos_norm > 0.1f) {
    const Eigen::Vector3f u = pos / pos_norm;
    const Eigen::Matrix3f identity = Eigen::Matrix3f::Identity();
    const Eigen::Matrix3f uu_t = u * u.transpose();
    return var_r * uu_t + (range_m * range_m * var_theta) * (identity - uu_t);
  }

  return tracking::MeasurementCovariance::Identity() * var_r;
}

bool HasValidBuffers(const DetectionExecutionBuffers& buffers) {
  return buffers.target_geometry != nullptr && buffers.signal_term_db != nullptr &&
         buffers.speed_penalty_db != nullptr && buffers.detection_margin_db != nullptr &&
         buffers.detection_succeeded != nullptr && buffers.measurement_covariances != nullptr;
}

}  // namespace

void RunHeuristicDetectionPass(const session::ArSceneTargetList& input,
                               const ExecutionConfig& config,
                               const session::ArControlProfile& control_profile,
                               const session::EnvironmentSnapshot& environment_snapshot,
                               DetectionExecutionBuffers* buffers) {
  if (buffers == nullptr || !HasValidBuffers(*buffers)) {
    return;
  }

  const std::size_t count = input.size();
  const float signal_adjustment_db =
      ComputeHeuristicSignalAdjustmentDb(config.control_profile_effects, control_profile);
  for (std::size_t i = 0; i < count; ++i) {
    (*buffers->target_geometry)[i] = detection::TargetGeometryResolver::Resolve(input[i]);
    const float effective_rcs_m2 =
        ComputeEffectiveTargetRcsM2(input[i], (*buffers->target_geometry)[i], config);
    (*buffers->signal_term_db)[i] = effective_rcs_m2 * 6.0f + signal_adjustment_db;
    (*buffers->speed_penalty_db)[i] = ResolveSpeedMagnitude(input[i]) * 0.002f;
  }

  const float jamming_penalty_db =
      ComputeHeuristicJammingPenaltyDb(config.jamming_effects, environment_snapshot);
  const float environment_penalty_db =
      std::max(0.0f, environment_snapshot.propagation_loss_db * 0.2f +
                         environment_snapshot.clutter_power_db * 0.3f + jamming_penalty_db -
                         ComputeHeuristicEnvironmentReliefDb(
                             config.jamming_effects, control_profile, environment_snapshot));
  for (std::size_t i = 0; i < count; ++i) {
    const float margin =
        (*buffers->signal_term_db)[i] - (*buffers->speed_penalty_db)[i] - environment_penalty_db;
    (*buffers->detection_margin_db)[i] = margin;
    (*buffers->detection_succeeded)[i] =
        static_cast<std::uint8_t>(margin >= config.detection.engineering.min_detection_margin_db);
  }
}

void RunPhysicalDetectionPass(const session::ArSceneTargetList& input,
                              const ExecutionConfig& config,
                              const session::ArControlProfile& control_profile,
                              const session::EnvironmentSnapshot& environment_snapshot,
                              float platform_altitude_m, detection::SignalDetector* signal_detector,
                              DetectionExecutionBuffers* buffers) {
  if (signal_detector == nullptr || buffers == nullptr || !HasValidBuffers(*buffers)) {
    return;
  }

  const std::size_t count = input.size();

  float clutter_w = ComputeEquivalentClutterNoiseW(config.detection.engineering,
                                                   environment_snapshot.clutter_power_db);
  if (control_profile.enable_sidelobe_canceller &&
      HasMultiSourceJammingFacts(environment_snapshot)) {
    const bool has_sidelobe_source = std::find_if(environment_snapshot.jammer_sources.begin(),
                                                  environment_snapshot.jammer_sources.end(),
                                                  [](const session::JammerSourceFact& source) {
                                                    return source.in_sidelobe;
                                                  }) != environment_snapshot.jammer_sources.end();
    clutter_w *= has_sidelobe_source ? 0.55f : 0.80f;
  }

  float jam_w = 0.0f;
  if (HasMultiSourceJammingFacts(environment_snapshot)) {
    for (std::size_t i = 0; i < environment_snapshot.jammer_sources.size(); ++i) {
      const session::JammerSourceFact& source = environment_snapshot.jammer_sources[i];
      jam_w += ComputePhysicalSourceJamContributionW(config.jamming_effects, source) *
               ComputeResidualJammerFactor(control_profile, source);
    }
  }

  detection::EnvironmentState env;
  env.propagation_loss_db =
      environment_snapshot.propagation_loss_db - environment_snapshot.atmospheric_physics_loss_db;
  env.clutter_noise_w = clutter_w;
  env.jam_noise_w = jam_w;

  signal_detector->UpdateConfig(config.detection.engineering);

  const float measurement_covariance_inflation = ComputeMeasurementCovarianceInflation(
      config.jamming_effects, control_profile, environment_snapshot);

  constexpr float kSpeedOfLightMps = 299792458.0f;
  const float wavelength_m =
      config.detection.engineering.transmitter.frequency_hz > 0.0f
          ? kSpeedOfLightMps / config.detection.engineering.transmitter.frequency_hz
          : 0.0f;

  for (std::size_t i = 0; i < count; ++i) {
    (*buffers->target_geometry)[i] = detection::TargetGeometryResolver::Resolve(input[i]);
    env.propagation_loss_db =
        environment_snapshot.propagation_loss_db -
        environment_snapshot.atmospheric_physics_loss_db +
        ComputeTargetSpecificAtmosphericLossDb(config, environment_snapshot, platform_altitude_m,
                                               (*buffers->target_geometry)[i]);
    const float effective_rcs_m2 =
        ComputeEffectiveTargetRcsM2(input[i], (*buffers->target_geometry)[i], config);
    detection::TargetReturn target;
    target.rcs_m2 = effective_rcs_m2;
    target.range_m = (*buffers->target_geometry)[i].range_m;
    if (input[i].target_swerling_type >=
            static_cast<int>(config::profiles::SwerlingModel::kSwerling0) &&
        input[i].target_swerling_type <=
            static_cast<int>(config::profiles::SwerlingModel::kSwerling4)) {
      target.swerling_type =
          static_cast<config::profiles::SwerlingModel>(input[i].target_swerling_type);
    } else {
      target.swerling_type = config::profiles::SwerlingModel::kSwerling0;
    }

    const detection::ResolvedBeamState beam_state = detection::BeamControlResolver::Resolve(
        config.detection.engineering.antenna, config.detection.orientation,
        config.detection.platform_attitude_deg, (*buffers->target_geometry)[i].look_angles_deg,
        config::AzimuthElevationDeg{}, wavelength_m);
    const detection::DetectionResult detection_result = signal_detector->Detect(
        target, env, beam_state.one_way_antenna_gain_db, config.detection.engineering.pulse_count);
    const detection::MeasurementErrorState measurement_error =
        detection::MeasurementErrorModel::Compute(
            detection_result.snr_db, beam_state.effective_beamwidth_deg,
            config.detection.engineering.transmitter.bandwidth_hz);

    (*buffers->signal_term_db)[i] = detection_result.snr_db;
    (*buffers->speed_penalty_db)[i] = 0.0f;
    (*buffers->detection_margin_db)[i] = detection_result.snr_db;
    (*buffers->detection_succeeded)[i] =
        static_cast<std::uint8_t>(detection_result.detected ? 1U : 0U);
    (*buffers->measurement_covariances)[i] = BuildMeasurementCovariance(
        (*buffers->target_geometry)[i], measurement_error.range_error_std_m,
        measurement_error.angle_error_std_rad,
        config.tracking.engineering.kalman_measurement_noise_std);
    (*buffers->measurement_covariances)[i] *= measurement_covariance_inflation;
  }
}

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
