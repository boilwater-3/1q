#include "airborne_radar/signal/pipeline/DetectionExecution.h"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>

#include "airborne_radar/config/SignalEngineeringConfig.h"
#include "airborne_radar/signal/detection/ArDetectionCellResolver.h"
#include "airborne_radar/signal/detection/BeamControlResolver.h"
#include "airborne_radar/signal/detection/MeasurementErrorModel.h"
#include "airborne_radar/signal/detection/SignalDetector.h"
#include "airborne_radar/signal/pipeline/PipelineTargetUtils.h"
#include "common/atmosphere/AtmospherePhysics.h"
#include "common/rcs/RcsPhysics.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

namespace {

float ComputeEquivalentClutterNoiseW(const config::engineering::DetectionConfig& detection_config,
                                     float clutter_power_db) {
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
    const ExecutionConfig& exec_config, const session::EnvironmentSnapshot& environment_snapshot,
    float platform_altitude_m, const detection::ResolvedTargetGeometry& geometry) {
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
      exec_config.detection.engineering.transmitter.frequency_hz, std::max(geometry.range_m, 0.1f),
      platform_altitude_m, std::max(platform_altitude_m + geometry.position_m.z(), 0.0f),
      geometry.look_angles_deg.has_look_angles ? geometry.look_angles_deg.look_el_deg : 0.0f, obs);
  return oneq::common::atmosphere::EvaluateAtmosphericPropagation(inputs).total_physics_loss_db;
}

float ComputeEquivalentRadiusM(float input_rcs_m2,
                               const config::engineering::RcsPhysicsConfig& rcs_config) {
  const float min_radius_m = std::max(rcs_config.min_equivalent_radius_m, 1.0e-3f);
  const float max_radius_m = std::max(rcs_config.max_equivalent_radius_m, min_radius_m);
  const float safe_input_rcs_m2 = std::max(input_rcs_m2, 0.0f);
  const float equivalent_radius_m =
      std::sqrt(safe_input_rcs_m2 / static_cast<float>(oneq::common::numerics::kPi));
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

  const float frequency_hz = exec_config.detection.engineering.transmitter.frequency_hz;
  if (frequency_hz <= 0.0f) {
    return input_rcs_m2;
  }

  const float wavenumber_k0 = 2.0f * static_cast<float>(oneq::common::numerics::kPi) *
                              frequency_hz /
                              static_cast<float>(oneq::common::numerics::kLightSpeed);
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
  const float psi_s_deg = oneq::common::numerics::Clamp(
      psi_i_deg + std::fabs(rcs_config.bistatic_psi_offset_deg), 0.0f, 89.0f);

  const float cylinder_rcs_m2 =
      oneq::common::rcs::ComputeCylinderRcs(equivalent_radius_m, wavenumber_k0);
  const float bistatic_rcs_m2 = oneq::common::rcs::ComputeBistaticCylinderRcs(
      wavenumber_k0, equivalent_radius_m, psi_i_deg, psi_s_deg, azimuth_deg);
  const float planar_rcs_m2 =
      oneq::common::rcs::ComputePlanarPlateRcs(wavenumber_k0, equivalent_radius_m, elevation_deg);

  const float cylinder_weight =
      oneq::common::numerics::Clamp(rcs_config.cylinder_weight, 0.0f, 1.0f);
  const float physical_rcs_m2 = cylinder_weight * (0.5f * (cylinder_rcs_m2 + bistatic_rcs_m2)) +
                                (1.0f - cylinder_weight) * planar_rcs_m2;

  const float min_rcs_m2 = std::max(rcs_config.min_rcs_m2, 0.0f);
  const float max_rcs_m2 = std::max(rcs_config.max_rcs_m2, min_rcs_m2);
  const float clamped_physical_rcs_m2 =
      oneq::common::numerics::Clamp(physical_rcs_m2, min_rcs_m2, max_rcs_m2);
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

bool RunPhysicalDetectionPass(const session::ArSceneTargetList& input,
                              const ExecutionConfig& config,
                              const session::EnvironmentSnapshot& environment_snapshot,
                              float platform_altitude_m,
                              const RfV2DetectionContext* rf_v2_detection_context,
                              detection::SignalDetector* signal_detector,
                              DetectionExecutionBuffers* buffers) {
  if (buffers == nullptr || !HasValidBuffers(*buffers)) {
    return false;
  }
  if (signal_detector == nullptr) {
    return true;
  }

  const std::size_t count = input.size();

  float clutter_w = ComputeEquivalentClutterNoiseW(config.detection.engineering,
                                                   environment_snapshot.clutter_power_db);
  detection::EnvironmentState env;
  env.propagation_loss_db =
      environment_snapshot.propagation_loss_db - environment_snapshot.atmospheric_physics_loss_db;
  env.clutter_noise_w = clutter_w;
  env.jam_noise_w = 0.0f;  // RF v2 interference is resolved in the detection-cell path below.

  signal_detector->UpdateConfig(config.detection.engineering);

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

    const detection::ResolvedBeamState beam_state =
        rf_v2_detection_context == nullptr
            ? detection::BeamControlResolver::Resolve(
                  config.detection.engineering.antenna, config.detection.orientation,
                  config.detection.platform_attitude_deg,
                  (*buffers->target_geometry)[i].look_angles_deg,
                  config::AzimuthElevationDeg{}, wavelength_m)
            : detection::BeamControlResolver::ResolveFrozen(
                  config.detection.engineering.antenna, config.detection.orientation,
                  (*buffers->target_geometry)[i].look_angles_deg,
                  rf_v2_detection_context->beam_pointing_deg, wavelength_m);
    detection::DetectionResult detection_result;
    if (rf_v2_detection_context == nullptr) {
      detection_result = signal_detector->Detect(target, env, beam_state.one_way_antenna_gain_db,
                                                 config.detection.engineering.pulse_count);
    } else {
      const Eigen::Vector3f& position = (*buffers->target_geometry)[i].position_m;
      const float position_norm = position.norm();
      if (!std::isfinite(position_norm) || position_norm <= 0.0f) {
        return false;
      }
      const Eigen::Vector3f relative_velocity(input[i].velocity_x, input[i].velocity_y,
                                              input[i].velocity_z);
      const double closing_radial_velocity_mps =
          -static_cast<double>(relative_velocity.dot(position / position_norm));
      const double two_way_propagation_loss_db = static_cast<double>(env.propagation_loss_db);
      if (!std::isfinite(two_way_propagation_loss_db) || two_way_propagation_loss_db < 0.0) {
        return false;
      }
      detection::ArDetectionCellConfig cell_config;
      cell_config.own_transmit_waveform = rf_v2_detection_context->own_transmit_waveform;
      cell_config.receive_window_start_time_s =
          rf_v2_detection_context->receive_window_start_time_s;
      cell_config.receive_window_duration_s =
          rf_v2_detection_context->receive_window_duration_s;
      cell_config.matched_filter_bandwidth_hz =
          rf_v2_detection_context->own_transmit_waveform.occupied_bandwidth_hz;
      cell_config.one_way_antenna_gain_dbi =
          static_cast<double>(beam_state.one_way_antenna_gain_db);
      cell_config.receiver_loss_db =
          static_cast<double>(config.detection.engineering.receiver.receive_loss_db);
      cell_config.receiver_noise_figure_db =
          static_cast<double>(config.detection.engineering.receiver.noise_figure_db);
      detection::ArDetectionCellTarget cell_target;
      cell_target.range_m = static_cast<double>((*buffers->target_geometry)[i].range_m);
      cell_target.closing_radial_velocity_mps = closing_radial_velocity_mps;
      cell_target.rcs_m2 = static_cast<double>(effective_rcs_m2);
      cell_target.two_way_additional_propagation_loss_db = two_way_propagation_loss_db;
      cell_target.effective_pulse_count = static_cast<std::uint32_t>(
          std::max(1, config.detection.engineering.pulse_count));
      detection::ArDetectionCellResult cell_result;
      if (!detection::TryResolveArDetectionCell(
              cell_config, cell_target, rf_v2_detection_context->own_emission_identity,
              rf_v2_detection_context->incident_links, static_cast<double>(clutter_w),
              &cell_result)) {
        return false;
      }
      detection_result = signal_detector->DetectResolvedCell(target, cell_result);
    }
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
  }
  return true;
}

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
