#include "airborne_radar/session/RadarReplayFlatbufferCodec.h"

#include <cstdint>
#include <string>
#include <vector>

#include "1q/airborne_radar/model/TrackStateSnapshot.h"
#include "airborne_radar/session/generated/airborne_radar_replay_generated.h"
#include "airborne_radar/session/generated/airborne_radar_scene_replay_generated.h"
#include "airborne_radar/session/generated/airborne_radar_session_replay_generated.h"

namespace airborne_radar {
namespace session {
namespace {

namespace fb = oneq::replay::airborne_radar::fb;
namespace scene_fb = oneq::replay::airborne_radar::scene::fb;
namespace session_fb = oneq::replay::airborne_radar::session::fb;

std::size_t CountTracksByStatus(const model::TrackStateSnapshotList& tracks, model::TrackStatus status) {
  std::size_t count = 0U;
  for (std::size_t i = 0; i < tracks.size(); ++i) {
    if (tracks[i].status == status) {
      ++count;
    }
  }
  return count;
}

flatbuffers::Offset<fb::Vector3f> EncodeVector3(flatbuffers::FlatBufferBuilder* builder,
                                                const oneq::foundation::Vector3f& value) {
  return fb::CreateVector3f(*builder, value.x, value.y, value.z);
}

flatbuffers::Offset<fb::Vector3f> EncodeVector3(flatbuffers::FlatBufferBuilder* builder, float x,
                                                float y, float z) {
  return fb::CreateVector3f(*builder, x, y, z);
}

flatbuffers::Offset<fb::EulerAnglesDeg> EncodeEulerAngles(
    flatbuffers::FlatBufferBuilder* builder, const oneq::foundation::EulerAnglesDeg& value) {
  return fb::CreateEulerAnglesDeg(*builder, value.yaw_deg, value.pitch_deg, value.roll_deg);
}

flatbuffers::Offset<fb::PoseState> EncodePoseState(flatbuffers::FlatBufferBuilder* builder,
                                                   const oneq::foundation::PoseState& value) {
  return fb::CreatePoseState(*builder, EncodeVector3(builder, value.position_m),
                             EncodeVector3(builder, value.velocity_mps),
                             EncodeEulerAngles(builder, value.attitude_deg));
}

flatbuffers::Offset<fb::TargetFeature> EncodeTargetFeature(flatbuffers::FlatBufferBuilder* builder,
                                                           const RadarSceneTarget& value) {
  return fb::CreateTargetFeature(
      *builder, value.external_target_id,
      EncodeVector3(builder, value.current_track_velocity_x, value.current_track_velocity_y,
                    value.current_track_velocity_z),
      value.current_track_speed, value.current_track_rcs, value.range_m,
      value.has_cartesian_position,
      EncodeVector3(builder, value.position_x, value.position_y, value.position_z),
      value.target_swerling_type);
}

oneq::foundation::Vector3f DecodeVector3(const fb::Vector3f* value) {
  oneq::foundation::Vector3f result;
  if (value != nullptr) {
    result.x = value->x();
    result.y = value->y();
    result.z = value->z();
  }
  return result;
}

oneq::foundation::EulerAnglesDeg DecodeEulerAngles(const fb::EulerAnglesDeg* value) {
  oneq::foundation::EulerAnglesDeg result;
  if (value != nullptr) {
    result.yaw_deg = value->yaw_deg();
    result.pitch_deg = value->pitch_deg();
    result.roll_deg = value->roll_deg();
  }
  return result;
}

oneq::foundation::PoseState DecodePoseState(const fb::PoseState* value) {
  oneq::foundation::PoseState result;
  if (value != nullptr) {
    result.position_m = DecodeVector3(value->position_m());
    result.velocity_mps = DecodeVector3(value->velocity_mps());
    result.attitude_deg = DecodeEulerAngles(value->attitude_deg());
  }
  return result;
}

RadarSceneTarget DecodeTargetFeature(const fb::TargetFeature* value) {
  RadarSceneTarget result;
  if (value != nullptr) {
    result.external_target_id = value->external_target_id();
    const oneq::foundation::Vector3f velocity = DecodeVector3(value->velocity_mps());
    result.current_track_velocity_x = velocity.x;
    result.current_track_velocity_y = velocity.y;
    result.current_track_velocity_z = velocity.z;
    result.current_track_speed = value->current_track_speed();
    result.current_track_rcs = value->current_track_rcs();
    result.range_m = value->range_m();
    result.has_cartesian_position = value->has_cartesian_position();
    const oneq::foundation::Vector3f position = DecodeVector3(value->position_m());
    result.position_x = position.x;
    result.position_y = position.y;
    result.position_z = position.z;
    result.target_swerling_type = value->target_swerling_type();
  }
  return result;
}

flatbuffers::Offset<fb::DecisionTrackStateSnapshot> EncodeTrackStateSnapshot(
    flatbuffers::FlatBufferBuilder* builder, const model::TrackStateSnapshot& value) {
  return fb::CreateDecisionTrackStateSnapshot(
      *builder, value.association_key, value.external_target_id,
      static_cast<int>(value.status), value.position_x, value.position_y, value.position_z,
      value.velocity_x, value.velocity_y, value.velocity_z, value.speed,
      value.acceleration_x, value.acceleration_y, value.acceleration_z, value.acceleration,
      value.rcs, value.jamming_detected, value.hit_count, value.miss_count);
}

flatbuffers::Offset<fb::TrackStateSnapshot> EncodeTrackSnapshot(
    flatbuffers::FlatBufferBuilder* builder, const model::TrackStateSnapshot& value) {
  return fb::CreateTrackStateSnapshot(*builder, EncodeTrackStateSnapshot(builder, value));
}

model::TrackStateSnapshot DecodeTrackStateSnapshot(
    const fb::DecisionTrackStateSnapshot* value) {
  model::TrackStateSnapshot result;
  if (value != nullptr) {
    result.association_key = value->association_key();
    result.external_target_id = value->external_target_id();
    result.status = static_cast<model::TrackStatus>(value->status());
    result.position_x = value->position_x();
    result.position_y = value->position_y();
    result.position_z = value->position_z();
    result.velocity_x = value->velocity_x();
    result.velocity_y = value->velocity_y();
    result.velocity_z = value->velocity_z();
    result.speed = value->speed();
    result.acceleration_x = value->acceleration_x();
    result.acceleration_y = value->acceleration_y();
    result.acceleration_z = value->acceleration_z();
    result.acceleration = value->acceleration();
    result.rcs = value->rcs();
    result.jamming_detected = value->jamming_detected();
    result.hit_count = value->hit_count();
    result.miss_count = value->miss_count();
  }
  return result;
}

model::TrackStateSnapshot DecodeTrackSnapshot(const fb::TrackStateSnapshot* value) {
  model::TrackStateSnapshot result;
  if (value != nullptr) {
    result = DecodeTrackStateSnapshot(value->state());
  }
  return result;
}

flatbuffers::Offset<fb::TrackOutputFrame> EncodeTrackOutputFrame(
    flatbuffers::FlatBufferBuilder* builder, const output::TrackOutputFrame& value) {
  std::vector<flatbuffers::Offset<fb::TrackStateSnapshot>> track_offsets;
  track_offsets.reserve(value.tracks.size());
  for (std::size_t i = 0; i < value.tracks.size(); ++i) {
    track_offsets.push_back(EncodeTrackSnapshot(builder, value.tracks[i]));
  }
  const flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<fb::TrackStateSnapshot>>>
      tracks_vec = builder->CreateVector(track_offsets);
  return fb::CreateTrackOutputFrame(
      *builder, value.cycle_index, static_cast<std::uint64_t>(value.tracks.size()),
      value.batch_id,
      static_cast<std::uint64_t>(CountTracksByStatus(value.tracks, model::TrackStatus::kConfirmed)),
      CountTracksByStatus(value.tracks, model::TrackStatus::kLost) > 0U, tracks_vec);
}

output::TrackOutputFrame DecodeTrackOutputFrame(const fb::TrackOutputFrame* value) {
  output::TrackOutputFrame result;
  if (value != nullptr) {
    result.cycle_index = value->cycle_index();
    result.batch_id = value->batch_id();
    const flatbuffers::Vector<flatbuffers::Offset<fb::TrackStateSnapshot>>* tracks =
        value->tracks();
    if (tracks != nullptr) {
      result.tracks.reserve(tracks->size());
      for (flatbuffers::uoffset_t i = 0; i < tracks->size(); ++i) {
        result.tracks.push_back(DecodeTrackSnapshot(tracks->Get(i)));
      }
    }
  }
  return result;
}

flatbuffers::Offset<fb::RadarCycleResult> EncodeCycleResult(flatbuffers::FlatBufferBuilder* builder,
                                                            const RadarCycleResult& value) {
  return fb::CreateRadarCycleResult(
      *builder, EncodeTrackOutputFrame(builder, value.track_output_frame),
      static_cast<std::uint32_t>(value.validation_issues.size()), value.executed_this_cycle);
}

RadarCycleResult DecodeCycleResult(const fb::RadarCycleResult* value) {
  RadarCycleResult result;
  if (value != nullptr) {
    result.track_output_frame = DecodeTrackOutputFrame(value->track_output_frame());
    result.validation_issues.resize(value->validation_issue_count());
    result.executed_this_cycle = value->executed_this_cycle();
  }
  return result;
}

flatbuffers::Offset<scene_fb::AtmosphericPhysicsConfig> EncodeAtmosphericPhysicsConfig(
    flatbuffers::FlatBufferBuilder* builder, const environment::AtmosphericPhysicsConfig& value) {
  return scene_fb::CreateAtmosphericPhysicsConfig(*builder, value.enable_physical_model,
                                                  value.pressure_hpa, value.temperature_k,
                                                  value.relative_humidity);
}

flatbuffers::Offset<scene_fb::AtmosphericDerivedContext> EncodeAtmosphericDerivedContext(
    flatbuffers::FlatBufferBuilder* builder, const environment::AtmosphericDerivedContext& value) {
  return scene_fb::CreateAtmosphericDerivedContext(
      *builder, value.has_simulation_unix_seconds, value.simulation_unix_seconds,
      value.solar_flux_f107a, value.solar_flux_f107, value.geomagnetic_ap);
}

flatbuffers::Offset<scene_fb::VegetationScatterPhysicsConfig> EncodeVegetationScatterPhysicsConfig(
    flatbuffers::FlatBufferBuilder* builder,
    const environment::VegetationScatterPhysicsConfig& value) {
  return scene_fb::CreateVegetationScatterPhysicsConfig(
      *builder, static_cast<int>(value.cover_profile), value.enable_physical_model);
}

flatbuffers::Offset<scene_fb::JammerEmitterState> EncodeJammerEmitterState(
    flatbuffers::FlatBufferBuilder* builder, const environment::JammerEmitterState& value) {
  return scene_fb::CreateJammerEmitterState(*builder, static_cast<int>(value.technique),
                                            value.power_db, value.js_db, value.has_direction_deg,
                                            value.azimuth_deg, value.elevation_deg,
                                            value.angular_span_deg, value.confidence);
}

environment::AtmosphericPhysicsConfig DecodeAtmosphericPhysicsConfig(
    const scene_fb::AtmosphericPhysicsConfig* value) {
  environment::AtmosphericPhysicsConfig result;
  if (value != nullptr) {
    result.enable_physical_model = value->enable_physical_model();
    result.pressure_hpa = value->pressure_hpa();
    result.temperature_k = value->temperature_k();
    result.relative_humidity = value->relative_humidity();
  }
  return result;
}

environment::AtmosphericDerivedContext DecodeAtmosphericDerivedContext(
    const scene_fb::AtmosphericDerivedContext* value) {
  environment::AtmosphericDerivedContext result;
  if (value != nullptr) {
    result.has_simulation_unix_seconds = value->has_simulation_unix_seconds();
    result.simulation_unix_seconds = value->simulation_unix_seconds();
    result.solar_flux_f107a = value->solar_flux_f107a();
    result.solar_flux_f107 = value->solar_flux_f107();
    result.geomagnetic_ap = value->geomagnetic_ap();
  }
  return result;
}

environment::VegetationScatterPhysicsConfig DecodeVegetationScatterPhysicsConfig(
    const scene_fb::VegetationScatterPhysicsConfig* value) {
  environment::VegetationScatterPhysicsConfig result;
  if (value != nullptr) {
    result.cover_profile = static_cast<environment::VegetationCoverProfile>(value->cover_profile());
    result.enable_physical_model = value->enable_physical_model();
  }
  return result;
}

environment::JammerEmitterState DecodeJammerEmitterState(
    const scene_fb::JammerEmitterState* value) {
  environment::JammerEmitterState result;
  if (value != nullptr) {
    result.technique = static_cast<environment::JammingTechnique>(value->technique());
    result.power_db = value->power_db();
    result.js_db = value->js_db();
    result.has_direction_deg = value->has_direction_deg();
    result.azimuth_deg = value->azimuth_deg();
    result.elevation_deg = value->elevation_deg();
    result.angular_span_deg = value->angular_span_deg();
    result.confidence = value->confidence();
  }
  return result;
}

flatbuffers::Offset<session_fb::EulerAnglesDeg> EncodeSessionEulerAngles(
    flatbuffers::FlatBufferBuilder* builder, const model::EulerAnglesDeg& value) {
  return session_fb::CreateEulerAnglesDeg(*builder, value.yaw_deg, value.pitch_deg, value.roll_deg);
}

flatbuffers::Offset<session_fb::AzimuthElevationDeg> EncodeSessionAzEl(
    flatbuffers::FlatBufferBuilder* builder, const model::AzimuthElevationDeg& value) {
  return session_fb::CreateAzimuthElevationDeg(*builder, value.az_deg, value.el_deg);
}

flatbuffers::Offset<session_fb::AzimuthElevationLimitsDeg> EncodeSessionAzElLimits(
    flatbuffers::FlatBufferBuilder* builder, const model::AzimuthElevationLimitsDeg& value) {
  return session_fb::CreateAzimuthElevationLimitsDeg(*builder, value.az_min_deg, value.az_max_deg,
                                                     value.el_min_deg, value.el_max_deg);
}

flatbuffers::Offset<session_fb::CommandedBeamwidthDeg> EncodeSessionCommandedBeamwidth(
    flatbuffers::FlatBufferBuilder* builder, const model::CommandedBeamwidthDeg& value) {
  return session_fb::CreateCommandedBeamwidthDeg(*builder, value.commanded_az_beamwidth_deg,
                                                 value.commanded_el_beamwidth_deg);
}

flatbuffers::Offset<session_fb::RadarOrientationConfig> EncodeSessionOrientation(
    flatbuffers::FlatBufferBuilder* builder, const model::RadarOrientationConfig& value) {
  return session_fb::CreateRadarOrientationConfig(
      *builder, EncodeSessionEulerAngles(builder, value.mount_angles_deg),
      EncodeSessionAzEl(builder, value.scan_center_deg),
      EncodeSessionAzElLimits(builder, value.mechanical_scan_limits_deg),
      EncodeSessionAzElLimits(builder, value.electronic_scan_limits_deg),
      static_cast<int>(value.scan_start_position), static_cast<int>(value.scan_sequence),
      static_cast<int>(value.work_sub_mode), value.commanded_beamwidth_enabled,
      EncodeSessionCommandedBeamwidth(builder, value.commanded_beamwidth_deg),
      static_cast<int>(value.stabilization_mode));
}

flatbuffers::Offset<session_fb::DetectionConfig> EncodeSessionDetectionConfig(
    flatbuffers::FlatBufferBuilder* builder, const config::DetectionConfig& value) {
  const flatbuffers::Offset<session_fb::TransmitterConfig> transmitter =
      session_fb::CreateTransmitterConfig(
          *builder, value.transmitter.peak_power_w, value.transmitter.frequency_hz,
          value.transmitter.bandwidth_hz, value.transmitter.pulse_width_s, value.transmitter.prf_hz,
          value.transmitter.transmit_loss_db);
  const flatbuffers::Offset<session_fb::AntennaPatternConfig> pattern =
      session_fb::CreateAntennaPatternConfig(
          *builder, static_cast<int>(value.antenna.pattern.model_type),
          value.antenna.pattern.max_sidelobe_level_db, value.antenna.pattern.backlobe_level_db,
          value.antenna.pattern.scan_loss_coeff_db_per_deg2, value.antenna.pattern.max_scan_loss_db,
          EncodeSessionAzEl(builder, value.antenna.pattern.boresight_offset_deg));
  const flatbuffers::Offset<session_fb::AntennaConfig> antenna = session_fb::CreateAntennaConfig(
      *builder, value.antenna.main_beam_gain_db, value.antenna.nominal_az_beamwidth_deg,
      value.antenna.nominal_el_beamwidth_deg, value.antenna.enable_directional_pattern, pattern);
  const flatbuffers::Offset<session_fb::ReceiverConfig> receiver = session_fb::CreateReceiverConfig(
      *builder, value.receiver.noise_figure_db, value.receiver.receive_loss_db);
  const flatbuffers::Offset<session_fb::DetectionPolicyConfig> policy =
      session_fb::CreateDetectionPolicyConfig(*builder, value.detection_policy.cfar_pfa,
                                              value.detection_policy.min_snr_db);
  const flatbuffers::Offset<session_fb::RcsPhysicsConfig> rcs_physics =
      session_fb::CreateRcsPhysicsConfig(
          *builder, value.rcs_physics.enable_physical_rcs, value.rcs_physics.frequency_hz,
          value.rcs_physics.physics_mix_ratio, value.rcs_physics.cylinder_weight,
          value.rcs_physics.min_equivalent_radius_m, value.rcs_physics.max_equivalent_radius_m,
          value.rcs_physics.min_rcs_m2, value.rcs_physics.max_rcs_m2,
          value.rcs_physics.bistatic_psi_offset_deg);
  return session_fb::CreateDetectionConfig(
      *builder, value.enable_physics_detection, static_cast<int>(value.swerling_model), transmitter,
      antenna, receiver, policy, rcs_physics, value.min_detection_margin_db, value.pulse_count);
}

flatbuffers::Offset<session_fb::RadarPolicyConfig> EncodeSessionPolicyConfig(
    flatbuffers::FlatBufferBuilder* builder, const config::RadarPolicyConfig& value) {
  const flatbuffers::Offset<session_fb::BeamPointingConfig> pointing =
      session_fb::CreateBeamPointingConfig(
          *builder, EncodeSessionAzEl(builder, value.beam_control.pointing.default_scan_center_deg),
          EncodeSessionCommandedBeamwidth(builder,
                                          value.beam_control.pointing.nominal_beamwidth_deg));
  const flatbuffers::Offset<session_fb::BeamSchedulerConfig> scheduler =
      session_fb::CreateBeamSchedulerConfig(*builder,
                                            value.beam_control.scheduler.azimuth_step_count_hint,
                                            value.beam_control.scheduler.elevation_step_count_hint,
                                            value.beam_control.scheduler.prefer_dense_tas_sampling);
  const flatbuffers::Offset<session_fb::BeamControlConfig> beam_control =
      session_fb::CreateBeamControlConfig(*builder, pointing, scheduler);
  const flatbuffers::Offset<session_fb::AssociationConfig> association =
      session_fb::CreateAssociationConfig(*builder, value.association.unassigned_cost,
                                          value.association.use_distance_gate_hint,
                                          value.association.distance_gate_sigma_hint);
  const flatbuffers::Offset<session_fb::TrackingConfig> tracking = session_fb::CreateTrackingConfig(
      *builder, value.tracking.enable_kalman_filter, value.tracking.kalman_measurement_noise_std,
      static_cast<int>(value.tracking.kalman_update_backend),
      value.tracking.speed_decay_ratio_on_loss, value.tracking.rcs_decay_ratio_on_loss);
  const flatbuffers::Offset<session_fb::LifecycleConfig> lifecycle =
      session_fb::CreateLifecycleConfig(
          *builder, value.lifecycle.confirm_hits, value.lifecycle.max_miss_before_lost,
          value.lifecycle.max_lost_cycles, value.lifecycle.enable_imm_lifecycle);
  const flatbuffers::Offset<session_fb::ImmConfig> imm = session_fb::CreateImmConfig(
      *builder, value.imm.enable_imm_lifecycle, value.imm.model_count_hint);
  return session_fb::CreateRadarPolicyConfig(*builder, beam_control, association, tracking,
                                             lifecycle, imm);
}

flatbuffers::Offset<session_fb::AtmosphericPhysicsConfig> EncodeSessionAtmosphericPhysicsConfig(
    flatbuffers::FlatBufferBuilder* builder, const environment::AtmosphericPhysicsConfig& value) {
  return session_fb::CreateAtmosphericPhysicsConfig(*builder, value.enable_physical_model,
                                                    value.pressure_hpa, value.temperature_k,
                                                    value.relative_humidity);
}

flatbuffers::Offset<session_fb::AtmosphericDerivedContext> EncodeSessionAtmosphericDerivedContext(
    flatbuffers::FlatBufferBuilder* builder, const environment::AtmosphericDerivedContext& value) {
  return session_fb::CreateAtmosphericDerivedContext(
      *builder, value.has_simulation_unix_seconds, value.simulation_unix_seconds,
      value.solar_flux_f107a, value.solar_flux_f107, value.geomagnetic_ap);
}

flatbuffers::Offset<session_fb::VegetationScatterPhysicsConfig>
EncodeSessionVegetationScatterPhysicsConfig(
    flatbuffers::FlatBufferBuilder* builder,
    const environment::VegetationScatterPhysicsConfig& value) {
  return session_fb::CreateVegetationScatterPhysicsConfig(
      *builder, static_cast<int>(value.cover_profile), value.enable_physical_model);
}

flatbuffers::Offset<session_fb::JammerEmitterState> EncodeSessionJammerEmitterState(
    flatbuffers::FlatBufferBuilder* builder, const environment::JammerEmitterState& value) {
  return session_fb::CreateJammerEmitterState(*builder, static_cast<int>(value.technique),
                                              value.power_db, value.js_db, value.has_direction_deg,
                                              value.azimuth_deg, value.elevation_deg,
                                              value.angular_span_deg, value.confidence);
}

flatbuffers::Offset<session_fb::EnvironmentScenarioConfig> EncodeSessionEnvironmentScenarioConfig(
    flatbuffers::FlatBufferBuilder* builder, const environment::EnvironmentScenarioConfig& value) {
  std::vector<flatbuffers::Offset<session_fb::JammerEmitterState>> jammer_sources;
  jammer_sources.reserve(value.jammer_sources.size());
  for (std::size_t i = 0; i < value.jammer_sources.size(); ++i) {
    jammer_sources.push_back(EncodeSessionJammerEmitterState(builder, value.jammer_sources[i]));
  }

  const flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<session_fb::JammerEmitterState>>>
      source_vector = builder->CreateVector(jammer_sources);
  return session_fb::CreateEnvironmentScenarioConfig(
      *builder, EncodeSessionAtmosphericPhysicsConfig(builder, value.atmospheric_physics),
      EncodeSessionAtmosphericDerivedContext(builder, value.atmospheric_context),
      EncodeSessionVegetationScatterPhysicsConfig(builder, value.vegetation_scatter_physics),
      source_vector);
}

flatbuffers::Offset<session_fb::EnvironmentRuntimeConfigPatch>
EncodeSessionEnvironmentRuntimeConfigPatch(
    flatbuffers::FlatBufferBuilder* builder,
    const environment::EnvironmentRuntimeConfigPatch& value) {
  return session_fb::CreateEnvironmentRuntimeConfigPatch(
      *builder, value.has_scenario_config,
      EncodeSessionEnvironmentScenarioConfig(builder, value.scenario_config),
      value.has_jamming_sensitivity_profile, static_cast<int>(value.jamming_sensitivity_profile));
}

model::EulerAnglesDeg DecodeSessionEulerAngles(const session_fb::EulerAnglesDeg* value) {
  model::EulerAnglesDeg result;
  if (value != nullptr) {
    result.yaw_deg = value->yaw_deg();
    result.pitch_deg = value->pitch_deg();
    result.roll_deg = value->roll_deg();
  }
  return result;
}

model::AzimuthElevationDeg DecodeSessionAzEl(const session_fb::AzimuthElevationDeg* value) {
  model::AzimuthElevationDeg result;
  if (value != nullptr) {
    result.az_deg = value->az_deg();
    result.el_deg = value->el_deg();
  }
  return result;
}

model::AzimuthElevationLimitsDeg DecodeSessionAzElLimits(
    const session_fb::AzimuthElevationLimitsDeg* value) {
  model::AzimuthElevationLimitsDeg result;
  if (value != nullptr) {
    result.az_min_deg = value->az_min_deg();
    result.az_max_deg = value->az_max_deg();
    result.el_min_deg = value->el_min_deg();
    result.el_max_deg = value->el_max_deg();
  }
  return result;
}

model::CommandedBeamwidthDeg DecodeSessionCommandedBeamwidth(
    const session_fb::CommandedBeamwidthDeg* value) {
  model::CommandedBeamwidthDeg result;
  if (value != nullptr) {
    result.commanded_az_beamwidth_deg = value->commanded_az_beamwidth_deg();
    result.commanded_el_beamwidth_deg = value->commanded_el_beamwidth_deg();
  }
  return result;
}

model::RadarOrientationConfig DecodeSessionOrientation(
    const session_fb::RadarOrientationConfig* value) {
  model::RadarOrientationConfig result;
  if (value != nullptr) {
    result.mount_angles_deg = DecodeSessionEulerAngles(value->mount_angles_deg());
    result.scan_center_deg = DecodeSessionAzEl(value->scan_center_deg());
    result.mechanical_scan_limits_deg =
        DecodeSessionAzElLimits(value->mechanical_scan_limits_deg());
    result.electronic_scan_limits_deg =
        DecodeSessionAzElLimits(value->electronic_scan_limits_deg());
    result.scan_start_position =
        static_cast<oneq::foundation::ScanStartPosition>(value->scan_start_position());
    result.scan_sequence = static_cast<oneq::foundation::ScanSequence>(value->scan_sequence());
    result.work_sub_mode = static_cast<model::RadarWorkSubMode>(value->work_sub_mode());
    result.commanded_beamwidth_enabled = value->commanded_beamwidth_enabled();
    result.commanded_beamwidth_deg =
        DecodeSessionCommandedBeamwidth(value->commanded_beamwidth_deg());
    result.stabilization_mode = static_cast<model::StabilizationMode>(value->stabilization_mode());
  }
  return result;
}

config::DetectionConfig DecodeSessionDetectionConfig(const session_fb::DetectionConfig* value) {
  config::DetectionConfig result;
  if (value != nullptr) {
    result.enable_physics_detection = value->enable_physics_detection();
    result.swerling_model = static_cast<config::profiles::SwerlingModel>(value->swerling_model());
    const session_fb::TransmitterConfig* transmitter = value->transmitter();
    if (transmitter != nullptr) {
      result.transmitter.peak_power_w = transmitter->peak_power_w();
      result.transmitter.frequency_hz = transmitter->frequency_hz();
      result.transmitter.bandwidth_hz = transmitter->bandwidth_hz();
      result.transmitter.pulse_width_s = transmitter->pulse_width_s();
      result.transmitter.prf_hz = transmitter->prf_hz();
      result.transmitter.transmit_loss_db = transmitter->transmit_loss_db();
    }
    const session_fb::AntennaConfig* antenna = value->antenna();
    if (antenna != nullptr) {
      result.antenna.main_beam_gain_db = antenna->main_beam_gain_db();
      result.antenna.nominal_az_beamwidth_deg = antenna->nominal_az_beamwidth_deg();
      result.antenna.nominal_el_beamwidth_deg = antenna->nominal_el_beamwidth_deg();
      result.antenna.enable_directional_pattern = antenna->enable_directional_pattern();
      const session_fb::AntennaPatternConfig* pattern = antenna->pattern();
      if (pattern != nullptr) {
        result.antenna.pattern.model_type =
            static_cast<config::detection::AntennaPatternModelType>(pattern->model_type());
        result.antenna.pattern.max_sidelobe_level_db = pattern->max_sidelobe_level_db();
        result.antenna.pattern.backlobe_level_db = pattern->backlobe_level_db();
        result.antenna.pattern.scan_loss_coeff_db_per_deg2 = pattern->scan_loss_coeff_db_per_deg2();
        result.antenna.pattern.max_scan_loss_db = pattern->max_scan_loss_db();
        result.antenna.pattern.boresight_offset_deg =
            DecodeSessionAzEl(pattern->boresight_offset_deg());
      }
    }
    const session_fb::ReceiverConfig* receiver = value->receiver();
    if (receiver != nullptr) {
      result.receiver.noise_figure_db = receiver->noise_figure_db();
      result.receiver.receive_loss_db = receiver->receive_loss_db();
    }
    const session_fb::DetectionPolicyConfig* policy = value->detection_policy();
    if (policy != nullptr) {
      result.detection_policy.cfar_pfa = policy->cfar_pfa();
      result.detection_policy.min_snr_db = policy->min_snr_db();
    }
    const session_fb::RcsPhysicsConfig* rcs_physics = value->rcs_physics();
    if (rcs_physics != nullptr) {
      result.rcs_physics.enable_physical_rcs = rcs_physics->enable_physical_rcs();
      result.rcs_physics.frequency_hz = rcs_physics->frequency_hz();
      result.rcs_physics.physics_mix_ratio = rcs_physics->physics_mix_ratio();
      result.rcs_physics.cylinder_weight = rcs_physics->cylinder_weight();
      result.rcs_physics.min_equivalent_radius_m = rcs_physics->min_equivalent_radius_m();
      result.rcs_physics.max_equivalent_radius_m = rcs_physics->max_equivalent_radius_m();
      result.rcs_physics.min_rcs_m2 = rcs_physics->min_rcs_m2();
      result.rcs_physics.max_rcs_m2 = rcs_physics->max_rcs_m2();
      result.rcs_physics.bistatic_psi_offset_deg = rcs_physics->bistatic_psi_offset_deg();
    }
    result.min_detection_margin_db = value->min_detection_margin_db();
    result.pulse_count = value->pulse_count();
  }
  return result;
}

config::RadarPolicyConfig DecodeSessionPolicyConfig(const session_fb::RadarPolicyConfig* value) {
  config::RadarPolicyConfig result;
  if (value != nullptr) {
    const session_fb::BeamControlConfig* beam_control = value->beam_control();
    if (beam_control != nullptr) {
      const session_fb::BeamPointingConfig* pointing = beam_control->pointing();
      if (pointing != nullptr) {
        result.beam_control.pointing.default_scan_center_deg =
            DecodeSessionAzEl(pointing->default_scan_center_deg());
        result.beam_control.pointing.nominal_beamwidth_deg =
            DecodeSessionCommandedBeamwidth(pointing->nominal_beamwidth_deg());
      }
      const session_fb::BeamSchedulerConfig* scheduler = beam_control->scheduler();
      if (scheduler != nullptr) {
        result.beam_control.scheduler.azimuth_step_count_hint =
            scheduler->azimuth_step_count_hint();
        result.beam_control.scheduler.elevation_step_count_hint =
            scheduler->elevation_step_count_hint();
        result.beam_control.scheduler.prefer_dense_tas_sampling =
            scheduler->prefer_dense_tas_sampling();
      }
    }
    const session_fb::AssociationConfig* association = value->association();
    if (association != nullptr) {
      result.association.unassigned_cost = association->unassigned_cost();
      result.association.use_distance_gate_hint = association->use_distance_gate_hint();
      result.association.distance_gate_sigma_hint = association->distance_gate_sigma_hint();
    }
    const session_fb::TrackingConfig* tracking = value->tracking();
    if (tracking != nullptr) {
      result.tracking.enable_kalman_filter = tracking->enable_kalman_filter();
      result.tracking.kalman_measurement_noise_std = tracking->kalman_measurement_noise_std();
      result.tracking.kalman_update_backend =
          static_cast<config::KalmanUpdateBackend>(tracking->kalman_update_backend());
      result.tracking.speed_decay_ratio_on_loss = tracking->speed_decay_ratio_on_loss();
      result.tracking.rcs_decay_ratio_on_loss = tracking->rcs_decay_ratio_on_loss();
    }
    const session_fb::LifecycleConfig* lifecycle = value->lifecycle();
    if (lifecycle != nullptr) {
      result.lifecycle.confirm_hits = lifecycle->confirm_hits();
      result.lifecycle.max_miss_before_lost = lifecycle->max_miss_before_lost();
      result.lifecycle.max_lost_cycles = lifecycle->max_lost_cycles();
      result.lifecycle.enable_imm_lifecycle = lifecycle->enable_imm_lifecycle();
    }
    const session_fb::ImmConfig* imm = value->imm();
    if (imm != nullptr) {
      result.imm.enable_imm_lifecycle = imm->enable_imm_lifecycle();
      result.imm.model_count_hint = imm->model_count_hint();
    }
  }
  return result;
}

environment::AtmosphericPhysicsConfig DecodeSessionAtmosphericPhysicsConfig(
    const session_fb::AtmosphericPhysicsConfig* value) {
  environment::AtmosphericPhysicsConfig result;
  if (value != nullptr) {
    result.enable_physical_model = value->enable_physical_model();
    result.pressure_hpa = value->pressure_hpa();
    result.temperature_k = value->temperature_k();
    result.relative_humidity = value->relative_humidity();
  }
  return result;
}

environment::AtmosphericDerivedContext DecodeSessionAtmosphericDerivedContext(
    const session_fb::AtmosphericDerivedContext* value) {
  environment::AtmosphericDerivedContext result;
  if (value != nullptr) {
    result.has_simulation_unix_seconds = value->has_simulation_unix_seconds();
    result.simulation_unix_seconds = value->simulation_unix_seconds();
    result.solar_flux_f107a = value->solar_flux_f107a();
    result.solar_flux_f107 = value->solar_flux_f107();
    result.geomagnetic_ap = value->geomagnetic_ap();
  }
  return result;
}

environment::VegetationScatterPhysicsConfig DecodeSessionVegetationScatterPhysicsConfig(
    const session_fb::VegetationScatterPhysicsConfig* value) {
  environment::VegetationScatterPhysicsConfig result;
  if (value != nullptr) {
    result.cover_profile = static_cast<environment::VegetationCoverProfile>(value->cover_profile());
    result.enable_physical_model = value->enable_physical_model();
  }
  return result;
}

environment::JammerEmitterState DecodeSessionJammerEmitterState(
    const session_fb::JammerEmitterState* value) {
  environment::JammerEmitterState result;
  if (value != nullptr) {
    result.technique = static_cast<environment::JammingTechnique>(value->technique());
    result.power_db = value->power_db();
    result.js_db = value->js_db();
    result.has_direction_deg = value->has_direction_deg();
    result.azimuth_deg = value->azimuth_deg();
    result.elevation_deg = value->elevation_deg();
    result.angular_span_deg = value->angular_span_deg();
    result.confidence = value->confidence();
  }
  return result;
}

environment::EnvironmentScenarioConfig DecodeSessionEnvironmentScenarioConfig(
    const session_fb::EnvironmentScenarioConfig* value) {
  environment::EnvironmentScenarioConfig result;
  if (value != nullptr) {
    result.atmospheric_physics =
        DecodeSessionAtmosphericPhysicsConfig(value->atmospheric_physics());
    result.atmospheric_context =
        DecodeSessionAtmosphericDerivedContext(value->atmospheric_context());
    result.vegetation_scatter_physics =
        DecodeSessionVegetationScatterPhysicsConfig(value->vegetation_scatter_physics());
    result.jammer_sources.clear();
    const flatbuffers::Vector<flatbuffers::Offset<session_fb::JammerEmitterState>>* jammer_sources =
        value->jammer_sources();
    if (jammer_sources != nullptr) {
      result.jammer_sources.reserve(jammer_sources->size());
      for (flatbuffers::uoffset_t i = 0; i < jammer_sources->size(); ++i) {
        result.jammer_sources.push_back(DecodeSessionJammerEmitterState(jammer_sources->Get(i)));
      }
    }
  }
  return result;
}

environment::EnvironmentRuntimeConfigPatch DecodeSessionEnvironmentRuntimeConfigPatch(
    const session_fb::EnvironmentRuntimeConfigPatch* value) {
  environment::EnvironmentRuntimeConfigPatch result;
  if (value != nullptr) {
    result.has_scenario_config = value->has_scenario_config();
    result.scenario_config = DecodeSessionEnvironmentScenarioConfig(value->scenario_config());
    result.has_jamming_sensitivity_profile = value->has_jamming_sensitivity_profile();
    result.jamming_sensitivity_profile =
        static_cast<environment::JammingSensitivityProfile>(value->jamming_sensitivity_profile());
  }
  return result;
}

flatbuffers::Offset<session_fb::EnvironmentDefaultConfig> EncodeEnvironmentDefaultConfig(
    flatbuffers::FlatBufferBuilder* builder, const environment::EnvironmentDefaultConfig& value) {
  return session_fb::CreateEnvironmentDefaultConfig(
      *builder, EncodeSessionEnvironmentScenarioConfig(builder, value.scenario_config));
}

environment::EnvironmentDefaultConfig DecodeEnvironmentDefaultConfig(
    const session_fb::EnvironmentDefaultConfig* value) {
  environment::EnvironmentDefaultConfig result;
  if (value != nullptr) {
    result.scenario_config = DecodeSessionEnvironmentScenarioConfig(value->scenario_config());
  }
  return result;
}

}  // namespace

std::string EncodeCycleInputFlatbuffer(const RadarCycleInput& input) {
  flatbuffers::FlatBufferBuilder builder;
  std::vector<flatbuffers::Offset<fb::TargetFeature>> targets;
  targets.reserve(input.scene.targets.size());
  for (std::size_t i = 0; i < input.scene.targets.size(); ++i) {
    targets.push_back(EncodeTargetFeature(&builder, input.scene.targets[i]));
  }

  const flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<fb::TargetFeature>>>
      target_vector = builder.CreateVector(targets);
  const flatbuffers::Offset<fb::RadarCycleInput> root = fb::CreateRadarCycleInput(
      builder, input.dt_sec, EncodePoseState(&builder, input.platform_pose), target_vector);
  builder.Finish(root, fb::RadarCycleInputIdentifier());

  const std::uint8_t* buffer = builder.GetBufferPointer();
  return std::string(reinterpret_cast<const char*>(buffer),
                     reinterpret_cast<const char*>(buffer) + builder.GetSize());
}

bool DecodeCycleInputFlatbuffer(const std::string& payload_bytes, RadarCycleInput* input,
                                std::string* error) {
  if (input == nullptr) {
    if (error != nullptr) {
      *error = "null RadarCycleInput output";
    }
    return false;
  }
  if (payload_bytes.empty()) {
    if (error != nullptr) {
      *error = "empty RadarCycleInput flatbuffers payload";
    }
    return false;
  }

  const std::uint8_t* data = reinterpret_cast<const std::uint8_t*>(payload_bytes.data());
  flatbuffers::Verifier verifier(data, payload_bytes.size());
  if (!fb::VerifyRadarCycleInputBuffer(verifier)) {
    if (error != nullptr) {
      *error = "invalid RadarCycleInput flatbuffers payload";
    }
    return false;
  }

  const fb::RadarCycleInput* root = fb::GetRadarCycleInput(data);
  input->dt_sec = root->dt_sec();
  input->platform_pose = DecodePoseState(root->platform_pose());
  input->scene.targets.clear();
  const flatbuffers::Vector<flatbuffers::Offset<fb::TargetFeature>>* targets =
      root->target_features();
  if (targets != nullptr) {
    input->scene.targets.reserve(targets->size());
    for (flatbuffers::uoffset_t i = 0; i < targets->size(); ++i) {
      input->scene.targets.push_back(DecodeTargetFeature(targets->Get(i)));
    }
  }
  return true;
}

std::string EncodeTrackOutputFrameFlatbuffer(const output::TrackOutputFrame& output_frame) {
  flatbuffers::FlatBufferBuilder builder;
  const flatbuffers::Offset<fb::TrackOutputFrame> root =
      EncodeTrackOutputFrame(&builder, output_frame);
  builder.Finish(root);

  const std::uint8_t* buffer = builder.GetBufferPointer();
  return std::string(reinterpret_cast<const char*>(buffer),
                     reinterpret_cast<const char*>(buffer) + builder.GetSize());
}

bool DecodeTrackOutputFrameFlatbuffer(const std::string& payload_bytes,
                                      output::TrackOutputFrame* output_frame, std::string* error) {
  if (output_frame == nullptr) {
    if (error != nullptr) {
      *error = "null TrackOutputFrame output";
    }
    return false;
  }
  if (payload_bytes.empty()) {
    if (error != nullptr) {
      *error = "empty TrackOutputFrame flatbuffers payload";
    }
    return false;
  }

  const std::uint8_t* data = reinterpret_cast<const std::uint8_t*>(payload_bytes.data());
  flatbuffers::Verifier verifier(data, payload_bytes.size());
  const fb::TrackOutputFrame* root = flatbuffers::GetRoot<fb::TrackOutputFrame>(data);
  if (root == nullptr || !root->Verify(verifier)) {
    if (error != nullptr) {
      *error = "invalid TrackOutputFrame flatbuffers payload";
    }
    return false;
  }

  *output_frame = DecodeTrackOutputFrame(root);
  return true;
}

std::string EncodeCycleResultFlatbuffer(const RadarCycleResult& result) {
  flatbuffers::FlatBufferBuilder builder;
  const flatbuffers::Offset<fb::RadarCycleResult> root = EncodeCycleResult(&builder, result);
  builder.Finish(root);

  const std::uint8_t* buffer = builder.GetBufferPointer();
  return std::string(reinterpret_cast<const char*>(buffer),
                     reinterpret_cast<const char*>(buffer) + builder.GetSize());
}

bool DecodeCycleResultFlatbuffer(const std::string& payload_bytes, RadarCycleResult* result,
                                 std::string* error) {
  if (result == nullptr) {
    if (error != nullptr) {
      *error = "null RadarCycleResult output";
    }
    return false;
  }
  if (payload_bytes.empty()) {
    if (error != nullptr) {
      *error = "empty RadarCycleResult flatbuffers payload";
    }
    return false;
  }

  const std::uint8_t* data = reinterpret_cast<const std::uint8_t*>(payload_bytes.data());
  flatbuffers::Verifier verifier(data, payload_bytes.size());
  const fb::RadarCycleResult* root = flatbuffers::GetRoot<fb::RadarCycleResult>(data);
  if (root == nullptr || !root->Verify(verifier)) {
    if (error != nullptr) {
      *error = "invalid RadarCycleResult flatbuffers payload";
    }
    return false;
  }

  *result = DecodeCycleResult(root);
  return true;
}

std::string EncodeSessionConfigFlatbuffer(const RadarSessionConfig& config) {
  flatbuffers::FlatBufferBuilder builder;
  const flatbuffers::Offset<session_fb::RadarSessionConfig> root =
      session_fb::CreateRadarSessionConfig(
          builder, EncodeSessionDetectionConfig(&builder, config.hardware.detection),
          EncodeSessionOrientation(&builder, config.mission.orientation),
          EncodeSessionPolicyConfig(&builder, config.policy),
          static_cast<int>(config.jamming_sensitivity_profile),
          EncodeEnvironmentDefaultConfig(&builder, config.environment));
  builder.Finish(root, session_fb::RadarSessionConfigIdentifier());

  const std::uint8_t* buffer = builder.GetBufferPointer();
  return std::string(reinterpret_cast<const char*>(buffer),
                     reinterpret_cast<const char*>(buffer) + builder.GetSize());
}

bool DecodeSessionConfigFlatbuffer(const std::string& payload_bytes, RadarSessionConfig* config,
                                   std::string* error) {
  if (config == nullptr) {
    if (error != nullptr) {
      *error = "null RadarSessionConfig output";
    }
    return false;
  }
  if (payload_bytes.empty()) {
    if (error != nullptr) {
      *error = "empty RadarSessionConfig flatbuffers payload";
    }
    return false;
  }

  const std::uint8_t* data = reinterpret_cast<const std::uint8_t*>(payload_bytes.data());
  flatbuffers::Verifier verifier(data, payload_bytes.size());
  if (!session_fb::VerifyRadarSessionConfigBuffer(verifier)) {
    if (error != nullptr) {
      *error = "invalid RadarSessionConfig flatbuffers payload";
    }
    return false;
  }

  const session_fb::RadarSessionConfig* root = session_fb::GetRadarSessionConfig(data);
  config->hardware.detection = DecodeSessionDetectionConfig(root->hardware_detection());
  config->mission.orientation = DecodeSessionOrientation(root->mission_orientation());
  config->policy = DecodeSessionPolicyConfig(root->policy());
  config->jamming_sensitivity_profile =
      static_cast<environment::JammingSensitivityProfile>(root->jamming_sensitivity_profile());
  config->environment = DecodeEnvironmentDefaultConfig(root->environment_default_config());
  return true;
}

std::string EncodeRuntimeConfigPatchFlatbuffer(const config::RadarRuntimeConfigPatch& patch) {
  flatbuffers::FlatBufferBuilder builder;
  const flatbuffers::Offset<session_fb::RadarRuntimeConfigPatch> root =
      session_fb::CreateRadarRuntimeConfigPatch(
          builder, patch.has_mission, EncodeSessionOrientation(&builder, patch.mission.orientation),
          patch.has_policy, EncodeSessionPolicyConfig(&builder, patch.policy),
          patch.has_environment_runtime_config,
          EncodeSessionEnvironmentRuntimeConfigPatch(&builder, patch.environment_runtime_config),
          patch.has_work_sub_mode, static_cast<int>(patch.work_sub_mode), patch.has_scan_center_deg,
          EncodeSessionAzEl(&builder, patch.scan_center_deg), patch.has_dwell_center_deg,
          EncodeSessionAzEl(&builder, patch.dwell_center_deg), patch.has_commanded_beamwidth_deg,
          EncodeSessionCommandedBeamwidth(&builder, patch.commanded_beamwidth_deg),
          patch.has_commanded_beamwidth_enabled, patch.commanded_beamwidth_enabled);
  builder.Finish(root);

  const std::uint8_t* buffer = builder.GetBufferPointer();
  return std::string(reinterpret_cast<const char*>(buffer),
                     reinterpret_cast<const char*>(buffer) + builder.GetSize());
}

bool DecodeRuntimeConfigPatchFlatbuffer(const std::string& payload_bytes,
                                        config::RadarRuntimeConfigPatch* patch,
                                        std::string* error) {
  if (patch == nullptr) {
    if (error != nullptr) {
      *error = "null RadarRuntimeConfigPatch output";
    }
    return false;
  }
  if (payload_bytes.empty()) {
    if (error != nullptr) {
      *error = "empty RadarRuntimeConfigPatch flatbuffers payload";
    }
    return false;
  }

  const std::uint8_t* data = reinterpret_cast<const std::uint8_t*>(payload_bytes.data());
  flatbuffers::Verifier verifier(data, payload_bytes.size());
  const session_fb::RadarRuntimeConfigPatch* root =
      flatbuffers::GetRoot<session_fb::RadarRuntimeConfigPatch>(data);
  if (root == nullptr || !root->Verify(verifier)) {
    if (error != nullptr) {
      *error = "invalid RadarRuntimeConfigPatch flatbuffers payload";
    }
    return false;
  }

  patch->has_mission = root->has_mission();
  patch->mission.orientation = DecodeSessionOrientation(root->mission_orientation());
  patch->has_policy = root->has_policy();
  patch->policy = DecodeSessionPolicyConfig(root->policy());
  patch->has_environment_runtime_config = root->has_environment_runtime_config();
  patch->environment_runtime_config =
      DecodeSessionEnvironmentRuntimeConfigPatch(root->environment_runtime_config());
  patch->has_work_sub_mode = root->has_work_sub_mode();
  patch->work_sub_mode = static_cast<config::RadarWorkSubMode>(root->work_sub_mode());
  patch->has_scan_center_deg = root->has_scan_center_deg();
  patch->scan_center_deg = DecodeSessionAzEl(root->scan_center_deg());
  patch->has_dwell_center_deg = root->has_dwell_center_deg();
  patch->dwell_center_deg = DecodeSessionAzEl(root->dwell_center_deg());
  patch->has_commanded_beamwidth_deg = root->has_commanded_beamwidth_deg();
  patch->commanded_beamwidth_deg = DecodeSessionCommandedBeamwidth(root->commanded_beamwidth_deg());
  patch->has_commanded_beamwidth_enabled = root->has_commanded_beamwidth_enabled();
  patch->commanded_beamwidth_enabled = root->commanded_beamwidth_enabled();
  return true;
}

std::string EncodeSceneStateFlatbuffer(const environment::EnvironmentSceneState& scene_state) {
  flatbuffers::FlatBufferBuilder builder;
  std::vector<flatbuffers::Offset<scene_fb::JammerEmitterState>> emitters;
  emitters.reserve(scene_state.jammer_emitters.size());
  for (std::size_t i = 0; i < scene_state.jammer_emitters.size(); ++i) {
    emitters.push_back(EncodeJammerEmitterState(&builder, scene_state.jammer_emitters[i]));
  }

  const flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<scene_fb::JammerEmitterState>>>
      emitter_vector = builder.CreateVector(emitters);
  const flatbuffers::Offset<scene_fb::EnvironmentSceneState> root =
      scene_fb::CreateEnvironmentSceneState(
          builder, EncodeAtmosphericPhysicsConfig(&builder, scene_state.atmospheric_physics),
          EncodeAtmosphericDerivedContext(&builder, scene_state.atmospheric_context),
          EncodeVegetationScatterPhysicsConfig(&builder, scene_state.vegetation_scatter_physics),
          emitter_vector);
  builder.Finish(root, scene_fb::EnvironmentSceneStateIdentifier());

  const std::uint8_t* buffer = builder.GetBufferPointer();
  return std::string(reinterpret_cast<const char*>(buffer),
                     reinterpret_cast<const char*>(buffer) + builder.GetSize());
}

bool DecodeSceneStateFlatbuffer(const std::string& payload_bytes,
                                environment::EnvironmentSceneState* scene_state,
                                std::string* error) {
  if (scene_state == nullptr) {
    if (error != nullptr) {
      *error = "null EnvironmentSceneState output";
    }
    return false;
  }
  if (payload_bytes.empty()) {
    if (error != nullptr) {
      *error = "empty EnvironmentSceneState flatbuffers payload";
    }
    return false;
  }

  const std::uint8_t* data = reinterpret_cast<const std::uint8_t*>(payload_bytes.data());
  flatbuffers::Verifier verifier(data, payload_bytes.size());
  if (!scene_fb::VerifyEnvironmentSceneStateBuffer(verifier)) {
    if (error != nullptr) {
      *error = "invalid EnvironmentSceneState flatbuffers payload";
    }
    return false;
  }

  const scene_fb::EnvironmentSceneState* root = scene_fb::GetEnvironmentSceneState(data);
  scene_state->atmospheric_physics = DecodeAtmosphericPhysicsConfig(root->atmospheric_physics());
  scene_state->atmospheric_context = DecodeAtmosphericDerivedContext(root->atmospheric_context());
  scene_state->vegetation_scatter_physics =
      DecodeVegetationScatterPhysicsConfig(root->vegetation_scatter_physics());
  scene_state->jammer_emitters.clear();
  const flatbuffers::Vector<flatbuffers::Offset<scene_fb::JammerEmitterState>>* emitters =
      root->jammer_emitters();
  if (emitters != nullptr) {
    scene_state->jammer_emitters.reserve(emitters->size());
    for (flatbuffers::uoffset_t i = 0; i < emitters->size(); ++i) {
      scene_state->jammer_emitters.push_back(DecodeJammerEmitterState(emitters->Get(i)));
    }
  }
  return true;
}

std::string EncodeFailureMarkerFlatbuffer(const oneq::replay::ReplayTraceFailure& failure,
                                          bool has_last_event_sequence,
                                          std::uint64_t last_event_sequence) {
  flatbuffers::FlatBufferBuilder builder;
  const flatbuffers::Offset<fb::FailureMarker> root = fb::CreateFailureMarkerDirect(
      builder, failure.error_code.c_str(), failure.message.c_str(), failure.location.c_str(),
      failure.has_cycle_index, failure.cycle_index, failure.has_sim_time_sec, failure.sim_time_sec,
      failure.diagnostics_payload.c_str(), has_last_event_sequence, last_event_sequence);
  builder.Finish(root);

  const std::uint8_t* buffer = builder.GetBufferPointer();
  return std::string(reinterpret_cast<const char*>(buffer),
                     reinterpret_cast<const char*>(buffer) + builder.GetSize());
}

bool DecodeFailureMarkerFlatbuffer(const std::string& payload_bytes,
                                   oneq::replay::ReplayTraceFailure* failure, std::string* error) {
  if (failure == nullptr) {
    if (error != nullptr) {
      *error = "null FailureMarker output";
    }
    return false;
  }
  if (payload_bytes.empty()) {
    if (error != nullptr) {
      *error = "empty FailureMarker flatbuffers payload";
    }
    return false;
  }

  const std::uint8_t* data = reinterpret_cast<const std::uint8_t*>(payload_bytes.data());
  flatbuffers::Verifier verifier(data, payload_bytes.size());
  const fb::FailureMarker* root = flatbuffers::GetRoot<fb::FailureMarker>(data);
  if (root == nullptr || !root->Verify(verifier)) {
    if (error != nullptr) {
      *error = "invalid FailureMarker flatbuffers payload";
    }
    return false;
  }

  failure->error_code = root->error_code() ? root->error_code()->str() : "";
  failure->message = root->message() ? root->message()->str() : "";
  failure->location = root->location() ? root->location()->str() : "";
  failure->has_cycle_index = root->has_cycle_index();
  failure->cycle_index = root->cycle_index();
  failure->has_sim_time_sec = root->has_sim_time_sec();
  failure->sim_time_sec = root->sim_time_sec();
  failure->diagnostics_payload = root->diagnostics() ? root->diagnostics()->str() : "{}";
  return true;
}

}  // namespace session
}  // namespace airborne_radar
