#include "airborne_radar/session/ArReplayFlatbufferCodec.h"

#include <cstdint>
#include <string>
#include <vector>

#include "1q/airborne_radar/session/TrackStateSnapshot.h"
#include "airborne_radar/session/generated/airborne_radar_replay_generated.h"
#include "airborne_radar/session/generated/airborne_radar_session_replay_generated.h"

namespace airborne_radar {
namespace session {
namespace {

namespace fb = oneq::replay::airborne_radar::fb;
namespace session_fb = oneq::replay::airborne_radar::session::fb;

std::size_t CountTracksByStatus(const session::TrackStateSnapshotList& tracks,
                                session::TrackStatus status) {
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

flatbuffers::Offset<fb::ArSceneTarget> EncodeSceneTarget(flatbuffers::FlatBufferBuilder* builder,
                                                            const ArSceneTarget& value) {
  return fb::CreateArSceneTarget(*builder, value.external_target_id, value.velocity_x,
                                    value.velocity_y, value.velocity_z, value.rcs, value.range_m,
                                    value.position_x, value.position_y, value.position_z,
                                    value.target_swerling_type, builder->CreateString(value.target_name));
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

ArSceneTarget DecodeSceneTarget(const fb::ArSceneTarget* value) {
  ArSceneTarget result;
  if (value != nullptr) {
    result.external_target_id = value->external_target_id();
    result.velocity_x = value->velocity_x();
    result.velocity_y = value->velocity_y();
    result.velocity_z = value->velocity_z();
    result.rcs = value->rcs();
    result.range_m = value->range_m();
    result.position_x = value->position_x();
    result.position_y = value->position_y();
    result.position_z = value->position_z();
    result.target_swerling_type = value->target_swerling_type();
    if (value->target_name() != nullptr) {
      result.target_name = value->target_name()->str();
    }
  }
  return result;
}

flatbuffers::Offset<fb::AtmosphericObservation> EncodeCycleAtmosphericObservation(
    flatbuffers::FlatBufferBuilder* builder, const config::AtmosphericPhysicsConfig& value) {
  return fb::CreateAtmosphericObservation(*builder, value.enable_physical_model, value.pressure_hpa,
                                          value.temperature_k, value.relative_humidity);
}

flatbuffers::Offset<fb::AtmosphericContext> EncodeCycleAtmosphericContext(
    flatbuffers::FlatBufferBuilder* builder, const config::AtmosphericDerivedContext& value) {
  return fb::CreateAtmosphericContext(*builder, value.has_simulation_unix_seconds,
                                      value.simulation_unix_seconds, value.solar_flux_f107a,
                                      value.solar_flux_f107, value.geomagnetic_ap);
}

flatbuffers::Offset<fb::SurfaceObservation> EncodeCycleSurfaceObservation(
    flatbuffers::FlatBufferBuilder* builder,
    const config::VegetationScatterPhysicsConfig& value) {
  return fb::CreateSurfaceObservation(*builder, static_cast<int>(value.cover_profile),
                                      value.enable_physical_model);
}

flatbuffers::Offset<fb::JammerSource> EncodeCycleJammerSource(
    flatbuffers::FlatBufferBuilder* builder, const config::JammerEmitterState& value) {
  return fb::CreateJammerSource(*builder, static_cast<int>(value.technique), value.power_db,
                                value.js_db, value.position_x, value.position_y, value.position_z,
                                value.angular_span_deg, value.confidence);
}

flatbuffers::Offset<fb::ArCycleEnvironmentInput> EncodeCycleEnvironmentInput(
    flatbuffers::FlatBufferBuilder* builder, const ArEnvironmentInput& value) {
  std::vector<flatbuffers::Offset<fb::JammerSource>> jammer_sources;
  jammer_sources.reserve(value.jammer_sources.size());
  for (std::size_t i = 0; i < value.jammer_sources.size(); ++i) {
    jammer_sources.push_back(EncodeCycleJammerSource(builder, value.jammer_sources[i]));
  }
  return fb::CreateArCycleEnvironmentInput(
      *builder, EncodeCycleAtmosphericObservation(builder, value.atmospheric_observation),
      EncodeCycleAtmosphericContext(builder, value.atmospheric_context),
      EncodeCycleSurfaceObservation(builder, value.surface_observation),
      builder->CreateVector(jammer_sources));
}

config::AtmosphericPhysicsConfig DecodeCycleAtmosphericObservation(
    const fb::AtmosphericObservation* value) {
  config::AtmosphericPhysicsConfig result;
  if (value != nullptr) {
    result.enable_physical_model = value->enable_physical_model();
    result.pressure_hpa = value->pressure_hpa();
    result.temperature_k = value->temperature_k();
    result.relative_humidity = value->relative_humidity();
  }
  return result;
}

config::AtmosphericDerivedContext DecodeCycleAtmosphericContext(
    const fb::AtmosphericContext* value) {
  config::AtmosphericDerivedContext result;
  if (value != nullptr) {
    result.has_simulation_unix_seconds = value->has_simulation_unix_seconds();
    result.simulation_unix_seconds = value->simulation_unix_seconds();
    result.solar_flux_f107a = value->solar_flux_f107a();
    result.solar_flux_f107 = value->solar_flux_f107();
    result.geomagnetic_ap = value->geomagnetic_ap();
  }
  return result;
}

config::VegetationScatterPhysicsConfig DecodeCycleSurfaceObservation(
    const fb::SurfaceObservation* value) {
  config::VegetationScatterPhysicsConfig result;
  if (value != nullptr) {
    result.cover_profile = static_cast<config::VegetationCoverProfile>(value->cover_profile());
    result.enable_physical_model = value->enable_physical_model();
  }
  return result;
}

config::JammerEmitterState DecodeCycleJammerSource(const fb::JammerSource* value) {
  config::JammerEmitterState result;
  if (value != nullptr) {
    result.technique = static_cast<config::JammingTechnique>(value->technique());
    result.power_db = value->power_db();
    result.js_db = value->js_db();
    result.position_x = value->position_x();
    result.position_y = value->position_y();
    result.position_z = value->position_z();
    result.angular_span_deg = value->angular_span_deg();
    result.confidence = value->confidence();
  }
  return result;
}

ArEnvironmentInput DecodeCycleEnvironmentInput(const fb::ArCycleEnvironmentInput* value) {
  ArEnvironmentInput result;
  if (value != nullptr) {
    result.atmospheric_observation =
        DecodeCycleAtmosphericObservation(value->atmospheric_observation());
    result.atmospheric_context = DecodeCycleAtmosphericContext(value->atmospheric_context());
    result.surface_observation = DecodeCycleSurfaceObservation(value->surface_observation());
    const flatbuffers::Vector<flatbuffers::Offset<fb::JammerSource>>* jammer_sources =
        value->jammer_sources();
    if (jammer_sources != nullptr) {
      result.jammer_sources.reserve(jammer_sources->size());
      for (flatbuffers::uoffset_t i = 0; i < jammer_sources->size(); ++i) {
        result.jammer_sources.push_back(DecodeCycleJammerSource(jammer_sources->Get(i)));
      }
    }
  }
  return result;
}

flatbuffers::Offset<fb::DecisionTrackStateSnapshot> EncodeTrackStateSnapshot(
    flatbuffers::FlatBufferBuilder* builder, const session::TrackStateSnapshot& value) {
  return fb::CreateDecisionTrackStateSnapshot(
      *builder, value.association_key, value.external_target_id, static_cast<int>(value.status),
      value.position_x, value.position_y, value.position_z, value.velocity_x, value.velocity_y,
      value.velocity_z, value.speed, value.acceleration_x, value.acceleration_y,
      value.acceleration_z, value.acceleration, value.rcs, value.jamming_detected, value.hit_count,
      value.miss_count, builder->CreateString(value.target_type), value.target_probability,
      builder->CreateString(value.target_name));
}

flatbuffers::Offset<fb::TrackStateSnapshot> EncodeTrackSnapshot(
    flatbuffers::FlatBufferBuilder* builder, const session::TrackStateSnapshot& value) {
  return fb::CreateTrackStateSnapshot(*builder, EncodeTrackStateSnapshot(builder, value));
}

session::TrackStateSnapshot DecodeTrackStateSnapshot(const fb::DecisionTrackStateSnapshot* value) {
  session::TrackStateSnapshot result;
  if (value != nullptr) {
    result.association_key = value->association_key();
    result.external_target_id = value->external_target_id();
    result.status = static_cast<session::TrackStatus>(value->status());
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
    if (value->target_type() != nullptr) {
      result.target_type = value->target_type()->str();
    }
    result.target_probability = value->target_probability();
    if (value->target_name() != nullptr) {
      result.target_name = value->target_name()->str();
    }
  }
  return result;
}

session::TrackStateSnapshot DecodeTrackSnapshot(const fb::TrackStateSnapshot* value) {
  session::TrackStateSnapshot result;
  if (value != nullptr) {
    result = DecodeTrackStateSnapshot(value->state());
  }
  return result;
}

flatbuffers::Offset<fb::TrackOutputFrame> EncodeTrackOutputFrame(
    flatbuffers::FlatBufferBuilder* builder, const session::TrackOutputFrame& value) {
  std::vector<flatbuffers::Offset<fb::TrackStateSnapshot>> track_offsets;
  track_offsets.reserve(value.tracks.size());
  for (std::size_t i = 0; i < value.tracks.size(); ++i) {
    track_offsets.push_back(EncodeTrackSnapshot(builder, value.tracks[i]));
  }
  const flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<fb::TrackStateSnapshot>>>
      tracks_vec = builder->CreateVector(track_offsets);
  return fb::CreateTrackOutputFrame(
      *builder, value.cycle_index, static_cast<std::uint64_t>(value.tracks.size()), value.batch_id,
      static_cast<std::uint64_t>(CountTracksByStatus(value.tracks, session::TrackStatus::kConfirmed)),
      CountTracksByStatus(value.tracks, session::TrackStatus::kLost) > 0U, tracks_vec);
}

session::TrackOutputFrame DecodeTrackOutputFrame(const fb::TrackOutputFrame* value) {
  session::TrackOutputFrame result;
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

flatbuffers::Offset<fb::ValidationIssue> EncodeValidationIssue(
    flatbuffers::FlatBufferBuilder* builder, const ValidationIssue& value) {
  const int encoded_entity_index = value.location.kind == ValidationLocationKind::kSceneEntity
                                       ? static_cast<int>(value.location.entity_index)
                                       : -1;
  return fb::CreateValidationIssue(
      *builder, static_cast<int>(value.severity), static_cast<int>(value.code),
      static_cast<int>(value.location.kind), encoded_entity_index,
      builder->CreateString(value.field), builder->CreateString(value.message));
}

ValidationIssue DecodeValidationIssue(const fb::ValidationIssue* value) {
  ValidationIssue result;
  if (value != nullptr) {
    result.severity = static_cast<ValidationSeverity>(value->severity());
    result.code = static_cast<ValidationCode>(value->code());
    result.location.kind = static_cast<ValidationLocationKind>(value->location_kind());
    result.location.entity_index = value->entity_index() >= 0
                                       ? static_cast<std::size_t>(value->entity_index())
                                       : static_cast<std::size_t>(-1);
    if (value->field() != nullptr) {
      result.field = value->field()->str();
    }
    if (value->message() != nullptr) {
      result.message = value->message()->str();
    }
  }
  return result;
}

flatbuffers::Offset<fb::ArCommand> EncodeArCommand(
    flatbuffers::FlatBufferBuilder* builder, const session::ArCommand& value) {
  return fb::CreateArCommand(*builder, static_cast<int>(value.type),
                                static_cast<int>(value.source));
}

session::ArCommand DecodeArCommand(const fb::ArCommand* value) {
  session::ArCommand result;
  if (value != nullptr) {
    result.type = static_cast<session::ArCommandType>(value->type());
    result.source = static_cast<session::ArCommandSource>(value->source());
  }
  return result;
}

flatbuffers::Offset<fb::ArControlProfile> EncodeArControlProfile(
    flatbuffers::FlatBufferBuilder* builder, const session::ArControlProfile& value) {
  return fb::CreateArControlProfile(
      *builder, value.version, value.enable_lpi_power_control, value.lpi_power_scale,
      value.enable_lpi_beamforming, value.lpi_dwell_scale, value.enable_agility_frequency,
      value.agility_frequency_hop_phase, value.enable_sidelobe_canceller,
      value.enable_adaptive_beamforming, value.enable_eccm_rejitter, value.eccm_burnthrough_gain);
}

session::ArControlProfile DecodeArControlProfile(
    const fb::ArControlProfile* value) {
  session::ArControlProfile result;
  if (value != nullptr) {
    result.version = value->version();
    result.enable_lpi_power_control = value->enable_lpi_power_control();
    result.lpi_power_scale = value->lpi_power_scale();
    result.enable_lpi_beamforming = value->enable_lpi_beamforming();
    result.lpi_dwell_scale = value->lpi_dwell_scale();
    result.enable_agility_frequency = value->enable_agility_frequency();
    result.agility_frequency_hop_phase = value->agility_frequency_hop_phase();
    result.enable_sidelobe_canceller = value->enable_sidelobe_canceller();
    result.enable_adaptive_beamforming = value->enable_adaptive_beamforming();
    result.enable_eccm_rejitter = value->enable_eccm_rejitter();
    result.eccm_burnthrough_gain = value->eccm_burnthrough_gain();
  }
  return result;
}

flatbuffers::Offset<fb::AssociationQualityMetrics> EncodeAssociationQualityMetrics(
    flatbuffers::FlatBufferBuilder* builder, const session::AssociationQualityMetrics& value) {
  return fb::CreateAssociationQualityMetrics(
      *builder, static_cast<std::uint64_t>(value.prior_track_count),
      static_cast<std::uint64_t>(value.detection_count),
      static_cast<std::uint64_t>(value.matched_count),
      static_cast<std::uint64_t>(value.new_track_count),
      static_cast<std::uint64_t>(value.missed_track_count), value.match_rate, value.new_track_rate,
      value.missed_track_rate, value.mean_match_cost, value.p95_match_cost,
      static_cast<int>(value.dominant_jamming_semantic), value.jamming_severity,
      value.association_stress);
}

session::AssociationQualityMetrics DecodeAssociationQualityMetrics(
    const fb::AssociationQualityMetrics* value) {
  session::AssociationQualityMetrics result;
  if (value != nullptr) {
    result.prior_track_count = static_cast<std::size_t>(value->prior_track_count());
    result.detection_count = static_cast<std::size_t>(value->detection_count());
    result.matched_count = static_cast<std::size_t>(value->matched_count());
    result.new_track_count = static_cast<std::size_t>(value->new_track_count());
    result.missed_track_count = static_cast<std::size_t>(value->missed_track_count());
    result.match_rate = value->match_rate();
    result.new_track_rate = value->new_track_rate();
    result.missed_track_rate = value->missed_track_rate();
    result.mean_match_cost = value->mean_match_cost();
    result.p95_match_cost = value->p95_match_cost();
    result.dominant_jamming_semantic =
        static_cast<config::JammingSemantic>(value->dominant_jamming_semantic());
    result.jamming_severity = value->jamming_severity();
    result.association_stress = value->association_stress();
  }
  return result;
}

flatbuffers::Offset<fb::ArCycleResult> EncodeCycleResult(flatbuffers::FlatBufferBuilder* builder,
                                                            const ArCycleResult& value) {
  std::vector<flatbuffers::Offset<fb::ArCommand>> command_offsets;
  command_offsets.reserve(value.submitted_commands.size());
  for (std::size_t i = 0; i < value.submitted_commands.size(); ++i) {
    command_offsets.push_back(EncodeArCommand(builder, value.submitted_commands[i]));
  }

  std::vector<flatbuffers::Offset<fb::ValidationIssue>> issue_offsets;
  issue_offsets.reserve(value.validation_issues.size());
  for (std::size_t i = 0; i < value.validation_issues.size(); ++i) {
    issue_offsets.push_back(EncodeValidationIssue(builder, value.validation_issues[i]));
  }

  return fb::CreateArCycleResult(
      *builder, value.input_cycle_index, EncodeTrackOutputFrame(builder, value.track_output_frame),
      builder->CreateVector(command_offsets), builder->CreateVector(issue_offsets),
      value.has_validation_error, value.executed_this_cycle,
      static_cast<int>(value.abort_reason), value.reused_previous_output,
      value.has_control_profile, EncodeArControlProfile(builder, value.control_profile),
      EncodeAssociationQualityMetrics(builder, value.association_quality_metrics));
}

ArCycleResult DecodeCycleResult(const fb::ArCycleResult* value) {
  ArCycleResult result;
  if (value != nullptr) {
    result.input_cycle_index = value->input_cycle_index();
    result.track_output_frame = DecodeTrackOutputFrame(value->track_output_frame());
    const flatbuffers::Vector<flatbuffers::Offset<fb::ArCommand>>* commands =
        value->submitted_commands();
    if (commands != nullptr) {
      result.submitted_commands.reserve(commands->size());
      for (flatbuffers::uoffset_t i = 0; i < commands->size(); ++i) {
        result.submitted_commands.push_back(DecodeArCommand(commands->Get(i)));
      }
    }
    const flatbuffers::Vector<flatbuffers::Offset<fb::ValidationIssue>>* issues =
        value->validation_issues();
    if (issues != nullptr) {
      result.validation_issues.reserve(issues->size());
      for (flatbuffers::uoffset_t i = 0; i < issues->size(); ++i) {
        result.validation_issues.push_back(DecodeValidationIssue(issues->Get(i)));
      }
    }
    result.has_validation_error = value->has_validation_error();
    result.executed_this_cycle = value->executed_this_cycle();
    result.abort_reason =
        static_cast<session::SignalCycleAbortReason>(value->abort_reason());
    result.reused_previous_output = value->reused_previous_output();
    result.has_control_profile = value->has_control_profile();
    result.control_profile = DecodeArControlProfile(value->control_profile());
    result.association_quality_metrics =
        DecodeAssociationQualityMetrics(value->association_quality_metrics());
  }
  return result;
}

flatbuffers::Offset<session_fb::EulerAnglesDeg> EncodeSessionEulerAngles(
    flatbuffers::FlatBufferBuilder* builder, const config::EulerAnglesDeg& value) {
  return session_fb::CreateEulerAnglesDeg(*builder, value.yaw_deg, value.pitch_deg, value.roll_deg);
}

flatbuffers::Offset<session_fb::AzimuthElevationDeg> EncodeSessionAzEl(
    flatbuffers::FlatBufferBuilder* builder, const config::AzimuthElevationDeg& value) {
  return session_fb::CreateAzimuthElevationDeg(*builder, value.az_deg, value.el_deg);
}

flatbuffers::Offset<session_fb::AzimuthElevationLimitsDeg> EncodeSessionAzElLimits(
    flatbuffers::FlatBufferBuilder* builder, const config::AzimuthElevationLimitsDeg& value) {
  return session_fb::CreateAzimuthElevationLimitsDeg(*builder, value.az_min_deg, value.az_max_deg,
                                                     value.el_min_deg, value.el_max_deg);
}

flatbuffers::Offset<session_fb::CommandedBeamwidthDeg> EncodeSessionCommandedBeamwidth(
    flatbuffers::FlatBufferBuilder* builder, const config::CommandedBeamwidthDeg& value) {
  return session_fb::CreateCommandedBeamwidthDeg(*builder, value.commanded_az_beamwidth_deg,
                                                 value.commanded_el_beamwidth_deg);
}

flatbuffers::Offset<session_fb::ArOrientationConfig> EncodeSessionOrientation(
    flatbuffers::FlatBufferBuilder* builder, const config::ArOrientationConfig& value) {
  return session_fb::CreateArOrientationConfig(
      *builder, EncodeSessionEulerAngles(builder, value.mount_angles_deg),
      EncodeSessionAzEl(builder, value.scan_center_deg),
      EncodeSessionAzElLimits(builder, value.mechanical_scan_limits_deg),
      EncodeSessionAzElLimits(builder, value.electronic_scan_limits_deg),
      static_cast<int>(value.scan_start_position), static_cast<int>(value.scan_sequence),
      static_cast<int>(value.work_mode), value.commanded_beamwidth_enabled,
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
      value.antenna.nominal_el_beamwidth_deg, value.antenna.enable_directional_pattern, pattern,
      value.antenna.antenna_length_m, value.antenna.antenna_width_m);
  const flatbuffers::Offset<session_fb::ReceiverConfig> receiver = session_fb::CreateReceiverConfig(
      *builder, value.receiver.noise_figure_db, value.receiver.receive_loss_db);
  const flatbuffers::Offset<session_fb::RcsPhysicsConfig> rcs_physics =
      session_fb::CreateRcsPhysicsConfig(
          *builder, value.rcs_physics.enable_physical_rcs,
          value.rcs_physics.physics_mix_ratio, value.rcs_physics.cylinder_weight,
          value.rcs_physics.min_equivalent_radius_m, value.rcs_physics.max_equivalent_radius_m,
          value.rcs_physics.min_rcs_m2, value.rcs_physics.max_rcs_m2,
          value.rcs_physics.bistatic_psi_offset_deg);
  return session_fb::CreateDetectionConfig(
      *builder, value.enable_physics_detection,
      static_cast<int>(config::profiles::SwerlingModel::kSwerling0), transmitter,
      antenna, receiver, rcs_physics);
}

flatbuffers::Offset<session_fb::ArPolicyConfig> EncodeSessionPolicyConfig(
    flatbuffers::FlatBufferBuilder* builder, const config::ArPolicyConfig& value) {
  const flatbuffers::Offset<session_fb::ArDetectionPolicyConfig> detection =
      session_fb::CreateArDetectionPolicyConfig(
          *builder, value.detection.minimum_snr_db, value.detection.pfa,
          value.detection.pulse_count, value.detection.minimum_detection_margin_db);
  const flatbuffers::Offset<session_fb::BeamPointingConfig> pointing =
      session_fb::CreateBeamPointingConfig(
          *builder, EncodeSessionCommandedBeamwidth(
                        builder, value.beam_control.pointing.nominal_beamwidth_deg));
  const flatbuffers::Offset<session_fb::BeamSchedulerConfig> scheduler =
      session_fb::CreateBeamSchedulerConfig(*builder,
                                            value.beam_control.scheduler.azimuth_step_count_hint,
                                            value.beam_control.scheduler.elevation_step_count_hint,
                                            value.beam_control.scheduler.prefer_dense_tas_sampling);
  const flatbuffers::Offset<session_fb::BeamControlConfig> beam_control =
      session_fb::CreateBeamControlConfig(*builder, pointing, scheduler);
  const flatbuffers::Offset<session_fb::AssociationConfig> association =
      session_fb::CreateAssociationConfig(*builder, value.association.distance_gate_sigma);
  const flatbuffers::Offset<session_fb::TrackingConfig> tracking = session_fb::CreateTrackingConfig(
      *builder, value.tracking.enable_kalman_filter, value.tracking.kalman_measurement_noise_std,
      value.tracking.speed_decay_ratio_on_loss, value.tracking.rcs_decay_ratio_on_loss);
  const flatbuffers::Offset<session_fb::LifecycleConfig> lifecycle =
      session_fb::CreateLifecycleConfig(
          *builder, value.lifecycle.confirm_hits, value.lifecycle.max_miss_before_lost,
          value.lifecycle.max_lost_cycles, value.lifecycle.enable_imm_lifecycle);
  const flatbuffers::Offset<session_fb::ImmConfig> imm = session_fb::CreateImmConfig(
      *builder, value.lifecycle.enable_imm_lifecycle, value.lifecycle.model_count_hint);
  return session_fb::CreateArPolicyConfig(*builder, detection, beam_control, association,
                                          tracking, lifecycle, imm);
}

flatbuffers::Offset<session_fb::AtmosphericPhysicsConfig> EncodeSessionAtmosphericPhysicsConfig(
    flatbuffers::FlatBufferBuilder* builder, const config::AtmosphericPhysicsConfig& value) {
  return session_fb::CreateAtmosphericPhysicsConfig(*builder, value.enable_physical_model,
                                                    value.pressure_hpa, value.temperature_k,
                                                    value.relative_humidity);
}

flatbuffers::Offset<session_fb::AtmosphericDerivedContext> EncodeSessionAtmosphericDerivedContext(
    flatbuffers::FlatBufferBuilder* builder, const config::AtmosphericDerivedContext& value) {
  return session_fb::CreateAtmosphericDerivedContext(
      *builder, value.has_simulation_unix_seconds, value.simulation_unix_seconds,
      value.solar_flux_f107a, value.solar_flux_f107, value.geomagnetic_ap);
}

flatbuffers::Offset<session_fb::VegetationScatterPhysicsConfig>
EncodeSessionVegetationScatterPhysicsConfig(
    flatbuffers::FlatBufferBuilder* builder,
    const config::VegetationScatterPhysicsConfig& value) {
  return session_fb::CreateVegetationScatterPhysicsConfig(
      *builder, static_cast<int>(value.cover_profile), value.enable_physical_model);
}

flatbuffers::Offset<session_fb::JammerEmitterState> EncodeSessionJammerEmitterState(
    flatbuffers::FlatBufferBuilder* builder, const config::JammerEmitterState& value) {
  return session_fb::CreateJammerEmitterState(*builder, static_cast<int>(value.technique),
                                              value.power_db, value.js_db, value.position_x,
                                              value.position_y, value.position_z,
                                              value.angular_span_deg, value.confidence);
}

flatbuffers::Offset<session_fb::EnvironmentScenarioConfig> EncodeSessionEnvironmentScenarioConfig(
    flatbuffers::FlatBufferBuilder* builder, const config::EnvironmentScenarioConfig& value) {
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
    const config::EnvironmentRuntimeConfigPatch& value) {
  return session_fb::CreateEnvironmentRuntimeConfigPatch(
      *builder, value.has_scenario_config,
      EncodeSessionEnvironmentScenarioConfig(builder, value.scenario_config),
      value.has_jamming_sensitivity_profile, static_cast<int>(value.jamming_sensitivity_profile));
}

config::EulerAnglesDeg DecodeSessionEulerAngles(const session_fb::EulerAnglesDeg* value) {
  config::EulerAnglesDeg result;
  if (value != nullptr) {
    result.yaw_deg = value->yaw_deg();
    result.pitch_deg = value->pitch_deg();
    result.roll_deg = value->roll_deg();
  }
  return result;
}

config::AzimuthElevationDeg DecodeSessionAzEl(const session_fb::AzimuthElevationDeg* value) {
  config::AzimuthElevationDeg result;
  if (value != nullptr) {
    result.az_deg = value->az_deg();
    result.el_deg = value->el_deg();
  }
  return result;
}

config::AzimuthElevationLimitsDeg DecodeSessionAzElLimits(
    const session_fb::AzimuthElevationLimitsDeg* value) {
  config::AzimuthElevationLimitsDeg result;
  if (value != nullptr) {
    result.az_min_deg = value->az_min_deg();
    result.az_max_deg = value->az_max_deg();
    result.el_min_deg = value->el_min_deg();
    result.el_max_deg = value->el_max_deg();
  }
  return result;
}

config::CommandedBeamwidthDeg DecodeSessionCommandedBeamwidth(
    const session_fb::CommandedBeamwidthDeg* value) {
  config::CommandedBeamwidthDeg result;
  if (value != nullptr) {
    result.commanded_az_beamwidth_deg = value->commanded_az_beamwidth_deg();
    result.commanded_el_beamwidth_deg = value->commanded_el_beamwidth_deg();
  }
  return result;
}

config::ArOrientationConfig DecodeSessionOrientation(
    const session_fb::ArOrientationConfig* value) {
  config::ArOrientationConfig result;
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
    result.work_mode = static_cast<config::ArWorkMode>(value->work_mode());
    result.commanded_beamwidth_enabled = value->commanded_beamwidth_enabled();
    result.commanded_beamwidth_deg =
        DecodeSessionCommandedBeamwidth(value->commanded_beamwidth_deg());
    result.stabilization_mode = static_cast<config::StabilizationMode>(value->stabilization_mode());
  }
  return result;
}

config::DetectionConfig DecodeSessionDetectionConfig(const session_fb::DetectionConfig* value) {
  config::DetectionConfig result;
  if (value != nullptr) {
    result.enable_physics_detection = value->enable_physics_detection();
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
      result.antenna.antenna_length_m = antenna->antenna_length_m();
      result.antenna.antenna_width_m = antenna->antenna_width_m();
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
    const session_fb::RcsPhysicsConfig* rcs_physics = value->rcs_physics();
    if (rcs_physics != nullptr) {
      result.rcs_physics.enable_physical_rcs = rcs_physics->enable_physical_rcs();
      result.rcs_physics.physics_mix_ratio = rcs_physics->physics_mix_ratio();
      result.rcs_physics.cylinder_weight = rcs_physics->cylinder_weight();
      result.rcs_physics.min_equivalent_radius_m = rcs_physics->min_equivalent_radius_m();
      result.rcs_physics.max_equivalent_radius_m = rcs_physics->max_equivalent_radius_m();
      result.rcs_physics.min_rcs_m2 = rcs_physics->min_rcs_m2();
      result.rcs_physics.max_rcs_m2 = rcs_physics->max_rcs_m2();
      result.rcs_physics.bistatic_psi_offset_deg = rcs_physics->bistatic_psi_offset_deg();
    }
  }
  return result;
}

config::ArPolicyConfig DecodeSessionPolicyConfig(const session_fb::ArPolicyConfig* value) {
  config::ArPolicyConfig result;
  if (value != nullptr) {
    const session_fb::ArDetectionPolicyConfig* detection = value->detection();
    if (detection != nullptr) {
      result.detection.minimum_snr_db = detection->minimum_snr_db();
      result.detection.pfa = detection->pfa();
      result.detection.pulse_count = detection->pulse_count();
      result.detection.minimum_detection_margin_db =
          detection->minimum_detection_margin_db();
    }
    const session_fb::BeamControlConfig* beam_control = value->beam_control();
    if (beam_control != nullptr) {
      const session_fb::BeamPointingConfig* pointing = beam_control->pointing();
      if (pointing != nullptr) {
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
      result.association.distance_gate_sigma = association->distance_gate_sigma();
    }
    const session_fb::TrackingConfig* tracking = value->tracking();
    if (tracking != nullptr) {
      result.tracking.enable_kalman_filter = tracking->enable_kalman_filter();
      result.tracking.kalman_measurement_noise_std = tracking->kalman_measurement_noise_std();
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
      result.lifecycle.enable_imm_lifecycle = imm->enable_imm_lifecycle();
      result.lifecycle.model_count_hint = imm->model_count_hint();
    }
  }
  return result;
}

config::AtmosphericPhysicsConfig DecodeSessionAtmosphericPhysicsConfig(
    const session_fb::AtmosphericPhysicsConfig* value) {
  config::AtmosphericPhysicsConfig result;
  if (value != nullptr) {
    result.enable_physical_model = value->enable_physical_model();
    result.pressure_hpa = value->pressure_hpa();
    result.temperature_k = value->temperature_k();
    result.relative_humidity = value->relative_humidity();
  }
  return result;
}

config::AtmosphericDerivedContext DecodeSessionAtmosphericDerivedContext(
    const session_fb::AtmosphericDerivedContext* value) {
  config::AtmosphericDerivedContext result;
  if (value != nullptr) {
    result.has_simulation_unix_seconds = value->has_simulation_unix_seconds();
    result.simulation_unix_seconds = value->simulation_unix_seconds();
    result.solar_flux_f107a = value->solar_flux_f107a();
    result.solar_flux_f107 = value->solar_flux_f107();
    result.geomagnetic_ap = value->geomagnetic_ap();
  }
  return result;
}

config::VegetationScatterPhysicsConfig DecodeSessionVegetationScatterPhysicsConfig(
    const session_fb::VegetationScatterPhysicsConfig* value) {
  config::VegetationScatterPhysicsConfig result;
  if (value != nullptr) {
    result.cover_profile = static_cast<config::VegetationCoverProfile>(value->cover_profile());
    result.enable_physical_model = value->enable_physical_model();
  }
  return result;
}

config::JammerEmitterState DecodeSessionJammerEmitterState(
    const session_fb::JammerEmitterState* value) {
  config::JammerEmitterState result;
  if (value != nullptr) {
    result.technique = static_cast<config::JammingTechnique>(value->technique());
    result.power_db = value->power_db();
    result.js_db = value->js_db();
    result.position_x = value->position_x();
    result.position_y = value->position_y();
    result.position_z = value->position_z();
    result.angular_span_deg = value->angular_span_deg();
    result.confidence = value->confidence();
  }
  return result;
}

config::EnvironmentScenarioConfig DecodeSessionEnvironmentScenarioConfig(
    const session_fb::EnvironmentScenarioConfig* value) {
  config::EnvironmentScenarioConfig result;
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

config::EnvironmentRuntimeConfigPatch DecodeSessionEnvironmentRuntimeConfigPatch(
    const session_fb::EnvironmentRuntimeConfigPatch* value) {
  config::EnvironmentRuntimeConfigPatch result;
  if (value != nullptr) {
    result.has_scenario_config = value->has_scenario_config();
    result.scenario_config = DecodeSessionEnvironmentScenarioConfig(value->scenario_config());
    result.has_jamming_sensitivity_profile = value->has_jamming_sensitivity_profile();
    result.jamming_sensitivity_profile =
        static_cast<config::JammingSensitivityProfile>(value->jamming_sensitivity_profile());
  }
  return result;
}

flatbuffers::Offset<session_fb::EnvironmentDefaultConfig> EncodeEnvironmentDefaultConfig(
    flatbuffers::FlatBufferBuilder* builder, const config::ArEnvironmentConfig& value) {
  return session_fb::CreateEnvironmentDefaultConfig(
      *builder, EncodeSessionEnvironmentScenarioConfig(builder, value.scenario_config));
}

config::ArEnvironmentConfig DecodeEnvironmentDefaultConfig(
    const session_fb::EnvironmentDefaultConfig* value) {
  config::ArEnvironmentConfig result;
  if (value != nullptr) {
    result.scenario_config = DecodeSessionEnvironmentScenarioConfig(value->scenario_config());
  }
  return result;
}

}  // namespace

std::string EncodeCycleInputFlatbuffer(const ArCycleInput& input) {
  flatbuffers::FlatBufferBuilder builder;
  std::vector<flatbuffers::Offset<fb::ArSceneTarget>> targets;
  targets.reserve(input.scene.size());
  for (std::size_t i = 0; i < input.scene.size(); ++i) {
    targets.push_back(EncodeSceneTarget(&builder, input.scene[i]));
  }

  const flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<fb::ArSceneTarget>>>
      scene_vector = builder.CreateVector(targets);
  const flatbuffers::Offset<fb::ArCycleEnvironmentInput> env =
      EncodeCycleEnvironmentInput(&builder, input.environment);
  const flatbuffers::Offset<fb::ArCycleInput> root = fb::CreateArCycleInput(
      builder, input.cycle_index, input.dt_sec, EncodePoseState(&builder, input.platform_pose),
      scene_vector, input.has_environment, env, input.platform_altitude_m);
  builder.Finish(root, fb::ArCycleInputIdentifier());

  const std::uint8_t* buffer = builder.GetBufferPointer();
  return std::string(reinterpret_cast<const char*>(buffer),
                     reinterpret_cast<const char*>(buffer) + builder.GetSize());
}

bool DecodeCycleInputFlatbuffer(const std::string& payload_bytes, ArCycleInput* input,
                                std::string* error) {
  if (input == nullptr) {
    if (error != nullptr) {
      *error = "null ArCycleInput output";
    }
    return false;
  }
  if (payload_bytes.empty()) {
    if (error != nullptr) {
      *error = "empty ArCycleInput flatbuffers payload";
    }
    return false;
  }

  const std::uint8_t* data = reinterpret_cast<const std::uint8_t*>(payload_bytes.data());
  flatbuffers::Verifier verifier(data, payload_bytes.size());
  if (!fb::VerifyArCycleInputBuffer(verifier)) {
    if (error != nullptr) {
      *error = "invalid ArCycleInput flatbuffers payload";
    }
    return false;
  }

  const fb::ArCycleInput* root = fb::GetArCycleInput(data);
  input->cycle_index = root->cycle_index();
  input->dt_sec = root->dt_sec();
  input->platform_altitude_m = root->platform_altitude_m();
  input->platform_pose = DecodePoseState(root->platform_pose());
  input->has_environment = root->has_environment();
  input->environment = DecodeCycleEnvironmentInput(root->environment());
  input->scene.clear();
  const flatbuffers::Vector<flatbuffers::Offset<fb::ArSceneTarget>>* scene = root->scene();
  if (scene != nullptr) {
    input->scene.reserve(scene->size());
    for (flatbuffers::uoffset_t i = 0; i < scene->size(); ++i) {
      input->scene.push_back(DecodeSceneTarget(scene->Get(i)));
    }
  }
  return true;
}

std::string EncodeTrackOutputFrameFlatbuffer(const session::TrackOutputFrame& output_frame) {
  flatbuffers::FlatBufferBuilder builder;
  const flatbuffers::Offset<fb::TrackOutputFrame> root =
      EncodeTrackOutputFrame(&builder, output_frame);
  builder.Finish(root);

  const std::uint8_t* buffer = builder.GetBufferPointer();
  return std::string(reinterpret_cast<const char*>(buffer),
                     reinterpret_cast<const char*>(buffer) + builder.GetSize());
}

bool DecodeTrackOutputFrameFlatbuffer(const std::string& payload_bytes,
                                      session::TrackOutputFrame* output_frame, std::string* error) {
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

std::string EncodeCycleResultFlatbuffer(const ArCycleResult& result) {
  flatbuffers::FlatBufferBuilder builder;
  const flatbuffers::Offset<fb::ArCycleResult> root = EncodeCycleResult(&builder, result);
  builder.Finish(root);

  const std::uint8_t* buffer = builder.GetBufferPointer();
  return std::string(reinterpret_cast<const char*>(buffer),
                     reinterpret_cast<const char*>(buffer) + builder.GetSize());
}

bool DecodeCycleResultFlatbuffer(const std::string& payload_bytes, ArCycleResult* result,
                                 std::string* error) {
  if (result == nullptr) {
    if (error != nullptr) {
      *error = "null ArCycleResult output";
    }
    return false;
  }
  if (payload_bytes.empty()) {
    if (error != nullptr) {
      *error = "empty ArCycleResult flatbuffers payload";
    }
    return false;
  }

  const std::uint8_t* data = reinterpret_cast<const std::uint8_t*>(payload_bytes.data());
  flatbuffers::Verifier verifier(data, payload_bytes.size());
  const fb::ArCycleResult* root = flatbuffers::GetRoot<fb::ArCycleResult>(data);
  if (root == nullptr || !root->Verify(verifier)) {
    if (error != nullptr) {
      *error = "invalid ArCycleResult flatbuffers payload";
    }
    return false;
  }

  *result = DecodeCycleResult(root);
  return true;
}

std::string EncodeSessionConfigFlatbuffer(const config::ArSessionConfig& config) {
  flatbuffers::FlatBufferBuilder builder;
  const flatbuffers::Offset<session_fb::ArSessionConfig> root =
      session_fb::CreateArSessionConfig(
          builder, EncodeSessionDetectionConfig(&builder, config.hardware),
          EncodeSessionOrientation(&builder, config.mission.orientation),
          EncodeSessionPolicyConfig(&builder, config.policy),
          static_cast<int>(config.environment.jamming_sensitivity_profile),
          EncodeEnvironmentDefaultConfig(&builder, config.environment),
          config.mission.power_on);
  builder.Finish(root, session_fb::ArSessionConfigIdentifier());

  const std::uint8_t* buffer = builder.GetBufferPointer();
  return std::string(reinterpret_cast<const char*>(buffer),
                     reinterpret_cast<const char*>(buffer) + builder.GetSize());
}

bool DecodeSessionConfigFlatbuffer(const std::string& payload_bytes, config::ArSessionConfig* config,
                                   std::string* error) {
  if (config == nullptr) {
    if (error != nullptr) {
      *error = "null config::ArSessionConfig output";
    }
    return false;
  }
  if (payload_bytes.empty()) {
    if (error != nullptr) {
      *error = "empty config::ArSessionConfig flatbuffers payload";
    }
    return false;
  }

  const std::uint8_t* data = reinterpret_cast<const std::uint8_t*>(payload_bytes.data());
  flatbuffers::Verifier verifier(data, payload_bytes.size());
  if (!session_fb::VerifyArSessionConfigBuffer(verifier)) {
    if (error != nullptr) {
      *error = "invalid config::ArSessionConfig flatbuffers payload";
    }
    return false;
  }

  const session_fb::ArSessionConfig* root = session_fb::GetArSessionConfig(data);
  config->hardware = DecodeSessionDetectionConfig(root->hardware_detection());
  config->mission.orientation = DecodeSessionOrientation(root->mission_orientation());
  config->mission.power_on = root->power_on();
  config->policy = DecodeSessionPolicyConfig(root->policy());
  config->environment = DecodeEnvironmentDefaultConfig(root->environment_default_config());
  config->environment.jamming_sensitivity_profile =
      static_cast<config::JammingSensitivityProfile>(root->jamming_sensitivity_profile());
  return true;
}

std::string EncodeRuntimeConfigPatchFlatbuffer(const config::ArRuntimeConfigPatch& patch) {
  flatbuffers::FlatBufferBuilder builder;
  const flatbuffers::Offset<session_fb::ArRuntimeConfigPatch> root =
      session_fb::CreateArRuntimeConfigPatch(
          builder, patch.has_mission, EncodeSessionOrientation(&builder, patch.mission.orientation),
          patch.has_policy, EncodeSessionPolicyConfig(&builder, patch.policy),
          patch.has_environment,
          EncodeSessionEnvironmentRuntimeConfigPatch(&builder, patch.environment),
          patch.has_work_mode, static_cast<int>(patch.work_mode), patch.has_scan_center_deg,
          EncodeSessionAzEl(&builder, patch.scan_center_deg), patch.has_dwell_center_deg,
          EncodeSessionAzEl(&builder, patch.dwell_center_deg), patch.has_commanded_beamwidth_deg,
          EncodeSessionCommandedBeamwidth(&builder, patch.commanded_beamwidth_deg),
          patch.has_commanded_beamwidth_enabled, patch.commanded_beamwidth_enabled,
          patch.has_sensor_enabled, patch.sensor_enabled);
  builder.Finish(root);

  const std::uint8_t* buffer = builder.GetBufferPointer();
  return std::string(reinterpret_cast<const char*>(buffer),
                     reinterpret_cast<const char*>(buffer) + builder.GetSize());
}

bool DecodeRuntimeConfigPatchFlatbuffer(const std::string& payload_bytes,
                                        config::ArRuntimeConfigPatch* patch,
                                        std::string* error) {
  if (patch == nullptr) {
    if (error != nullptr) {
      *error = "null ArRuntimeConfigPatch output";
    }
    return false;
  }
  if (payload_bytes.empty()) {
    if (error != nullptr) {
      *error = "empty ArRuntimeConfigPatch flatbuffers payload";
    }
    return false;
  }

  const std::uint8_t* data = reinterpret_cast<const std::uint8_t*>(payload_bytes.data());
  flatbuffers::Verifier verifier(data, payload_bytes.size());
  const session_fb::ArRuntimeConfigPatch* root =
      flatbuffers::GetRoot<session_fb::ArRuntimeConfigPatch>(data);
  if (root == nullptr || !root->Verify(verifier)) {
    if (error != nullptr) {
      *error = "invalid ArRuntimeConfigPatch flatbuffers payload";
    }
    return false;
  }

  patch->has_mission = root->has_mission();
  patch->mission.orientation = DecodeSessionOrientation(root->mission_orientation());
  patch->has_policy = root->has_policy();
  patch->policy = DecodeSessionPolicyConfig(root->policy());
  patch->has_environment = root->has_environment();
  patch->environment =
      DecodeSessionEnvironmentRuntimeConfigPatch(root->environment());
  patch->has_work_mode = root->has_work_mode();
  patch->work_mode = static_cast<config::ArWorkMode>(root->work_mode());
  patch->has_scan_center_deg = root->has_scan_center_deg();
  patch->scan_center_deg = DecodeSessionAzEl(root->scan_center_deg());
  patch->has_dwell_center_deg = root->has_dwell_center_deg();
  patch->dwell_center_deg = DecodeSessionAzEl(root->dwell_center_deg());
  patch->has_commanded_beamwidth_deg = root->has_commanded_beamwidth_deg();
  patch->commanded_beamwidth_deg = DecodeSessionCommandedBeamwidth(root->commanded_beamwidth_deg());
  patch->has_commanded_beamwidth_enabled = root->has_commanded_beamwidth_enabled();
  patch->commanded_beamwidth_enabled = root->commanded_beamwidth_enabled();
  patch->has_sensor_enabled = root->has_sensor_enabled();
  patch->sensor_enabled = root->sensor_enabled();
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
