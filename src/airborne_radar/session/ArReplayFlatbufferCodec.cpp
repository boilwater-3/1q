#include "airborne_radar/session/ArReplayFlatbufferCodec.h"

#include <cstdint>
#include <string>
#include <vector>

#include "1q/airborne_radar/session/TrackStateSnapshot.h"
#include "airborne_radar/session/generated/airborne_radar_replay_generated.h"
#include "airborne_radar/session/generated/airborne_radar_session_replay_generated.h"
#include "common/replay/ReplayFlatbufferCodecSupport.h"

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

flatbuffers::Offset<fb::ArSceneTarget> EncodeSceneTarget(flatbuffers::FlatBufferBuilder* builder,
                                                         const ArSceneTarget& value) {
  return fb::CreateArSceneTarget(
      *builder, value.external_target_id, value.velocity_x, value.velocity_y, value.velocity_z,
      value.rcs, value.range_m, value.position_x, value.position_y, value.position_z,
      value.target_swerling_type, builder->CreateString(value.target_name));
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
    flatbuffers::FlatBufferBuilder* builder, const config::VegetationScatterPhysicsConfig& value) {
  return fb::CreateSurfaceObservation(*builder, static_cast<int>(value.cover_profile),
                                      value.enable_physical_model);
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

flatbuffers::Offset<fb::DecisionTrackStateSnapshot> EncodeTrackStateSnapshot(
    flatbuffers::FlatBufferBuilder* builder, const session::TrackStateSnapshot& value) {
  return fb::CreateDecisionTrackStateSnapshot(
      *builder, value.association_key, value.external_target_id, static_cast<int>(value.status),
      value.position_x, value.position_y, value.position_z, value.velocity_x, value.velocity_y,
      value.velocity_z, value.speed, value.acceleration_x, value.acceleration_y,
      value.acceleration_z, value.acceleration, value.rcs, value.hit_count, value.miss_count,
      builder->CreateString(value.target_type), value.target_probability,
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
      static_cast<std::uint64_t>(
          CountTracksByStatus(value.tracks, session::TrackStatus::kConfirmed)),
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

flatbuffers::Offset<fb::ArControlProfile> EncodeArControlProfile(
    flatbuffers::FlatBufferBuilder* builder, const session::ArControlProfile& value) {
  return fb::CreateArControlProfile(
      *builder, value.version, value.enable_lpi_power_control, value.lpi_power_scale,
      value.enable_lpi_beamforming, value.lpi_dwell_scale, value.enable_agility_frequency,
      value.agility_frequency_hop_phase, value.enable_sidelobe_canceller,
      value.enable_adaptive_beamforming, value.enable_eccm_rejitter, value.eccm_burnthrough_gain);
}

session::ArControlProfile DecodeArControlProfile(const fb::ArControlProfile* value) {
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

flatbuffers::Offset<fb::ArInterferenceObservation> EncodeArInterferenceObservation(
    flatbuffers::FlatBufferBuilder* builder,
    const session::ArInterferenceObservation& value) {
  return fb::CreateArInterferenceObservation(
      *builder, value.observation_id, value.estimated_bearing_azimuth_deg,
      value.estimated_bearing_elevation_deg, value.estimated_off_boresight_deg,
      value.estimated_center_frequency_hz, value.estimated_bandwidth_hz,
      static_cast<int>(value.estimated_waveform_kind), value.jammer_to_noise_db,
      value.bearing_standard_deviation_deg, value.frequency_standard_deviation_hz,
      value.bandwidth_standard_deviation_hz);
}

flatbuffers::Offset<fb::DecisionInputFrame> EncodeDecisionInputFrame(
    flatbuffers::FlatBufferBuilder* builder, const session::DecisionInputFrame& value) {
  std::vector<flatbuffers::Offset<fb::TrackStateSnapshot>> track_offsets;
  track_offsets.reserve(value.tracks.size());
  for (std::size_t i = 0; i < value.tracks.size(); ++i) {
    track_offsets.push_back(EncodeTrackSnapshot(builder, value.tracks[i]));
  }
  std::vector<flatbuffers::Offset<fb::ArInterferenceObservation>> observation_offsets;
  observation_offsets.reserve(value.interference_observations.size());
  for (const session::ArInterferenceObservation& observation :
       value.interference_observations) {
    observation_offsets.push_back(EncodeArInterferenceObservation(builder, observation));
  }
  const session::AssociationQualityInfo& association = value.association_quality_info;
  const session::PerceptionQualityInfo& perception = value.perception_quality_info;
  return fb::CreateDecisionInputFrame(
      *builder, value.cycle_index, value.batch_id, builder->CreateVector(observation_offsets),
      fb::CreateAssociationQualityInfo(
          *builder, association.match_rate, association.new_track_rate,
          association.missed_track_rate, association.mean_match_cost, association.p95_match_cost,
          association.association_stress),
      fb::CreatePerceptionQualityInfo(*builder,
                                      static_cast<std::uint64_t>(perception.input_target_count),
                                      static_cast<std::uint64_t>(perception.detection_count),
                                      perception.detection_rate, perception.detection_stress),
      builder->CreateVector(track_offsets));
}

flatbuffers::Offset<fb::DecisionObservation> EncodeDecisionObservation(
    flatbuffers::FlatBufferBuilder* builder, const session::DecisionObservation& value) {
  return fb::CreateDecisionObservation(
      *builder, EncodeDecisionInputFrame(builder, value.input_frame),
      EncodeArControlProfile(builder, value.active_control_profile));
}

flatbuffers::Offset<fb::TacticalProposal> EncodeTacticalProposal(
    flatbuffers::FlatBufferBuilder* builder, const session::TacticalProposal& value) {
  const session::ControlDirective& directive = value.directive;
  return fb::CreateTacticalProposal(
      *builder,
      fb::CreateControlDirective(*builder, static_cast<int>(directive.type),
                                 static_cast<int>(directive.source), directive.has_requested_value,
                                 directive.requested_value),
      value.priority, builder->CreateString(value.rationale));
}

flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<fb::TacticalProposal>>>
EncodeTacticalProposals(flatbuffers::FlatBufferBuilder* builder,
                        const std::vector<session::TacticalProposal>& values) {
  std::vector<flatbuffers::Offset<fb::TacticalProposal>> offsets;
  offsets.reserve(values.size());
  for (std::size_t i = 0; i < values.size(); ++i) {
    offsets.push_back(EncodeTacticalProposal(builder, values[i]));
  }
  return builder->CreateVector(offsets);
}

flatbuffers::Offset<fb::ExternalDecisionResponse> EncodeExternalDecisionResponse(
    flatbuffers::FlatBufferBuilder* builder, const session::ExternalDecisionResponse& value) {
  return fb::CreateExternalDecisionResponse(*builder, value.source_cycle_index,
                                            value.source_batch_id,
                                            EncodeTacticalProposals(builder, value.proposals));
}

session::ArInterferenceObservation DecodeArInterferenceObservation(
    const fb::ArInterferenceObservation* value) {
  session::ArInterferenceObservation result;
  if (value != nullptr) {
    result.observation_id = value->observation_id();
    result.estimated_bearing_azimuth_deg = value->estimated_bearing_azimuth_deg();
    result.estimated_bearing_elevation_deg = value->estimated_bearing_elevation_deg();
    result.estimated_off_boresight_deg = value->estimated_off_boresight_deg();
    result.estimated_center_frequency_hz = value->estimated_center_frequency_hz();
    result.estimated_bandwidth_hz = value->estimated_bandwidth_hz();
    result.estimated_waveform_kind = static_cast<oneq::electromagnetics::RfSceneWaveformKind>(
        value->estimated_waveform_kind());
    result.jammer_to_noise_db = value->jammer_to_noise_db();
    result.bearing_standard_deviation_deg = value->bearing_standard_deviation_deg();
    result.frequency_standard_deviation_hz = value->frequency_standard_deviation_hz();
    result.bandwidth_standard_deviation_hz = value->bandwidth_standard_deviation_hz();
  }
  return result;
}

session::DecisionInputFrame DecodeDecisionInputFrame(const fb::DecisionInputFrame* value) {
  session::DecisionInputFrame result;
  if (value != nullptr) {
    result.cycle_index = value->cycle_index();
    result.batch_id = value->batch_id();
    if (value->interference_observations() != nullptr) {
      result.interference_observations.reserve(value->interference_observations()->size());
      for (flatbuffers::uoffset_t index = 0U;
           index < value->interference_observations()->size(); ++index) {
        result.interference_observations.push_back(
            DecodeArInterferenceObservation(value->interference_observations()->Get(index)));
      }
    }
    if (value->association_quality_info() != nullptr) {
      const fb::AssociationQualityInfo* association = value->association_quality_info();
      result.association_quality_info.match_rate = association->match_rate();
      result.association_quality_info.new_track_rate = association->new_track_rate();
      result.association_quality_info.missed_track_rate = association->missed_track_rate();
      result.association_quality_info.mean_match_cost = association->mean_match_cost();
      result.association_quality_info.p95_match_cost = association->p95_match_cost();
      result.association_quality_info.association_stress = association->association_stress();
    }
    if (value->perception_quality_info() != nullptr) {
      const fb::PerceptionQualityInfo* perception = value->perception_quality_info();
      result.perception_quality_info.input_target_count =
          static_cast<std::size_t>(perception->input_target_count());
      result.perception_quality_info.detection_count =
          static_cast<std::size_t>(perception->detection_count());
      result.perception_quality_info.detection_rate = perception->detection_rate();
      result.perception_quality_info.detection_stress = perception->detection_stress();
    }
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

session::DecisionObservation DecodeDecisionObservation(const fb::DecisionObservation* value) {
  session::DecisionObservation result;
  if (value != nullptr) {
    result.input_frame = DecodeDecisionInputFrame(value->input_frame());
    result.active_control_profile = DecodeArControlProfile(value->active_control_profile());
  }
  return result;
}

session::TacticalProposal DecodeTacticalProposal(const fb::TacticalProposal* value) {
  session::TacticalProposal result;
  if (value != nullptr) {
    if (value->directive() != nullptr) {
      result.directive.type =
          static_cast<session::ControlDirectiveType>(value->directive()->type());
      result.directive.source =
          static_cast<session::ControlDirectiveSource>(value->directive()->source());
      result.directive.has_requested_value = value->directive()->has_requested_value();
      result.directive.requested_value = value->directive()->requested_value();
    }
    result.priority = value->priority();
    if (value->rationale() != nullptr) {
      result.rationale = value->rationale()->str();
    }
  }
  return result;
}

std::vector<session::TacticalProposal> DecodeTacticalProposals(
    const flatbuffers::Vector<flatbuffers::Offset<fb::TacticalProposal>>* values) {
  std::vector<session::TacticalProposal> result;
  if (values != nullptr) {
    result.reserve(values->size());
    for (flatbuffers::uoffset_t i = 0; i < values->size(); ++i) {
      result.push_back(DecodeTacticalProposal(values->Get(i)));
    }
  }
  return result;
}

session::ExternalDecisionResponse DecodeExternalDecisionResponse(
    const fb::ExternalDecisionResponse* value) {
  session::ExternalDecisionResponse result;
  if (value != nullptr) {
    result.source_cycle_index = value->source_cycle_index();
    result.source_batch_id = value->source_batch_id();
    result.proposals = DecodeTacticalProposals(value->proposals());
  }
  return result;
}

flatbuffers::Offset<fb::ArDecisionReplayState> EncodeDecisionReplayState(
    flatbuffers::FlatBufferBuilder* builder, const ArDecisionReplayState& value) {
  return fb::CreateArDecisionReplayState(
      *builder, value.has_pending_internal_decision, value.pending_internal_cycle_index,
      value.pending_internal_batch_id,
      EncodeTacticalProposals(builder, value.pending_internal_proposals),
      static_cast<int>(value.applied_decision_source), value.applied_decision_cycle_index,
      value.applied_decision_batch_id,
      EncodeTacticalProposals(builder, value.applied_decision_proposals),
      value.has_pending_external_decision,
      EncodeExternalDecisionResponse(builder, value.pending_external_decision),
      value.reducer_state.lpi_hold_cycles_remaining, value.reducer_state.eccm_hold_cycles_remaining,
      value.reducer_state.lpi_cooldown_cycles_remaining,
      value.reducer_state.eccm_cooldown_cycles_remaining);
}

ArDecisionReplayState DecodeDecisionReplayState(const fb::ArDecisionReplayState* value) {
  ArDecisionReplayState result;
  if (value != nullptr) {
    result.has_pending_internal_decision = value->has_pending_internal_decision();
    result.pending_internal_cycle_index = value->pending_internal_cycle_index();
    result.pending_internal_batch_id = value->pending_internal_batch_id();
    result.pending_internal_proposals =
        DecodeTacticalProposals(value->pending_internal_proposals());
    result.applied_decision_source =
        static_cast<session::DecisionControlSource>(value->applied_decision_source());
    result.applied_decision_cycle_index = value->applied_decision_cycle_index();
    result.applied_decision_batch_id = value->applied_decision_batch_id();
    result.applied_decision_proposals =
        DecodeTacticalProposals(value->applied_decision_proposals());
    result.has_pending_external_decision = value->has_pending_external_decision();
    result.pending_external_decision =
        DecodeExternalDecisionResponse(value->pending_external_decision());
    result.reducer_state.lpi_hold_cycles_remaining = value->lpi_hold_cycles_remaining();
    result.reducer_state.eccm_hold_cycles_remaining = value->eccm_hold_cycles_remaining();
    result.reducer_state.lpi_cooldown_cycles_remaining = value->lpi_cooldown_cycles_remaining();
    result.reducer_state.eccm_cooldown_cycles_remaining = value->eccm_cooldown_cycles_remaining();
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
  const auto frequency_plan = builder->CreateVector(value.transmitter.frequency_plan_hz);
  session_fb::TransmitterConfigBuilder transmitter_builder(*builder);
  transmitter_builder.add_equipment_id(value.transmitter.equipment_id);
  transmitter_builder.add_peak_power_w(value.transmitter.peak_power_w);
  transmitter_builder.add_frequency_hz(value.transmitter.frequency_hz);
  transmitter_builder.add_bandwidth_hz(value.transmitter.bandwidth_hz);
  transmitter_builder.add_pulse_width_s(value.transmitter.pulse_width_s);
  transmitter_builder.add_prf_hz(value.transmitter.prf_hz);
  transmitter_builder.add_transmit_loss_db(value.transmitter.transmit_loss_db);
  transmitter_builder.add_maximum_peak_power_w(value.transmitter.maximum_peak_power_w);
  transmitter_builder.add_maximum_duty_cycle(value.transmitter.maximum_duty_cycle);
  transmitter_builder.add_maximum_pulse_energy_j(value.transmitter.maximum_pulse_energy_j);
  transmitter_builder.add_frequency_plan_hz(frequency_plan);
  const flatbuffers::Offset<session_fb::TransmitterConfig> transmitter =
      transmitter_builder.Finish();
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
  std::vector<flatbuffers::Offset<session_fb::RfCoSiteIsolationPath>> co_site_paths;
  co_site_paths.reserve(value.receiver.co_site_paths.size());
  for (const oneq::electromagnetics::RfCoSiteIsolationPath& path :
       value.receiver.co_site_paths) {
    co_site_paths.push_back(session_fb::CreateRfCoSiteIsolationPath(
        *builder, path.transmitter_equipment_id, path.receiver_equipment_id,
        path.isolation_db));
  }
  session_fb::ReceiverConfigBuilder receiver_builder(*builder);
  receiver_builder.add_equipment_id(value.receiver.equipment_id);
  receiver_builder.add_noise_figure_db(value.receiver.noise_figure_db);
  receiver_builder.add_receive_loss_db(value.receiver.receive_loss_db);
  receiver_builder.add_polarization(static_cast<int>(value.receiver.polarization));
  receiver_builder.add_cross_polarization_isolation_db(
      value.receiver.cross_polarization_isolation_db);
  receiver_builder.add_minimum_far_field_range_m(value.receiver.minimum_far_field_range_m);
  receiver_builder.add_has_co_site_isolation(value.receiver.has_co_site_isolation);
  receiver_builder.add_co_site_isolation_db(value.receiver.co_site_isolation_db);
  receiver_builder.add_maximum_linear_input_power_w(
      value.receiver.maximum_linear_input_power_w);
  receiver_builder.add_preselector_bandwidth_hz(value.receiver.preselector_bandwidth_hz);
  receiver_builder.add_interference_observation_jn_gate_db(
      value.receiver.interference_observation_jn_gate_db);
  receiver_builder.add_scene_polarization(
      static_cast<int>(value.receiver.scene_polarization));
  receiver_builder.add_co_site_paths(builder->CreateVector(co_site_paths));
  const flatbuffers::Offset<session_fb::ReceiverConfig> receiver =
      receiver_builder.Finish();
  const flatbuffers::Offset<session_fb::RcsPhysicsConfig> rcs_physics =
      session_fb::CreateRcsPhysicsConfig(
          *builder, value.rcs_physics.enable_physical_rcs, value.rcs_physics.physics_mix_ratio,
          value.rcs_physics.cylinder_weight, value.rcs_physics.min_equivalent_radius_m,
          value.rcs_physics.max_equivalent_radius_m, value.rcs_physics.min_rcs_m2,
          value.rcs_physics.max_rcs_m2, value.rcs_physics.bistatic_psi_offset_deg);
  return session_fb::CreateDetectionConfig(
      *builder, static_cast<int>(config::profiles::SwerlingModel::kSwerling0), transmitter, antenna,
      receiver, rcs_physics);
}

flatbuffers::Offset<session_fb::ArPolicyConfig> EncodeSessionPolicyConfig(
    flatbuffers::FlatBufferBuilder* builder, const config::ArPolicyConfig& value) {
  const flatbuffers::Offset<session_fb::ArDetectionPolicyConfig> detection =
      session_fb::CreateArDetectionPolicyConfig(*builder, value.detection.minimum_snr_db,
                                                value.detection.pfa, value.detection.pulse_count,
                                                value.detection.minimum_detection_margin_db);
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
  const flatbuffers::Offset<session_fb::DecisionControlConfig> decision_control =
      session_fb::CreateDecisionControlConfig(
          *builder, value.decision_control.lpi_hold_cycles_after_request,
          value.decision_control.eccm_hold_cycles_after_request,
          value.decision_control.lpi_cooldown_cycles_after_release,
          value.decision_control.eccm_cooldown_cycles_after_release);
  return session_fb::CreateArPolicyConfig(*builder, detection, beam_control, association, tracking,
                                          lifecycle, imm, decision_control);
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
EncodeSessionVegetationScatterPhysicsConfig(flatbuffers::FlatBufferBuilder* builder,
                                            const config::VegetationScatterPhysicsConfig& value) {
  return session_fb::CreateVegetationScatterPhysicsConfig(
      *builder, static_cast<int>(value.cover_profile), value.enable_physical_model);
}

flatbuffers::Offset<session_fb::JammerEmitterState> EncodeSessionJammerEmitterState(
    flatbuffers::FlatBufferBuilder* builder, const config::JammerEmitterState& value) {
  return session_fb::CreateJammerEmitterState(
      *builder, static_cast<int>(value.technique), value.power_db, value.js_db, value.position_x,
      value.position_y, value.position_z, value.angular_span_deg, value.confidence);
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
EncodeSessionEnvironmentRuntimeConfigPatch(flatbuffers::FlatBufferBuilder* builder,
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

config::ArOrientationConfig DecodeSessionOrientation(const session_fb::ArOrientationConfig* value) {
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
    const session_fb::TransmitterConfig* transmitter = value->transmitter();
    if (transmitter != nullptr) {
      result.transmitter.equipment_id = transmitter->equipment_id();
      result.transmitter.peak_power_w = transmitter->peak_power_w();
      result.transmitter.frequency_hz = transmitter->frequency_hz();
      result.transmitter.bandwidth_hz = transmitter->bandwidth_hz();
      result.transmitter.pulse_width_s = transmitter->pulse_width_s();
      result.transmitter.prf_hz = transmitter->prf_hz();
      result.transmitter.transmit_loss_db = transmitter->transmit_loss_db();
      result.transmitter.maximum_peak_power_w = transmitter->maximum_peak_power_w();
      result.transmitter.maximum_duty_cycle = transmitter->maximum_duty_cycle();
      result.transmitter.maximum_pulse_energy_j = transmitter->maximum_pulse_energy_j();
      if (transmitter->frequency_plan_hz() != nullptr) {
        result.transmitter.frequency_plan_hz.assign(transmitter->frequency_plan_hz()->begin(),
                                                    transmitter->frequency_plan_hz()->end());
      }
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
      result.receiver.equipment_id = receiver->equipment_id();
      result.receiver.noise_figure_db = receiver->noise_figure_db();
      result.receiver.receive_loss_db = receiver->receive_loss_db();
      result.receiver.polarization =
          static_cast<oneq::electromagnetics::RfPolarization>(receiver->polarization());
      result.receiver.cross_polarization_isolation_db =
          receiver->cross_polarization_isolation_db();
      result.receiver.minimum_far_field_range_m = receiver->minimum_far_field_range_m();
      result.receiver.has_co_site_isolation = receiver->has_co_site_isolation();
      result.receiver.co_site_isolation_db = receiver->co_site_isolation_db();
      result.receiver.maximum_linear_input_power_w =
          receiver->maximum_linear_input_power_w();
      result.receiver.preselector_bandwidth_hz = receiver->preselector_bandwidth_hz();
      result.receiver.interference_observation_jn_gate_db =
          receiver->interference_observation_jn_gate_db();
      result.receiver.scene_polarization =
          static_cast<oneq::electromagnetics::RfScenePolarization>(
              receiver->scene_polarization());
      result.receiver.co_site_paths.clear();
      if (receiver->co_site_paths() != nullptr) {
        result.receiver.co_site_paths.reserve(receiver->co_site_paths()->size());
        for (const session_fb::RfCoSiteIsolationPath* encoded :
             *receiver->co_site_paths()) {
          oneq::electromagnetics::RfCoSiteIsolationPath path;
          path.transmitter_equipment_id = encoded->transmitter_equipment_id();
          path.receiver_equipment_id = encoded->receiver_equipment_id();
          path.isolation_db = encoded->isolation_db();
          result.receiver.co_site_paths.push_back(path);
        }
      }
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
      result.detection.minimum_detection_margin_db = detection->minimum_detection_margin_db();
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
    const session_fb::DecisionControlConfig* decision_control = value->decision_control();
    if (decision_control != nullptr) {
      result.decision_control.lpi_hold_cycles_after_request =
          decision_control->lpi_hold_cycles_after_request();
      result.decision_control.eccm_hold_cycles_after_request =
          decision_control->eccm_hold_cycles_after_request();
      result.decision_control.lpi_cooldown_cycles_after_release =
          decision_control->lpi_cooldown_cycles_after_release();
      result.decision_control.eccm_cooldown_cycles_after_release =
          decision_control->eccm_cooldown_cycles_after_release();
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

flatbuffers::Offset<fb::Vector3d> EncodeRfV2Position(flatbuffers::FlatBufferBuilder* builder,
                                                     const oneq::coordinate::EcefPositionM& value) {
  return fb::CreateVector3d(*builder, value.x_m, value.y_m, value.z_m);
}

flatbuffers::Offset<fb::Vector3d> EncodeRfV2Velocity(
    flatbuffers::FlatBufferBuilder* builder, const oneq::coordinate::EcefVelocityMps& value) {
  return fb::CreateVector3d(*builder, value.x_mps, value.y_mps, value.z_mps);
}

flatbuffers::Offset<fb::RfSceneDirectionV2> EncodeRfV2Direction(
    flatbuffers::FlatBufferBuilder* builder,
    const oneq::electromagnetics::RfSceneDirection& value) {
  return fb::CreateRfSceneDirectionV2(*builder, value.x, value.y, value.z);
}

flatbuffers::Offset<fb::RfSceneAntennaPatternV2> EncodeRfV2Antenna(
    flatbuffers::FlatBufferBuilder* builder,
    const oneq::electromagnetics::RfSceneAntennaPattern& value) {
  return fb::CreateRfSceneAntennaPatternV2(
      *builder, EncodeRfV2Direction(builder, value.boresight_ecef), value.peak_gain_dbi,
      value.half_power_beamwidth_deg, value.sidelobe_level_db, value.backlobe_level_db,
      value.cross_polarization_isolation_db);
}

flatbuffers::Offset<fb::RfEmissionIdentityV2> EncodeRfV2Identity(
    flatbuffers::FlatBufferBuilder* builder,
    const oneq::electromagnetics::RfEmissionIdentity& value) {
  return fb::CreateRfEmissionIdentityV2(*builder, value.platform_id, value.equipment_id,
                                        value.emission_id);
}

flatbuffers::Offset<fb::RfWaveformScheduleV2> EncodeRfV2Waveform(
    flatbuffers::FlatBufferBuilder* builder,
    const oneq::electromagnetics::RfWaveformSchedule& value) {
  return fb::CreateRfWaveformScheduleV2(
      *builder, static_cast<int>(value.kind), value.activity_start_time_s,
      value.activity_duration_s, value.center_frequency_hz, value.occupied_bandwidth_hz,
      value.transmit_power_w, value.pulse_width_s, value.pulse_repetition_interval_s,
      value.first_pulse_time_s, value.pulse_count, value.pulse_jitter_fraction, value.timing_seed,
      value.timing_epoch, value.sweep_start_frequency_hz, value.sweep_stop_frequency_hz,
      value.sweep_period_s);
}

flatbuffers::Offset<fb::RfSceneEmissionV2> EncodeRfV2Emission(
    flatbuffers::FlatBufferBuilder* builder, const oneq::electromagnetics::RfSceneEmission& value) {
  return fb::CreateRfSceneEmissionV2(*builder, EncodeRfV2Identity(builder, value.identity),
                                     EncodeRfV2Position(builder, value.position_ecef_m),
                                     EncodeRfV2Velocity(builder, value.velocity_ecef_mps),
                                     EncodeRfV2Antenna(builder, value.antenna),
                                     static_cast<int>(value.polarization),
                                     EncodeRfV2Waveform(builder, value.waveform));
}

flatbuffers::Offset<fb::RfSceneFrameV2> EncodeRfV2Scene(
    flatbuffers::FlatBufferBuilder* builder, const oneq::electromagnetics::RfSceneFrame& value) {
  std::vector<flatbuffers::Offset<fb::RfSceneEmissionV2>> emissions;
  emissions.reserve(value.emissions.size());
  for (const auto& emission : value.emissions) {
    emissions.push_back(EncodeRfV2Emission(builder, emission));
  }
  return fb::CreateRfSceneFrameV2(*builder, value.world_cycle_index, value.window_start_time_s,
                                  value.window_duration_s, builder->CreateVector(emissions));
}

flatbuffers::Offset<fb::RfSceneReceiverStateV2> EncodeRfV2Receiver(
    flatbuffers::FlatBufferBuilder* builder,
    const oneq::electromagnetics::RfSceneReceiverState& value) {
  std::vector<flatbuffers::Offset<fb::RfCoSiteIsolationPathV2>> paths;
  paths.reserve(value.co_site_paths.size());
  for (const auto& path : value.co_site_paths) {
    paths.push_back(fb::CreateRfCoSiteIsolationPathV2(
        *builder, path.transmitter_equipment_id, path.receiver_equipment_id, path.isolation_db));
  }
  return fb::CreateRfSceneReceiverStateV2(
      *builder, value.platform_id, value.equipment_id,
      EncodeRfV2Position(builder, value.position_ecef_m),
      EncodeRfV2Velocity(builder, value.velocity_ecef_mps),
      EncodeRfV2Antenna(builder, value.antenna), static_cast<int>(value.polarization),
      value.window_start_time_s, value.window_duration_s, value.center_frequency_hz,
      value.bandwidth_hz, value.receiver_system_loss_db, value.minimum_far_field_range_m,
      builder->CreateVector(paths));
}

flatbuffers::Offset<fb::ArPreparedCycleTokenV2> EncodePreparedToken(
    flatbuffers::FlatBufferBuilder* builder, const ArPreparedCycleToken& value) {
  return fb::CreateArPreparedCycleTokenV2(*builder, value.value, value.world_cycle_index);
}

flatbuffers::Offset<fb::ArReceiverOperatingStateV2> EncodeReceiverOperatingState(
    flatbuffers::FlatBufferBuilder* builder, const ArReceiverOperatingState& value) {
  std::vector<flatbuffers::Offset<fb::RfSceneDirectionV2>> nulls;
  nulls.reserve(value.adaptive_nulls_ecef.size());
  for (const auto& direction : value.adaptive_nulls_ecef) {
    nulls.push_back(EncodeRfV2Direction(builder, direction));
  }
  return fb::CreateArReceiverOperatingStateV2(
      *builder, EncodeRfV2Receiver(builder, value.rf_receiver),
      fb::CreateAzimuthElevationDeg32(*builder, value.beam_pointing_deg.az_deg,
                                      value.beam_pointing_deg.el_deg),
      value.matched_filter_bandwidth_hz, value.receiver_noise_figure_db,
      value.maximum_linear_input_power_w, value.transmit_receive_blanking_enabled,
      builder->CreateVector(nulls));
}

flatbuffers::Offset<fb::ArPrepareCycleInputV2> EncodePrepareInput(
    flatbuffers::FlatBufferBuilder* builder, const ArPrepareCycleInput& value) {
  return fb::CreateArPrepareCycleInputV2(
      *builder, value.world_cycle_index, value.window_start_time_s, value.window_duration_s,
      value.platform_id, EncodeRfV2Position(builder, value.platform_position_ecef_m),
      EncodeRfV2Velocity(builder, value.platform_velocity_ecef_mps),
      fb::CreateEulerAnglesDeg64(*builder, value.radar_frame_attitude_deg.yaw_deg,
                                 value.radar_frame_attitude_deg.pitch_deg,
                                 value.radar_frame_attitude_deg.roll_deg),
      fb::CreateAzimuthElevationDeg32(*builder, value.beam_pointing_deg.az_deg,
                                      value.beam_pointing_deg.el_deg));
}

flatbuffers::Offset<fb::ArPrepareCycleResultV2> EncodePrepareResult(
    flatbuffers::FlatBufferBuilder* builder, const ArPrepareCycleResult& value) {
  return fb::CreateArPrepareCycleResultV2(
      *builder, static_cast<int>(value.status), EncodePreparedToken(builder, value.token),
      value.has_emission, EncodeRfV2Emission(builder, value.emission),
      EncodeReceiverOperatingState(builder, value.operating_state));
}

flatbuffers::Offset<fb::ArSessionReplayStateV2> EncodeSessionReplayState(
    flatbuffers::FlatBufferBuilder* builder, const ArSessionReplayState& value) {
  return fb::CreateArSessionReplayStateV2(
      *builder, value.has_prepared_cycle, EncodePreparedToken(builder, value.prepared_token),
      value.has_world_chronology, value.last_world_window_end_s, value.next_token_value,
      value.next_emission_id, value.successful_prepare_count, value.timing_seed,
      value.frequency_hop_index, value.has_pending_runtime_update,
      value.pending_execution_config_changed, value.pending_environment_scenario_config_changed,
      value.pending_jamming_sensitivity_profile_changed,
      EncodeDecisionReplayState(builder, value.decision_state));
}

oneq::coordinate::EcefPositionM DecodeRfV2Position(const fb::Vector3d* value) {
  oneq::coordinate::EcefPositionM result;
  if (value != nullptr) {
    result.x_m = value->x();
    result.y_m = value->y();
    result.z_m = value->z();
  }
  return result;
}

oneq::coordinate::EcefVelocityMps DecodeRfV2Velocity(const fb::Vector3d* value) {
  oneq::coordinate::EcefVelocityMps result;
  if (value != nullptr) {
    result.x_mps = value->x();
    result.y_mps = value->y();
    result.z_mps = value->z();
  }
  return result;
}

oneq::electromagnetics::RfSceneDirection DecodeRfV2Direction(const fb::RfSceneDirectionV2* value) {
  oneq::electromagnetics::RfSceneDirection result;
  if (value != nullptr) {
    result.x = value->x();
    result.y = value->y();
    result.z = value->z();
  }
  return result;
}

oneq::electromagnetics::RfSceneAntennaPattern DecodeRfV2Antenna(
    const fb::RfSceneAntennaPatternV2* value) {
  oneq::electromagnetics::RfSceneAntennaPattern result;
  if (value != nullptr) {
    result.boresight_ecef = DecodeRfV2Direction(value->boresight_ecef());
    result.peak_gain_dbi = value->peak_gain_dbi();
    result.half_power_beamwidth_deg = value->half_power_beamwidth_deg();
    result.sidelobe_level_db = value->sidelobe_level_db();
    result.backlobe_level_db = value->backlobe_level_db();
    result.cross_polarization_isolation_db = value->cross_polarization_isolation_db();
  }
  return result;
}

oneq::electromagnetics::RfWaveformSchedule DecodeRfV2Waveform(
    const fb::RfWaveformScheduleV2* value) {
  oneq::electromagnetics::RfWaveformSchedule result;
  if (value != nullptr) {
    result.kind = static_cast<oneq::electromagnetics::RfSceneWaveformKind>(value->kind());
    result.activity_start_time_s = value->activity_start_time_s();
    result.activity_duration_s = value->activity_duration_s();
    result.center_frequency_hz = value->center_frequency_hz();
    result.occupied_bandwidth_hz = value->occupied_bandwidth_hz();
    result.transmit_power_w = value->transmit_power_w();
    result.pulse_width_s = value->pulse_width_s();
    result.pulse_repetition_interval_s = value->pulse_repetition_interval_s();
    result.first_pulse_time_s = value->first_pulse_time_s();
    result.pulse_count = value->pulse_count();
    result.pulse_jitter_fraction = value->pulse_jitter_fraction();
    result.timing_seed = value->timing_seed();
    result.timing_epoch = value->timing_epoch();
    result.sweep_start_frequency_hz = value->sweep_start_frequency_hz();
    result.sweep_stop_frequency_hz = value->sweep_stop_frequency_hz();
    result.sweep_period_s = value->sweep_period_s();
  }
  return result;
}

oneq::electromagnetics::RfSceneEmission DecodeRfV2Emission(const fb::RfSceneEmissionV2* value) {
  oneq::electromagnetics::RfSceneEmission result;
  if (value != nullptr) {
    if (value->identity() != nullptr) {
      result.identity.platform_id = value->identity()->platform_id();
      result.identity.equipment_id = value->identity()->equipment_id();
      result.identity.emission_id = value->identity()->emission_id();
    }
    result.position_ecef_m = DecodeRfV2Position(value->position_ecef_m());
    result.velocity_ecef_mps = DecodeRfV2Velocity(value->velocity_ecef_mps());
    result.antenna = DecodeRfV2Antenna(value->antenna());
    result.polarization =
        static_cast<oneq::electromagnetics::RfScenePolarization>(value->polarization());
    result.waveform = DecodeRfV2Waveform(value->waveform());
  }
  return result;
}

oneq::electromagnetics::RfSceneFrame DecodeRfV2Scene(const fb::RfSceneFrameV2* value) {
  oneq::electromagnetics::RfSceneFrame result;
  if (value != nullptr) {
    result.world_cycle_index = value->world_cycle_index();
    result.window_start_time_s = value->window_start_time_s();
    result.window_duration_s = value->window_duration_s();
    if (value->emissions() != nullptr) {
      result.emissions.reserve(value->emissions()->size());
      for (const fb::RfSceneEmissionV2* emission : *value->emissions()) {
        result.emissions.push_back(DecodeRfV2Emission(emission));
      }
    }
  }
  return result;
}

oneq::electromagnetics::RfSceneReceiverState DecodeRfV2Receiver(
    const fb::RfSceneReceiverStateV2* value) {
  oneq::electromagnetics::RfSceneReceiverState result;
  if (value != nullptr) {
    result.platform_id = value->platform_id();
    result.equipment_id = value->equipment_id();
    result.position_ecef_m = DecodeRfV2Position(value->position_ecef_m());
    result.velocity_ecef_mps = DecodeRfV2Velocity(value->velocity_ecef_mps());
    result.antenna = DecodeRfV2Antenna(value->antenna());
    result.polarization =
        static_cast<oneq::electromagnetics::RfScenePolarization>(value->polarization());
    result.window_start_time_s = value->window_start_time_s();
    result.window_duration_s = value->window_duration_s();
    result.center_frequency_hz = value->center_frequency_hz();
    result.bandwidth_hz = value->bandwidth_hz();
    result.receiver_system_loss_db = value->receiver_system_loss_db();
    result.minimum_far_field_range_m = value->minimum_far_field_range_m();
    if (value->co_site_paths() != nullptr) {
      result.co_site_paths.reserve(value->co_site_paths()->size());
      for (const fb::RfCoSiteIsolationPathV2* encoded : *value->co_site_paths()) {
        oneq::electromagnetics::RfCoSiteIsolationPath path;
        path.transmitter_equipment_id = encoded->transmitter_equipment_id();
        path.receiver_equipment_id = encoded->receiver_equipment_id();
        path.isolation_db = encoded->isolation_db();
        result.co_site_paths.push_back(path);
      }
    }
  }
  return result;
}

ArPreparedCycleToken DecodePreparedToken(const fb::ArPreparedCycleTokenV2* value) {
  ArPreparedCycleToken result;
  if (value != nullptr) {
    result.value = value->value();
    result.world_cycle_index = value->world_cycle_index();
  }
  return result;
}

ArReceiverOperatingState DecodeReceiverOperatingState(const fb::ArReceiverOperatingStateV2* value) {
  ArReceiverOperatingState result;
  if (value != nullptr) {
    result.rf_receiver = DecodeRfV2Receiver(value->rf_receiver());
    if (value->beam_pointing_deg() != nullptr) {
      result.beam_pointing_deg.az_deg = value->beam_pointing_deg()->az_deg();
      result.beam_pointing_deg.el_deg = value->beam_pointing_deg()->el_deg();
    }
    result.matched_filter_bandwidth_hz = value->matched_filter_bandwidth_hz();
    result.receiver_noise_figure_db = value->receiver_noise_figure_db();
    result.maximum_linear_input_power_w = value->maximum_linear_input_power_w();
    result.transmit_receive_blanking_enabled = value->transmit_receive_blanking_enabled();
    if (value->adaptive_nulls_ecef() != nullptr) {
      result.adaptive_nulls_ecef.reserve(value->adaptive_nulls_ecef()->size());
      for (const fb::RfSceneDirectionV2* direction : *value->adaptive_nulls_ecef()) {
        result.adaptive_nulls_ecef.push_back(DecodeRfV2Direction(direction));
      }
    }
  }
  return result;
}

ArPrepareCycleInput DecodePrepareInput(const fb::ArPrepareCycleInputV2* value) {
  ArPrepareCycleInput result;
  if (value != nullptr) {
    result.world_cycle_index = value->world_cycle_index();
    result.window_start_time_s = value->window_start_time_s();
    result.window_duration_s = value->window_duration_s();
    result.platform_id = value->platform_id();
    result.platform_position_ecef_m = DecodeRfV2Position(value->platform_position_ecef_m());
    result.platform_velocity_ecef_mps = DecodeRfV2Velocity(value->platform_velocity_ecef_mps());
    if (value->radar_frame_attitude_deg() != nullptr) {
      result.radar_frame_attitude_deg.yaw_deg = value->radar_frame_attitude_deg()->yaw_deg();
      result.radar_frame_attitude_deg.pitch_deg = value->radar_frame_attitude_deg()->pitch_deg();
      result.radar_frame_attitude_deg.roll_deg = value->radar_frame_attitude_deg()->roll_deg();
    }
    if (value->beam_pointing_deg() != nullptr) {
      result.beam_pointing_deg.az_deg = value->beam_pointing_deg()->az_deg();
      result.beam_pointing_deg.el_deg = value->beam_pointing_deg()->el_deg();
    }
  }
  return result;
}

ArPrepareCycleResult DecodePrepareResult(const fb::ArPrepareCycleResultV2* value) {
  ArPrepareCycleResult result;
  if (value != nullptr) {
    result.status = static_cast<ArPrepareCycleStatus>(value->status());
    result.token = DecodePreparedToken(value->token());
    result.has_emission = value->has_emission();
    result.emission = DecodeRfV2Emission(value->emission());
    result.operating_state = DecodeReceiverOperatingState(value->operating_state());
  }
  return result;
}

ArSessionReplayState DecodeSessionReplayState(const fb::ArSessionReplayStateV2* value) {
  ArSessionReplayState result;
  if (value != nullptr) {
    result.has_prepared_cycle = value->has_prepared_cycle();
    result.prepared_token = DecodePreparedToken(value->prepared_token());
    result.has_world_chronology = value->has_world_chronology();
    result.last_world_window_end_s = value->last_world_window_end_s();
    result.next_token_value = value->next_token_value();
    result.next_emission_id = value->next_emission_id();
    result.successful_prepare_count = value->successful_prepare_count();
    result.timing_seed = value->timing_seed();
    result.frequency_hop_index = value->frequency_hop_index();
    result.has_pending_runtime_update = value->has_pending_runtime_update();
    result.pending_execution_config_changed = value->pending_execution_config_changed();
    result.pending_environment_scenario_config_changed =
        value->pending_environment_scenario_config_changed();
    result.pending_jamming_sensitivity_profile_changed =
        value->pending_jamming_sensitivity_profile_changed();
    result.decision_state = DecodeDecisionReplayState(value->decision_state());
  }
  return result;
}

template <typename FlatbufferType>
const FlatbufferType* TryGetReplayRoot(const std::string& payload_bytes, const char* payload_name,
                                       std::string* error) {
  if (payload_bytes.empty()) {
    if (error != nullptr) {
      *error = std::string("empty ") + payload_name + " flatbuffers payload";
    }
    return nullptr;
  }
  const std::uint8_t* data = reinterpret_cast<const std::uint8_t*>(payload_bytes.data());
  flatbuffers::Verifier verifier(data, payload_bytes.size());
  const FlatbufferType* root = flatbuffers::GetRoot<FlatbufferType>(data);
  if (root == nullptr || !root->Verify(verifier)) {
    if (error != nullptr) {
      *error = std::string("invalid ") + payload_name + " flatbuffers payload";
    }
    return nullptr;
  }
  return root;
}

}  // namespace

std::string EncodeExternalDecisionResponseFlatbuffer(
    const session::ExternalDecisionResponse& response) {
  flatbuffers::FlatBufferBuilder builder;
  const flatbuffers::Offset<fb::ExternalDecisionResponse> root =
      EncodeExternalDecisionResponse(&builder, response);
  builder.Finish(root);
  return oneq::common::replay::CopyFinishedFlatbuffer(builder);
}

bool DecodeExternalDecisionResponseFlatbuffer(const std::string& payload_bytes,
                                              session::ExternalDecisionResponse* response,
                                              std::string* error) {
  if (response == nullptr) {
    if (error != nullptr) {
      *error = "null ExternalDecisionResponse output";
    }
    return false;
  }
  if (payload_bytes.empty()) {
    if (error != nullptr) {
      *error = "empty ExternalDecisionResponse flatbuffers payload";
    }
    return false;
  }

  const std::uint8_t* data = reinterpret_cast<const std::uint8_t*>(payload_bytes.data());
  flatbuffers::Verifier verifier(data, payload_bytes.size());
  const fb::ExternalDecisionResponse* root =
      flatbuffers::GetRoot<fb::ExternalDecisionResponse>(data);
  if (root == nullptr || !root->Verify(verifier)) {
    if (error != nullptr) {
      *error = "invalid ExternalDecisionResponse flatbuffers payload";
    }
    return false;
  }

  *response = DecodeExternalDecisionResponse(root);
  return true;
}

std::string EncodePrepareCycleInputFlatbuffer(const ArPrepareCycleInput& input) {
  flatbuffers::FlatBufferBuilder builder;
  const auto root = EncodePrepareInput(&builder, input);
  builder.Finish(root);
  return oneq::common::replay::CopyFinishedFlatbuffer(builder);
}

bool DecodePrepareCycleInputFlatbuffer(const std::string& payload_bytes, ArPrepareCycleInput* input,
                                       std::string* error) {
  if (input == nullptr) {
    if (error != nullptr) {
      *error = "null ArPrepareCycleInput output";
    }
    return false;
  }
  const auto* root =
      TryGetReplayRoot<fb::ArPrepareCycleInputV2>(payload_bytes, "ArPrepareCycleInputV2", error);
  if (root == nullptr) {
    return false;
  }
  *input = DecodePrepareInput(root);
  return true;
}

std::string EncodePrepareReplayRecordFlatbuffer(const ArPrepareReplayRecord& record) {
  flatbuffers::FlatBufferBuilder builder;
  const auto root =
      fb::CreateArPrepareReplayRecordV2(builder, EncodePrepareResult(&builder, record.result),
                                        EncodeSessionReplayState(&builder, record.session_state));
  builder.Finish(root);
  return oneq::common::replay::CopyFinishedFlatbuffer(builder);
}

bool DecodePrepareReplayRecordFlatbuffer(const std::string& payload_bytes,
                                         ArPrepareReplayRecord* record, std::string* error) {
  if (record == nullptr) {
    if (error != nullptr) {
      *error = "null ArPrepareReplayRecord output";
    }
    return false;
  }
  const auto* root = TryGetReplayRoot<fb::ArPrepareReplayRecordV2>(
      payload_bytes, "ArPrepareReplayRecordV2", error);
  if (root == nullptr) {
    return false;
  }
  record->result = DecodePrepareResult(root->result());
  record->session_state = DecodeSessionReplayState(root->session_state());
  return true;
}

std::string EncodeCompleteReplayOperationInputFlatbuffer(
    const ArCompleteReplayOperationInput& operation) {
  flatbuffers::FlatBufferBuilder builder;
  std::vector<flatbuffers::Offset<fb::ArSceneTarget>> targets;
  targets.reserve(operation.input.targets.size());
  for (const auto& target : operation.input.targets) {
    targets.push_back(EncodeSceneTarget(&builder, target));
  }
  const auto root = fb::CreateArCompleteReplayOperationInputV2(
      builder, EncodePreparedToken(&builder, operation.token),
      EncodeRfV2Scene(&builder, operation.input.rf_scene), builder.CreateVector(targets),
      EncodeCycleAtmosphericObservation(&builder, operation.input.atmospheric_observation),
      EncodeCycleAtmosphericContext(&builder, operation.input.atmospheric_context),
      EncodeCycleSurfaceObservation(&builder, operation.input.surface_observation));
  builder.Finish(root);
  return oneq::common::replay::CopyFinishedFlatbuffer(builder);
}

bool DecodeCompleteReplayOperationInputFlatbuffer(const std::string& payload_bytes,
                                                  ArCompleteReplayOperationInput* operation,
                                                  std::string* error) {
  if (operation == nullptr) {
    if (error != nullptr) {
      *error = "null ArCompleteReplayOperationInput output";
    }
    return false;
  }
  const auto* root = TryGetReplayRoot<fb::ArCompleteReplayOperationInputV2>(
      payload_bytes, "ArCompleteReplayOperationInputV2", error);
  if (root == nullptr) {
    return false;
  }
  ArCompleteReplayOperationInput decoded;
  decoded.token = DecodePreparedToken(root->token());
  decoded.input.rf_scene = DecodeRfV2Scene(root->rf_scene());
  if (root->targets() != nullptr) {
    decoded.input.targets.reserve(root->targets()->size());
    for (const fb::ArSceneTarget* target : *root->targets()) {
      decoded.input.targets.push_back(DecodeSceneTarget(target));
    }
  }
  decoded.input.atmospheric_observation =
      DecodeCycleAtmosphericObservation(root->atmospheric_observation());
  decoded.input.atmospheric_context = DecodeCycleAtmosphericContext(root->atmospheric_context());
  decoded.input.surface_observation = DecodeCycleSurfaceObservation(root->surface_observation());
  *operation = decoded;
  return true;
}

std::string EncodeCompleteReplayRecordFlatbuffer(const ArCompleteReplayRecord& record) {
  flatbuffers::FlatBufferBuilder builder;
  std::vector<flatbuffers::Offset<fb::ArInterferenceObservation>> observations;
  observations.reserve(record.result.interference_observations.size());
  for (const auto& observation : record.result.interference_observations) {
    observations.push_back(EncodeArInterferenceObservation(&builder, observation));
  }
  const auto result = fb::CreateArCompleteCycleResultV2(
      builder, static_cast<int>(record.result.status), record.result.world_cycle_index,
      EncodeTrackOutputFrame(&builder, record.result.track_output_frame),
      builder.CreateVector(observations), static_cast<int>(record.result.receiver_impairment),
      record.result.has_decision_observation,
      EncodeDecisionObservation(&builder, record.result.decision_observation));
  const auto root = fb::CreateArCompleteReplayRecordV2(
      builder, result, EncodeSessionReplayState(&builder, record.session_state));
  builder.Finish(root);
  return oneq::common::replay::CopyFinishedFlatbuffer(builder);
}

bool DecodeCompleteReplayRecordFlatbuffer(const std::string& payload_bytes,
                                          ArCompleteReplayRecord* record, std::string* error) {
  if (record == nullptr) {
    if (error != nullptr) {
      *error = "null ArCompleteReplayRecord output";
    }
    return false;
  }
  const auto* root = TryGetReplayRoot<fb::ArCompleteReplayRecordV2>(
      payload_bytes, "ArCompleteReplayRecordV2", error);
  if (root == nullptr || root->result() == nullptr) {
    return false;
  }
  ArCompleteReplayRecord decoded;
  decoded.result.status = static_cast<ArCompleteCycleStatus>(root->result()->status());
  decoded.result.world_cycle_index = root->result()->world_cycle_index();
  decoded.result.track_output_frame = DecodeTrackOutputFrame(root->result()->track_output_frame());
  if (root->result()->interference_observations() != nullptr) {
    decoded.result.interference_observations.reserve(
        root->result()->interference_observations()->size());
    for (const fb::ArInterferenceObservation* observation :
         *root->result()->interference_observations()) {
      decoded.result.interference_observations.push_back(
          DecodeArInterferenceObservation(observation));
    }
  }
  decoded.result.receiver_impairment =
      static_cast<ArReceiverImpairment>(root->result()->receiver_impairment());
  decoded.result.has_decision_observation = root->result()->has_decision_observation();
  decoded.result.decision_observation =
      DecodeDecisionObservation(root->result()->decision_observation());
  decoded.session_state = DecodeSessionReplayState(root->session_state());
  *record = decoded;
  return true;
}

std::string EncodeAbandonReplayOperationInputFlatbuffer(
    const ArAbandonReplayOperationInput& operation) {
  flatbuffers::FlatBufferBuilder builder;
  const auto root = fb::CreateArAbandonReplayOperationInputV2(
      builder, EncodePreparedToken(&builder, operation.token));
  builder.Finish(root);
  return oneq::common::replay::CopyFinishedFlatbuffer(builder);
}

bool DecodeAbandonReplayOperationInputFlatbuffer(const std::string& payload_bytes,
                                                 ArAbandonReplayOperationInput* operation,
                                                 std::string* error) {
  if (operation == nullptr) {
    if (error != nullptr) {
      *error = "null ArAbandonReplayOperationInput output";
    }
    return false;
  }
  const auto* root = TryGetReplayRoot<fb::ArAbandonReplayOperationInputV2>(
      payload_bytes, "ArAbandonReplayOperationInputV2", error);
  if (root == nullptr) {
    return false;
  }
  operation->token = DecodePreparedToken(root->token());
  return true;
}

std::string EncodeAbandonReplayRecordFlatbuffer(const ArAbandonReplayRecord& record) {
  flatbuffers::FlatBufferBuilder builder;
  const auto root =
      fb::CreateArAbandonReplayRecordV2(builder, static_cast<int>(record.status),
                                        EncodeSessionReplayState(&builder, record.session_state));
  builder.Finish(root);
  return oneq::common::replay::CopyFinishedFlatbuffer(builder);
}

bool DecodeAbandonReplayRecordFlatbuffer(const std::string& payload_bytes,
                                         ArAbandonReplayRecord* record, std::string* error) {
  if (record == nullptr) {
    if (error != nullptr) {
      *error = "null ArAbandonReplayRecord output";
    }
    return false;
  }
  const auto* root = TryGetReplayRoot<fb::ArAbandonReplayRecordV2>(
      payload_bytes, "ArAbandonReplayRecordV2", error);
  if (root == nullptr) {
    return false;
  }
  record->status = static_cast<ArAbandonCycleStatus>(root->status());
  record->session_state = DecodeSessionReplayState(root->session_state());
  return true;
}

std::string EncodeRuntimeConfigAttemptFlatbuffer(const config::ArRuntimeConfigPatch& patch,
                                                 bool accepted) {
  const std::string patch_payload = EncodeRuntimeConfigPatchFlatbuffer(patch);
  flatbuffers::FlatBufferBuilder builder;
  const auto root = fb::CreateArRuntimeConfigAttemptV2(
      builder,
      builder.CreateVector(reinterpret_cast<const std::uint8_t*>(patch_payload.data()),
                           patch_payload.size()),
      accepted);
  builder.Finish(root);
  return oneq::common::replay::CopyFinishedFlatbuffer(builder);
}

bool DecodeRuntimeConfigAttemptFlatbuffer(const std::string& payload_bytes,
                                          config::ArRuntimeConfigPatch* patch, bool* accepted,
                                          std::string* error) {
  if (patch == nullptr || accepted == nullptr) {
    if (error != nullptr) {
      *error = "null ArRuntimeConfigAttempt output";
    }
    return false;
  }
  const auto* root = TryGetReplayRoot<fb::ArRuntimeConfigAttemptV2>(
      payload_bytes, "ArRuntimeConfigAttemptV2", error);
  if (root == nullptr || root->patch_payload() == nullptr) {
    return false;
  }
  const std::string patch_payload(reinterpret_cast<const char*>(root->patch_payload()->Data()),
                                  root->patch_payload()->size());
  if (!DecodeRuntimeConfigPatchFlatbuffer(patch_payload, patch, error)) {
    return false;
  }
  *accepted = root->accepted();
  return true;
}

std::string EncodeExternalDecisionAttemptFlatbuffer(
    const session::ExternalDecisionResponse& response,
    session::ExternalDecisionSubmitStatus status) {
  const std::string response_payload = EncodeExternalDecisionResponseFlatbuffer(response);
  flatbuffers::FlatBufferBuilder builder;
  const auto root = fb::CreateArExternalDecisionAttemptV2(
      builder,
      builder.CreateVector(reinterpret_cast<const std::uint8_t*>(response_payload.data()),
                           response_payload.size()),
      static_cast<int>(status));
  builder.Finish(root);
  return oneq::common::replay::CopyFinishedFlatbuffer(builder);
}

bool DecodeExternalDecisionAttemptFlatbuffer(const std::string& payload_bytes,
                                             session::ExternalDecisionResponse* response,
                                             session::ExternalDecisionSubmitStatus* status,
                                             std::string* error) {
  if (response == nullptr || status == nullptr) {
    if (error != nullptr) {
      *error = "null ArExternalDecisionAttempt output";
    }
    return false;
  }
  const auto* root = TryGetReplayRoot<fb::ArExternalDecisionAttemptV2>(
      payload_bytes, "ArExternalDecisionAttemptV2", error);
  if (root == nullptr || root->response_payload() == nullptr) {
    return false;
  }
  const std::string response_payload(
      reinterpret_cast<const char*>(root->response_payload()->Data()),
      root->response_payload()->size());
  if (!DecodeExternalDecisionResponseFlatbuffer(response_payload, response, error)) {
    return false;
  }
  *status = static_cast<session::ExternalDecisionSubmitStatus>(root->status());
  return true;
}

std::string EncodeSessionConfigFlatbuffer(const config::ArSessionConfig& config) {
  flatbuffers::FlatBufferBuilder builder;
  const flatbuffers::Offset<session_fb::ArSessionConfig> root = session_fb::CreateArSessionConfig(
      builder, EncodeSessionDetectionConfig(&builder, config.hardware),
      EncodeSessionOrientation(&builder, config.mission.orientation),
      EncodeSessionPolicyConfig(&builder, config.policy),
      static_cast<int>(config.environment.jamming_sensitivity_profile),
      EncodeEnvironmentDefaultConfig(&builder, config.environment), config.mission.power_on);
  builder.Finish(root, session_fb::ArSessionConfigIdentifier());

  return oneq::common::replay::CopyFinishedFlatbuffer(builder);
}

bool DecodeSessionConfigFlatbuffer(const std::string& payload_bytes,
                                   config::ArSessionConfig* config, std::string* error) {
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

  return oneq::common::replay::CopyFinishedFlatbuffer(builder);
}

bool DecodeRuntimeConfigPatchFlatbuffer(const std::string& payload_bytes,
                                        config::ArRuntimeConfigPatch* patch, std::string* error) {
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
  patch->environment = DecodeSessionEnvironmentRuntimeConfigPatch(root->environment());
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

  return oneq::common::replay::CopyFinishedFlatbuffer(builder);
}

bool DecodeFailureMarkerFlatbuffer(const std::string& payload_bytes,
                                   oneq::replay::ReplayTraceFailure* failure, std::string* error) {
  return oneq::common::replay::DecodeFailureMarkerPayload<fb::FailureMarker>(payload_bytes, failure,
                                                                             error);
}

}  // namespace session
}  // namespace airborne_radar
