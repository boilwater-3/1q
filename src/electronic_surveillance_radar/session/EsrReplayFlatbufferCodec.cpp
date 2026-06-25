#include "EsrReplayFlatbufferCodec.h"

#include <cstdint>
#include <string>

#include "flatbuffers/flatbuffers.h"
#include "generated/esr_replay_generated.h"
#include "generated/esr_session_replay_generated.h"

namespace electronic_surveillance_radar {
namespace session {
namespace {

esr::replay::Vec3 ToV(const oneq::foundation::Vector3f& v) {
  return {static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)};
}
esr::replay::EulerDeg ToE(const oneq::foundation::EulerAnglesDeg& e) {
  return {static_cast<float>(e.yaw_deg), static_cast<float>(e.pitch_deg),
          static_cast<float>(e.roll_deg)};
}

flatbuffers::Offset<esr::replay::PoseState> BuildPose(flatbuffers::FlatBufferBuilder& b,
                                                      const oneq::foundation::PoseState& v) {
  esr::replay::PoseStateBuilder pb(b);
  auto pos = ToV(v.position_m);
  pb.add_position_m(&pos);
  auto vel = ToV(v.velocity_mps);
  pb.add_velocity_mps(&vel);
  auto att = ToE(v.attitude_deg);
  pb.add_attitude_deg(&att);
  return pb.Finish();
}

oneq::foundation::PoseState FromPose(const esr::replay::PoseState* fb) {
  oneq::foundation::PoseState out{};
  if (!fb) {
    return out;
  }
  if (fb->position_m()) {
    out.position_m.x = fb->position_m()->x();
    out.position_m.y = fb->position_m()->y();
    out.position_m.z = fb->position_m()->z();
  }
  if (fb->velocity_mps()) {
    out.velocity_mps.x = fb->velocity_mps()->x();
    out.velocity_mps.y = fb->velocity_mps()->y();
    out.velocity_mps.z = fb->velocity_mps()->z();
  }
  if (fb->attitude_deg()) {
    out.attitude_deg.yaw_deg = fb->attitude_deg()->yaw_deg();
    out.attitude_deg.pitch_deg = fb->attitude_deg()->pitch_deg();
    out.attitude_deg.roll_deg = fb->attitude_deg()->roll_deg();
  }
  return out;
}

}  // namespace

std::string EncodeEsrCycleInput(const EsrCycleInput& v) {
  flatbuffers::FlatBufferBuilder fbb(1024);

  std::vector<flatbuffers::Offset<esr::replay::SceneEmitter>> emitters;
  for (const auto& e : v.scene) {
    auto pose = BuildPose(fbb, e.pose);
    auto emitter_name = fbb.CreateString(e.emitter_name);
    auto beam = esr::replay::CreateEmitterBeamState(
        fbb, e.beam_state.center_az_deg, e.beam_state.center_el_deg, e.beam_state.az_beamwidth_deg,
        e.beam_state.el_beamwidth_deg, e.beam_state.beam_state_valid);
    esr::replay::SceneEmitterBuilder eb(fbb);
    eb.add_emitter_id(e.emitter_id);
    eb.add_emitter_name(emitter_name);
    eb.add_pose(pose);
    eb.add_beam_state(beam);
    eb.add_carrier_hz(e.carrier_hz);
    eb.add_bandwidth_hz(e.bandwidth_hz);
    eb.add_tx_power_w(e.tx_power_w);
    eb.add_pulse_width_s(e.pulse_width_s);
    eb.add_pri_s(e.pri_s);
    eb.add_is_emitting(e.is_emitting);
    emitters.push_back(eb.Finish());
  }

  const auto& env = v.environment;
  std::vector<flatbuffers::Offset<esr::replay::EsrJammerSource>> jammers;
  for (const auto& j : env.jammer_sources) {
    jammers.push_back(esr::replay::CreateEsrJammerSource(
        fbb, static_cast<int32_t>(j.technique), j.active, j.center_hz, j.bandwidth_hz, j.power_w,
        j.deception_risk, j.confidence));
  }
  // EsrAtmosphericObservation is a FlatBuffers table, use Create helper
  auto atm = esr::replay::CreateEsrAtmosphericObservation(
      fbb, env.atmospheric_observation.relative_humidity_ratio,
      env.atmospheric_observation.precipitation_rate_mmph,
      env.atmospheric_observation.visibility_km);
  auto env_fb = esr::replay::CreateEsrEnvironmentInput(
      fbb, static_cast<int32_t>(env.propagation_profile), static_cast<int32_t>(env.clutter_density),
      env.spectrum_occupancy_ratio, atm, fbb.CreateVector(jammers));

  auto platform = BuildPose(fbb, v.platform_pose);
  auto emitters_vec = fbb.CreateVector(emitters);
  esr::replay::EsrCycleInputBuilder b(fbb);
  b.add_cycle_index(v.cycle_index);
  b.add_dt_sec(v.dt_sec);
  b.add_platform_pose(platform);
  b.add_scene_emitters(emitters_vec);
  b.add_environment(env_fb);
  b.add_platform_altitude_m(v.platform_altitude_m);
  fbb.Finish(b.Finish());
  return {reinterpret_cast<const char*>(fbb.GetBufferPointer()), fbb.GetSize()};
}

bool DecodeEsrCycleInput(const std::string& bytes, EsrCycleInput* out) {
  flatbuffers::Verifier ver(reinterpret_cast<const uint8_t*>(bytes.data()), bytes.size());
  if (!ver.VerifyBuffer<esr::replay::EsrCycleInput>()) {
    return false;
  }
  const auto* fb = flatbuffers::GetRoot<esr::replay::EsrCycleInput>(bytes.data());
  out->cycle_index = fb->cycle_index();
  out->dt_sec = fb->dt_sec();
  out->platform_altitude_m = fb->platform_altitude_m();
  out->platform_pose = FromPose(fb->platform_pose());
  out->scene.clear();
  if (fb->scene_emitters()) {
    for (const auto* e : *fb->scene_emitters()) {
      session::EsrSceneEmitter ts{};
      ts.emitter_id = e->emitter_id();
      ts.emitter_name = e->emitter_name() ? e->emitter_name()->str() : std::string();
      ts.pose = FromPose(e->pose());
      ts.carrier_hz = e->carrier_hz();
      ts.bandwidth_hz = e->bandwidth_hz();
      ts.tx_power_w = e->tx_power_w();
      ts.pulse_width_s = e->pulse_width_s();
      ts.pri_s = e->pri_s();
      ts.is_emitting = e->is_emitting();
      if (e->beam_state()) {
        ts.beam_state.center_az_deg = e->beam_state()->center_az_deg();
        ts.beam_state.center_el_deg = e->beam_state()->center_el_deg();
        ts.beam_state.az_beamwidth_deg = e->beam_state()->az_beamwidth_deg();
        ts.beam_state.el_beamwidth_deg = e->beam_state()->el_beamwidth_deg();
        ts.beam_state.beam_state_valid = e->beam_state()->beam_state_valid();
      }
      out->scene.push_back(ts);
    }
  }
  out->environment = {};
  if (fb->environment()) {
    const auto* e = fb->environment();
    out->environment.propagation_profile =
        static_cast<environment::EsrPropagationEnvironmentProfile>(e->propagation_profile());
    out->environment.clutter_density =
        static_cast<environment::EsrClutterDensityLevel>(e->clutter_density());
    out->environment.spectrum_occupancy_ratio = e->spectrum_occupancy_ratio();
    if (e->atmospheric_observation()) {
      out->environment.atmospheric_observation.relative_humidity_ratio =
          e->atmospheric_observation()->relative_humidity_ratio();
      out->environment.atmospheric_observation.precipitation_rate_mmph =
          e->atmospheric_observation()->precipitation_rate_mmph();
      out->environment.atmospheric_observation.visibility_km =
          e->atmospheric_observation()->visibility_km();
    }
    if (e->jammer_sources()) {
      for (const auto* j : *e->jammer_sources()) {
        environment::EsrJammerSource js{};
        js.technique = static_cast<environment::EsrJammingTechnique>(j->technique());
        js.active = j->active();
        js.center_hz = j->center_hz();
        js.bandwidth_hz = j->bandwidth_hz();
        js.power_w = j->power_w();
        js.deception_risk = j->deception_risk();
        js.confidence = j->confidence();
        out->environment.jammer_sources.push_back(js);
      }
    }
  }
  return true;
}

namespace {

flatbuffers::Offset<esr::replay::EsrOutputFrame> CreateEsrOutputFrameTable(
    flatbuffers::FlatBufferBuilder& fbb, const session::EsrOutputFrame& v) {
  // observation output
  std::vector<flatbuffers::Offset<esr::replay::EmitterObservation>> obs_vec;
  for (const auto& o : v.observation_output.observations) {
    obs_vec.push_back(esr::replay::CreateEmitterObservation(
        fbb, o.observation_id, o.timestamp_s, o.aoa_az_deg, o.aoa_el_deg, o.rf_hz, o.pulse_width_s,
        o.amplitude_db, o.snr_db, static_cast<int32_t>(o.quality), o.is_jammed));
  }
  auto obs_out = esr::replay::CreateObservationOutput(
      fbb, static_cast<std::uint32_t>(v.observation_output.raw_observation_count),
      static_cast<std::uint32_t>(v.observation_output.cluster_count), fbb.CreateVector(obs_vec));

  // emitter output
  std::vector<flatbuffers::Offset<esr::replay::EmitterHypothesis>> hyp_vec;
  for (const auto& h : v.emitter_output.hypotheses) {
    std::vector<flatbuffers::Offset<flatbuffers::String>> cls_vec;
    for (const auto& c : h.candidate_classes) {
      cls_vec.push_back(fbb.CreateString(c));
    }
    hyp_vec.push_back(esr::replay::CreateEmitterHypothesis(
        fbb, h.hypothesis_id, fbb.CreateVector(cls_vec), static_cast<int32_t>(h.mode),
        static_cast<int32_t>(h.threat_level), h.bearing_az_deg, h.bearing_el_deg, h.bearing_std_deg,
        h.confidence, h.last_seen_cycle));
  }
  auto em_out = esr::replay::CreateEmitterOutput(fbb, fbb.CreateVector(hyp_vec));

  // truth output
  std::vector<flatbuffers::Offset<esr::replay::TruthAssociationRecord>> ta_vec;
  for (const auto& a : v.truth_evaluation_output.associations) {
    ta_vec.push_back(esr::replay::CreateTruthAssociationRecord(
        fbb, a.observation_id, a.truth_emitter_id, a.matched, static_cast<float>(a.confidence)));
  }
  auto truth_out = esr::replay::CreateTruthEvaluationOutput(fbb, fbb.CreateVector(ta_vec));

  return esr::replay::CreateEsrOutputFrame(fbb, static_cast<std::uint32_t>(v.cycle_index),
                                           static_cast<std::uint32_t>(v.batch_id), obs_out, em_out,
                                           truth_out);
}

void PopulateOutputFrame(const esr::replay::EsrOutputFrame* fb, session::EsrOutputFrame* out) {
  if (!fb) {
    return;
  }
  out->cycle_index = fb->cycle_index();
  out->batch_id = fb->batch_id();
  if (fb->observation_output()) {
    const auto* o = fb->observation_output();
    out->observation_output.raw_observation_count = o->raw_observation_count();
    out->observation_output.cluster_count = o->cluster_count();
    if (o->observations()) {
      for (const auto* obs : *o->observations()) {
        model::EmitterObservation rec{};
        rec.observation_id = obs->observation_id();
        rec.timestamp_s = obs->timestamp_s();
        rec.aoa_az_deg = obs->aoa_az_deg();
        rec.aoa_el_deg = obs->aoa_el_deg();
        rec.rf_hz = obs->rf_hz();
        rec.pulse_width_s = obs->pulse_width_s();
        rec.amplitude_db = obs->amplitude_db();
        rec.snr_db = obs->snr_db();
        rec.quality = static_cast<model::EsrObservationQuality>(obs->quality());
        rec.is_jammed = obs->is_jammed();
        out->observation_output.observations.push_back(rec);
      }
    }
  }
  if (fb->emitter_output()) {
    const auto* e = fb->emitter_output();
    if (e->hypotheses()) {
      for (const auto* h : *e->hypotheses()) {
        model::EmitterHypothesis hyp{};
        hyp.hypothesis_id = h->hypothesis_id();
        hyp.mode = static_cast<model::EsrEmitterMode>(h->mode());
        hyp.threat_level = static_cast<model::EsrThreatLevel>(h->threat_level());
        hyp.bearing_az_deg = h->bearing_az_deg();
        hyp.bearing_el_deg = h->bearing_el_deg();
        hyp.bearing_std_deg = h->bearing_std_deg();
        hyp.confidence = h->confidence();
        hyp.last_seen_cycle = h->last_seen_cycle();
        if (h->candidate_classes()) {
          for (const auto* c : *h->candidate_classes()) {
            if (c) {
              hyp.candidate_classes.push_back(c->str());
            }
          }
        }
        out->emitter_output.hypotheses.push_back(hyp);
      }
    }
  }
  if (fb->truth_evaluation_output()) {
    const auto* t = fb->truth_evaluation_output();
    if (t->associations()) {
      for (const auto* a : *t->associations()) {
        extension::TruthAssociationRecord rec{};
        rec.observation_id = a->observation_id();
        rec.truth_emitter_id = a->truth_emitter_id();
        rec.matched = a->matched();
        rec.confidence = a->confidence();
        out->truth_evaluation_output.associations.push_back(rec);
      }
    }
  }
}

}  // namespace

std::string EncodeEsrOutputFrame(const session::EsrOutputFrame& v) {
  flatbuffers::FlatBufferBuilder fbb(512);
  fbb.Finish(CreateEsrOutputFrameTable(fbb, v));
  return {reinterpret_cast<const char*>(fbb.GetBufferPointer()), fbb.GetSize()};
}

bool DecodeEsrOutputFrame(const std::string& bytes, session::EsrOutputFrame* out) {
  flatbuffers::Verifier ver(reinterpret_cast<const uint8_t*>(bytes.data()), bytes.size());
  if (!ver.VerifyBuffer<esr::replay::EsrOutputFrame>()) {
    return false;
  }
  PopulateOutputFrame(flatbuffers::GetRoot<esr::replay::EsrOutputFrame>(bytes.data()), out);
  return true;
}

std::string EncodeEsrCycleResult(const EsrCycleResult& v) {
  flatbuffers::FlatBufferBuilder fbb(512);
  auto frame = CreateEsrOutputFrameTable(fbb, v.output_frame);
  std::vector<flatbuffers::Offset<esr::replay::ValidationIssue>> issues;
  for (const auto& i : v.validation_issues) {
    const std::size_t encoded_entity_index = i.location.kind == ValidationLocationKind::kSceneEntity
                                                 ? i.location.entity_index
                                                 : static_cast<std::size_t>(-1);
    issues.push_back(esr::replay::CreateValidationIssue(
        fbb, static_cast<int32_t>(i.severity), static_cast<int32_t>(i.code),
        static_cast<int32_t>(encoded_entity_index), fbb.CreateString(i.field),
        fbb.CreateString(i.message)));
  }
  fbb.Finish(esr::replay::CreateEsrCycleResult(
      fbb, v.input_cycle_index, frame, fbb.CreateVector(issues), v.has_validation_error,
      v.executed_this_cycle, v.reused_previous_output, static_cast<int32_t>(v.abort_reason)));
  return {reinterpret_cast<const char*>(fbb.GetBufferPointer()), fbb.GetSize()};
}

bool DecodeEsrCycleResult(const std::string& bytes, EsrCycleResult* out) {
  flatbuffers::Verifier ver(reinterpret_cast<const uint8_t*>(bytes.data()), bytes.size());
  if (!ver.VerifyBuffer<esr::replay::EsrCycleResult>()) {
    return false;
  }
  const auto* fb = flatbuffers::GetRoot<esr::replay::EsrCycleResult>(bytes.data());
  out->input_cycle_index = fb->input_cycle_index();
  PopulateOutputFrame(fb->output_frame(), &out->output_frame);
  out->has_validation_error = fb->has_validation_error();
  out->executed_this_cycle = fb->executed_this_cycle();
  out->reused_previous_output = fb->reused_previous_output();
  out->abort_reason = static_cast<extension::EsrPipelineAbortReason>(fb->abort_reason());
  out->validation_issues.clear();
  if (fb->validation_issues()) {
    for (const auto* i : *fb->validation_issues()) {
      ValidationIssue iss{};
      iss.severity = static_cast<ValidationSeverity>(i->severity());
      iss.code = static_cast<ValidationCode>(i->code());
      iss.location.kind = ValidationLocationKind::kSceneEntity;
      iss.location.entity_index = static_cast<std::size_t>(i->emitter_index());
      if (i->emitter_index() < 0) {
        iss.location.kind = ValidationLocationKind::kGlobal;
        iss.location.entity_index = static_cast<std::size_t>(-1);
      }
      if (i->message()) {
        iss.message = i->message()->str();
      }
      if (i->field()) {
        iss.field = i->field()->str();
      }
      out->validation_issues.push_back(iss);
    }
  }
  return true;
}

std::string EncodeEsrSessionConfig(const config::EsrSessionConfig& v) {
  flatbuffers::FlatBufferBuilder fbb(512);
  auto hw = esr::replay::CreateEsrHardwareConfig(
      fbb, v.hardware.receiver_band_lower_hz, v.hardware.receiver_band_upper_hz,
      v.hardware.receiver_sensitivity_w, v.hardware.integrated_receive_loss_db,
      v.hardware.beam_az_width_deg, v.hardware.beam_el_width_deg, v.hardware.az_scan_range_deg,
      v.hardware.el_scan_range_deg, v.hardware.antenna_mount_az_deg,
      v.hardware.antenna_mount_el_deg);
  const auto& sc = v.mission.scan;
  auto scan = esr::replay::CreateEsrScanConfig(
      fbb, sc.scan_center_az_deg, sc.scan_center_el_deg, sc.scan_rate_hz,
      static_cast<int32_t>(sc.scan_start_position), static_cast<int32_t>(sc.scan_sequence),
      sc.use_explicit_scan_bounds, sc.scan_start_az_deg, sc.scan_end_az_deg, sc.scan_start_el_deg,
      sc.scan_end_el_deg);
  auto mission = esr::replay::CreateEsrMissionConfig(
      fbb, v.mission.power_on, static_cast<int32_t>(v.mission.work_mode), scan);
  // detection_profile and use_profile_defaults retired in Phase 2; pass defaults for schema compat
  auto policy = esr::replay::CreateEsrPolicyConfig(
      fbb, 0, false, v.policy.detection.minimum_snr_db, v.policy.detection.pfa,
      v.policy.detection.pulse_count, v.policy.detection.threshold_scale,
      v.policy.detection.enable_statistical_detection);
  const auto& es = v.environment.scenario_config;
  auto ap = esr::replay::CreateEsrAtmosphericPhysicsConfig(
      fbb, es.atmospheric_physics.enable_physical_model, es.atmospheric_physics.pressure_hpa,
      es.atmospheric_physics.temperature_k, es.atmospheric_physics.relative_humidity);
  auto ac = esr::replay::CreateEsrAtmosphericDerivedContext(
      fbb, es.atmospheric_context.has_k_factor, es.atmospheric_context.k_factor,
      es.atmospheric_context.has_day_of_year, es.atmospheric_context.day_of_year,
      es.atmospheric_context.solar_flux_f107a, es.atmospheric_context.solar_flux_f107,
      es.atmospheric_context.geomagnetic_ap);
  auto env = esr::replay::CreateEsrEnvironmentConfig(fbb, static_cast<int32_t>(es.preset), ap, ac);
  fbb.Finish(esr::replay::CreateEsrSessionConfig(fbb, hw, mission, policy, env));
  return {reinterpret_cast<const char*>(fbb.GetBufferPointer()), fbb.GetSize()};
}

bool DecodeEsrSessionConfig(const std::string& bytes, config::EsrSessionConfig* out) {
  flatbuffers::Verifier ver(reinterpret_cast<const uint8_t*>(bytes.data()), bytes.size());
  if (!ver.VerifyBuffer<esr::replay::EsrSessionConfig>()) {
    return false;
  }
  const auto* fb = flatbuffers::GetRoot<esr::replay::EsrSessionConfig>(bytes.data());
  if (fb->hardware()) {
    const auto* h = fb->hardware();
    out->hardware.receiver_band_lower_hz = h->receiver_band_lower_hz();
    out->hardware.receiver_band_upper_hz = h->receiver_band_upper_hz();
    out->hardware.receiver_sensitivity_w = h->receiver_sensitivity_w();
    out->hardware.integrated_receive_loss_db = h->integrated_receive_loss_db();
    out->hardware.beam_az_width_deg = h->beam_az_width_deg();
    out->hardware.beam_el_width_deg = h->beam_el_width_deg();
    out->hardware.az_scan_range_deg = h->az_scan_range_deg();
    out->hardware.el_scan_range_deg = h->el_scan_range_deg();
    out->hardware.antenna_mount_az_deg = h->antenna_mount_az_deg();
    out->hardware.antenna_mount_el_deg = h->antenna_mount_el_deg();
  }
  if (fb->mission()) {
    const auto* m = fb->mission();
    out->mission.power_on = m->power_on();
    out->mission.work_mode = static_cast<config::EsrWorkMode>(m->work_mode());
    if (m->scan()) {
      const auto* s = m->scan();
      out->mission.scan.scan_center_az_deg = s->scan_center_az_deg();
      out->mission.scan.scan_center_el_deg = s->scan_center_el_deg();
      out->mission.scan.scan_rate_hz = s->scan_rate_hz();
      out->mission.scan.scan_start_position =
          static_cast<config::EsrScanStartPosition>(s->scan_start_position());
      out->mission.scan.scan_sequence = static_cast<config::EsrScanSequence>(s->scan_sequence());
      out->mission.scan.use_explicit_scan_bounds = s->use_explicit_scan_bounds();
      out->mission.scan.scan_start_az_deg = s->scan_start_az_deg();
      out->mission.scan.scan_end_az_deg = s->scan_end_az_deg();
      out->mission.scan.scan_start_el_deg = s->scan_start_el_deg();
      out->mission.scan.scan_end_el_deg = s->scan_end_el_deg();
    }
  }
  if (fb->policy()) {
    const auto* p = fb->policy();
    out->policy.detection.minimum_snr_db = p->minimum_snr_db();
    out->policy.detection.pfa = p->pfa();
    out->policy.detection.pulse_count = p->pulse_count();
    out->policy.detection.threshold_scale = p->threshold_scale();
    out->policy.detection.enable_statistical_detection = p->enable_statistical_detection();
  }
  if (fb->environment()) {
    const auto* e = fb->environment();
    out->environment.scenario_config.preset =
        static_cast<config::EsrEnvironmentPreset>(e->preset());
    if (e->atmospheric_physics()) {
      const auto* ap = e->atmospheric_physics();
      out->environment.scenario_config.atmospheric_physics.enable_physical_model =
          ap->enable_physical_model();
      out->environment.scenario_config.atmospheric_physics.pressure_hpa = ap->pressure_hpa();
      out->environment.scenario_config.atmospheric_physics.temperature_k = ap->temperature_k();
      out->environment.scenario_config.atmospheric_physics.relative_humidity =
          ap->relative_humidity();
    }
    if (e->atmospheric_context()) {
      const auto* ac = e->atmospheric_context();
      out->environment.scenario_config.atmospheric_context.has_k_factor = ac->has_k_factor();
      out->environment.scenario_config.atmospheric_context.k_factor = ac->k_factor();
      out->environment.scenario_config.atmospheric_context.has_day_of_year = ac->has_day_of_year();
      out->environment.scenario_config.atmospheric_context.day_of_year = ac->day_of_year();
      out->environment.scenario_config.atmospheric_context.solar_flux_f107a =
          ac->solar_flux_f107a();
      out->environment.scenario_config.atmospheric_context.solar_flux_f107 = ac->solar_flux_f107();
      out->environment.scenario_config.atmospheric_context.geomagnetic_ap = ac->geomagnetic_ap();
    }
  }
  return true;
}

std::string EncodeEsrRuntimeConfigPatch(const config::EsrRuntimeConfigPatch& v) {
  flatbuffers::FlatBufferBuilder fbb(512);
  const auto& ev = v.environment;
  auto ap = esr::replay::CreateEsrAtmosphericPhysicsConfig(
      fbb, ev.atmospheric_physics.enable_physical_model, ev.atmospheric_physics.pressure_hpa,
      ev.atmospheric_physics.temperature_k, ev.atmospheric_physics.relative_humidity);
  auto ac = esr::replay::CreateEsrAtmosphericDerivedContext(
      fbb, ev.atmospheric_context.has_k_factor, ev.atmospheric_context.k_factor,
      ev.atmospheric_context.has_day_of_year, ev.atmospheric_context.day_of_year,
      ev.atmospheric_context.solar_flux_f107a, ev.atmospheric_context.solar_flux_f107,
      ev.atmospheric_context.geomagnetic_ap);
  auto env_patch = esr::replay::CreateEsrEnvironmentRuntimeConfigPatch(
      fbb, ev.has_atmospheric_physics, ap, ev.has_atmospheric_context, ac);
  flatbuffers::Offset<esr::replay::EsrMissionConfig> mission;
  if (v.has_mission) {
    const auto& sc = v.mission.scan;
    auto scan = esr::replay::CreateEsrScanConfig(
        fbb, sc.scan_center_az_deg, sc.scan_center_el_deg, sc.scan_rate_hz,
        static_cast<int32_t>(sc.scan_start_position), static_cast<int32_t>(sc.scan_sequence),
        sc.use_explicit_scan_bounds, sc.scan_start_az_deg, sc.scan_end_az_deg, sc.scan_start_el_deg,
        sc.scan_end_el_deg);
    mission = esr::replay::CreateEsrMissionConfig(fbb, v.mission.power_on,
                                                  static_cast<int32_t>(v.mission.work_mode), scan);
  }
  flatbuffers::Offset<esr::replay::EsrPolicyConfig> policy;
  if (v.has_policy) {
    // detection_profile and use_profile_defaults retired in Phase 2; pass defaults for schema
    // compat
    policy = esr::replay::CreateEsrPolicyConfig(
        fbb, 0, false, v.policy.detection.minimum_snr_db, v.policy.detection.pfa,
        v.policy.detection.pulse_count, v.policy.detection.threshold_scale,
        v.policy.detection.enable_statistical_detection);
  }
  const auto& sb = v.explicit_scan_bounds;
  fbb.Finish(esr::replay::CreateEsrRuntimeConfigPatch(
      fbb, v.has_sensor_enabled, v.sensor_enabled, v.has_work_mode,
      static_cast<int32_t>(v.work_mode), v.has_scan_rate_hz, v.scan_rate_hz,
      v.has_scan_start_position, static_cast<int32_t>(v.scan_start_position), v.has_scan_sequence,
      static_cast<int32_t>(v.scan_sequence), v.has_scan_center_az_deg, v.scan_center_az_deg,
      v.has_scan_center_el_deg, v.scan_center_el_deg, v.has_explicit_scan_bounds, sb.enabled,
      v.has_explicit_scan_bounds, sb.scan_start_az_deg, v.has_explicit_scan_bounds,
      sb.scan_end_az_deg, v.has_explicit_scan_bounds, sb.scan_start_el_deg,
      v.has_explicit_scan_bounds, sb.scan_end_el_deg, v.has_mission, mission, v.has_policy, policy,
      v.has_environment, env_patch));
  return {reinterpret_cast<const char*>(fbb.GetBufferPointer()), fbb.GetSize()};
}

bool DecodeEsrRuntimeConfigPatch(const std::string& bytes, config::EsrRuntimeConfigPatch* out) {
  flatbuffers::Verifier ver(reinterpret_cast<const uint8_t*>(bytes.data()), bytes.size());
  if (!ver.VerifyBuffer<esr::replay::EsrRuntimeConfigPatch>()) {
    return false;
  }
  const auto* fb = flatbuffers::GetRoot<esr::replay::EsrRuntimeConfigPatch>(bytes.data());
  out->has_sensor_enabled = fb->has_sensor_enabled();
  out->sensor_enabled = fb->sensor_enabled();
  out->has_work_mode = fb->has_work_mode();
  out->work_mode = static_cast<config::EsrWorkMode>(fb->work_mode());
  out->has_scan_rate_hz = fb->has_scan_rate_hz();
  out->scan_rate_hz = fb->scan_rate_hz();
  out->has_scan_start_position = fb->has_scan_start_position();
  out->scan_start_position = static_cast<config::EsrScanStartPosition>(fb->scan_start_position());
  out->has_scan_sequence = fb->has_scan_sequence();
  out->scan_sequence = static_cast<config::EsrScanSequence>(fb->scan_sequence());
  out->has_scan_center_az_deg = fb->has_scan_center_az_deg();
  out->scan_center_az_deg = fb->scan_center_az_deg();
  out->has_scan_center_el_deg = fb->has_scan_center_el_deg();
  out->scan_center_el_deg = fb->scan_center_el_deg();
  out->has_explicit_scan_bounds = fb->has_use_explicit_scan_bounds();
  out->explicit_scan_bounds.enabled = fb->use_explicit_scan_bounds();
  out->explicit_scan_bounds.scan_start_az_deg = fb->scan_start_az_deg();
  out->explicit_scan_bounds.scan_end_az_deg = fb->scan_end_az_deg();
  out->explicit_scan_bounds.scan_start_el_deg = fb->scan_start_el_deg();
  out->explicit_scan_bounds.scan_end_el_deg = fb->scan_end_el_deg();
  out->has_mission = fb->has_mission();
  if (fb->mission()) {
    const auto* m = fb->mission();
    out->mission.power_on = m->power_on();
    out->mission.work_mode = static_cast<config::EsrWorkMode>(m->work_mode());
    if (m->scan()) {
      const auto* s = m->scan();
      out->mission.scan.scan_center_az_deg = s->scan_center_az_deg();
      out->mission.scan.scan_center_el_deg = s->scan_center_el_deg();
      out->mission.scan.scan_rate_hz = s->scan_rate_hz();
      out->mission.scan.scan_start_position =
          static_cast<config::EsrScanStartPosition>(s->scan_start_position());
      out->mission.scan.scan_sequence = static_cast<config::EsrScanSequence>(s->scan_sequence());
      out->mission.scan.use_explicit_scan_bounds = s->use_explicit_scan_bounds();
      out->mission.scan.scan_start_az_deg = s->scan_start_az_deg();
      out->mission.scan.scan_end_az_deg = s->scan_end_az_deg();
      out->mission.scan.scan_start_el_deg = s->scan_start_el_deg();
      out->mission.scan.scan_end_el_deg = s->scan_end_el_deg();
    }
  }
  out->has_policy = fb->has_policy();
  if (fb->policy()) {
    const auto* p = fb->policy();
    out->policy.detection.minimum_snr_db = p->minimum_snr_db();
    out->policy.detection.pfa = p->pfa();
    out->policy.detection.pulse_count = p->pulse_count();
    out->policy.detection.threshold_scale = p->threshold_scale();
    out->policy.detection.enable_statistical_detection = p->enable_statistical_detection();
  }
  out->has_environment = fb->has_environment();
  if (fb->environment()) {
    const auto* ec = fb->environment();
    out->environment.has_atmospheric_physics = ec->has_atmospheric_physics();
    if (ec->atmospheric_physics()) {
      out->environment.atmospheric_physics.enable_physical_model =
          ec->atmospheric_physics()->enable_physical_model();
      out->environment.atmospheric_physics.pressure_hpa =
          ec->atmospheric_physics()->pressure_hpa();
      out->environment.atmospheric_physics.temperature_k =
          ec->atmospheric_physics()->temperature_k();
      out->environment.atmospheric_physics.relative_humidity =
          ec->atmospheric_physics()->relative_humidity();
    }
    out->environment.has_atmospheric_context = ec->has_atmospheric_context();
    if (ec->atmospheric_context()) {
      out->environment.atmospheric_context.has_k_factor =
          ec->atmospheric_context()->has_k_factor();
      out->environment.atmospheric_context.k_factor =
          ec->atmospheric_context()->k_factor();
      out->environment.atmospheric_context.has_day_of_year =
          ec->atmospheric_context()->has_day_of_year();
      out->environment.atmospheric_context.day_of_year =
          ec->atmospheric_context()->day_of_year();
      out->environment.atmospheric_context.solar_flux_f107a =
          ec->atmospheric_context()->solar_flux_f107a();
      out->environment.atmospheric_context.solar_flux_f107 =
          ec->atmospheric_context()->solar_flux_f107();
      out->environment.atmospheric_context.geomagnetic_ap =
          ec->atmospheric_context()->geomagnetic_ap();
    }
  }
  return true;
}

std::string EncodeEsrFailureMarker(const oneq::replay::ReplayTraceFailure& failure) {
  flatbuffers::FlatBufferBuilder builder;
  const flatbuffers::Offset<esr::replay::FailureMarker> root =
      esr::replay::CreateFailureMarkerDirect(
          builder, failure.error_code.c_str(), failure.message.c_str(), failure.location.c_str(),
          failure.has_cycle_index, failure.cycle_index, failure.has_sim_time_sec,
          failure.sim_time_sec, failure.diagnostics_payload.c_str(), false, 0U);
  builder.Finish(root);

  const std::uint8_t* buffer = builder.GetBufferPointer();
  return std::string(reinterpret_cast<const char*>(buffer),
                     reinterpret_cast<const char*>(buffer) + builder.GetSize());
}

bool DecodeEsrFailureMarker(const std::string& payload_bytes,
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
  const esr::replay::FailureMarker* root = flatbuffers::GetRoot<esr::replay::FailureMarker>(data);
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
}  // namespace electronic_surveillance_radar
