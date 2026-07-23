#include "EsrReplayFlatbufferCodec.h"

#include <cstdint>
#include <string>

#include "flatbuffers/flatbuffers.h"
#include "common/replay/ReplayFlatbufferCodecSupport.h"
#include "electronic_surveillance_radar/session/generated/esr_replay_generated.h"
#include "electronic_surveillance_radar/session/generated/esr_session_replay_generated.h"

namespace electronic_surveillance_radar {
namespace session {
namespace {

esr::replay::Vec3 ToV(const oneq::coordinate::EcefPositionM& v) {
  return {v.x_m, v.y_m, v.z_m};
}
esr::replay::Vec3 ToV(const oneq::coordinate::EcefVelocityMps& v) {
  return {v.x_mps, v.y_mps, v.z_mps};
}
esr::replay::EulerDeg ToE(const oneq::coordinate::EulerAnglesDeg& e) {
  return {e.yaw_deg, e.pitch_deg, e.roll_deg};
}

flatbuffers::Offset<esr::replay::RfSceneFrame> BuildRfSceneFrame(
    flatbuffers::FlatBufferBuilder& fbb,
    const oneq::electromagnetics::RfEmissionFrame& frame) {
  std::vector<flatbuffers::Offset<esr::replay::RfSceneEmission>> emissions;
  emissions.reserve(frame.emissions.size());
  for (const oneq::electromagnetics::RfSceneEmission& emission : frame.emissions) {
    esr::replay::RfSceneDirection boresight(emission.antenna.boresight_ecef.x,
                                            emission.antenna.boresight_ecef.y,
                                            emission.antenna.boresight_ecef.z);
    const auto antenna = esr::replay::CreateRfSceneAntennaPattern(
        fbb, &boresight, emission.antenna.peak_gain_dbi,
        emission.antenna.half_power_beamwidth_deg, emission.antenna.sidelobe_level_db,
        emission.antenna.backlobe_level_db,
        emission.antenna.cross_polarization_isolation_db);
    const oneq::electromagnetics::RfWaveformSchedule& waveform = emission.waveform;
    const auto waveform_fb = esr::replay::CreateRfWaveformSchedule(
        fbb, static_cast<int32_t>(waveform.kind), waveform.activity_start_time_s,
        waveform.activity_duration_s, waveform.center_frequency_hz,
        waveform.occupied_bandwidth_hz, waveform.transmit_power_w, waveform.pulse_width_s,
        waveform.pulse_repetition_interval_s, waveform.first_pulse_time_s, waveform.pulse_count,
        waveform.pulse_jitter_fraction, waveform.timing_seed, waveform.timing_epoch,
        waveform.sweep_start_frequency_hz, waveform.sweep_stop_frequency_hz,
        waveform.sweep_period_s);
    const esr::replay::Vec3 position = ToV(emission.position_ecef_m);
    const esr::replay::Vec3 velocity = ToV(emission.velocity_ecef_mps);
    emissions.push_back(esr::replay::CreateRfSceneEmission(
        fbb, emission.identity.platform_id, emission.identity.equipment_id,
        emission.identity.emission_id, &position, &velocity, antenna,
        static_cast<int32_t>(emission.polarization), waveform_fb));
  }
  return esr::replay::CreateRfSceneFrame(
      fbb, frame.world_cycle_index, frame.window_start_time_s, frame.window_duration_s,
      fbb.CreateVector(emissions));
}

oneq::electromagnetics::RfEmissionFrame FromRfSceneFrame(
    const esr::replay::RfSceneFrame* fb) {
  oneq::electromagnetics::RfEmissionFrame frame;
  if (fb == nullptr) {
    return frame;
  }
  frame.world_cycle_index = fb->world_cycle_index();
  frame.window_start_time_s = fb->window_start_time_s();
  frame.window_duration_s = fb->window_duration_s();
  if (!fb->emissions()) {
    return frame;
  }
  frame.emissions.reserve(fb->emissions()->size());
  for (const esr::replay::RfSceneEmission* source : *fb->emissions()) {
    if (source == nullptr) {
      continue;
    }
    oneq::electromagnetics::RfSceneEmission emission;
    emission.identity.platform_id = source->platform_id();
    emission.identity.equipment_id = source->equipment_id();
    emission.identity.emission_id = source->emission_id();
    if (source->position_ecef_m()) {
      emission.position_ecef_m.x_m = source->position_ecef_m()->x();
      emission.position_ecef_m.y_m = source->position_ecef_m()->y();
      emission.position_ecef_m.z_m = source->position_ecef_m()->z();
    }
    if (source->velocity_ecef_mps()) {
      emission.velocity_ecef_mps.x_mps = source->velocity_ecef_mps()->x();
      emission.velocity_ecef_mps.y_mps = source->velocity_ecef_mps()->y();
      emission.velocity_ecef_mps.z_mps = source->velocity_ecef_mps()->z();
    }
    if (source->antenna()) {
      emission.antenna.peak_gain_dbi = source->antenna()->peak_gain_dbi();
      emission.antenna.half_power_beamwidth_deg =
          source->antenna()->half_power_beamwidth_deg();
      emission.antenna.sidelobe_level_db = source->antenna()->sidelobe_level_db();
      emission.antenna.backlobe_level_db = source->antenna()->backlobe_level_db();
      emission.antenna.cross_polarization_isolation_db =
          source->antenna()->cross_polarization_isolation_db();
      if (source->antenna()->boresight_ecef()) {
        emission.antenna.boresight_ecef.x = source->antenna()->boresight_ecef()->x();
        emission.antenna.boresight_ecef.y = source->antenna()->boresight_ecef()->y();
        emission.antenna.boresight_ecef.z = source->antenna()->boresight_ecef()->z();
      }
    }
    emission.polarization = static_cast<oneq::electromagnetics::RfScenePolarization>(
        source->polarization());
    if (source->waveform()) {
      const esr::replay::RfWaveformSchedule* waveform = source->waveform();
      emission.waveform.kind =
          static_cast<oneq::electromagnetics::RfSceneWaveformKind>(waveform->kind());
      emission.waveform.activity_start_time_s = waveform->activity_start_time_s();
      emission.waveform.activity_duration_s = waveform->activity_duration_s();
      emission.waveform.center_frequency_hz = waveform->center_frequency_hz();
      emission.waveform.occupied_bandwidth_hz = waveform->occupied_bandwidth_hz();
      emission.waveform.transmit_power_w = waveform->transmit_power_w();
      emission.waveform.pulse_width_s = waveform->pulse_width_s();
      emission.waveform.pulse_repetition_interval_s = waveform->pulse_repetition_interval_s();
      emission.waveform.first_pulse_time_s = waveform->first_pulse_time_s();
      emission.waveform.pulse_count = waveform->pulse_count();
      emission.waveform.pulse_jitter_fraction = waveform->pulse_jitter_fraction();
      emission.waveform.timing_seed = waveform->timing_seed();
      emission.waveform.timing_epoch = waveform->timing_epoch();
      emission.waveform.sweep_start_frequency_hz = waveform->sweep_start_frequency_hz();
      emission.waveform.sweep_stop_frequency_hz = waveform->sweep_stop_frequency_hz();
      emission.waveform.sweep_period_s = waveform->sweep_period_s();
    }
    frame.emissions.push_back(emission);
  }
  return frame;
}

}  // namespace

std::string EncodeEsrCycleInput(const EsrCycleInput& v) {
  flatbuffers::FlatBufferBuilder fbb(1024);

  const auto& env = v.environment;
  // EsrAtmosphericObservation is a FlatBuffers table, use Create helper
  auto atm = esr::replay::CreateEsrAtmosphericObservation(
      fbb, env.atmospheric_observation.relative_humidity_ratio,
      env.atmospheric_observation.precipitation_rate_mmph,
      env.atmospheric_observation.visibility_km);
  esr::replay::EsrEnvironmentInputBuilder env_builder(fbb);
  env_builder.add_propagation_profile(static_cast<int32_t>(env.propagation_profile));
  env_builder.add_clutter_density(static_cast<int32_t>(env.clutter_density));
  env_builder.add_spectrum_occupancy_ratio(env.spectrum_occupancy_ratio);
  env_builder.add_atmospheric_observation(atm);
  auto env_fb = env_builder.Finish();

  const flatbuffers::Offset<esr::replay::RfSceneFrame> interference =
      BuildRfSceneFrame(fbb, v.interference);
  esr::replay::EsrCycleInputBuilder b(fbb);
  b.add_cycle_index(v.cycle_index);
  b.add_dt_sec(v.dt_sec);
  b.add_environment(env_fb);
  esr::replay::Vec3 platform_position_ecef = ToV(v.platform_position_ecef_m);
  esr::replay::Vec3 platform_velocity_ecef = ToV(v.platform_velocity_ecef_mps);
  b.add_platform_entity_id(v.platform_entity_id);
  b.add_has_platform_ecef_kinematics(v.has_platform_ecef_kinematics);
  b.add_platform_position_ecef_m(&platform_position_ecef);
  b.add_platform_velocity_ecef_mps(&platform_velocity_ecef);
  const esr::replay::EulerDeg platform_attitude = ToE(v.platform_attitude_deg);
  b.add_platform_attitude_deg(&platform_attitude);
  b.add_cycle_start_time_s(v.cycle_start_time_s);
  b.add_interference(interference);
  fbb.Finish(b.Finish());
  return oneq::common::replay::CopyFinishedFlatbuffer(fbb);
}

bool DecodeEsrCycleInput(const std::string& bytes, EsrCycleInput* out) {
  flatbuffers::Verifier ver(reinterpret_cast<const uint8_t*>(bytes.data()), bytes.size());
  if (!ver.VerifyBuffer<esr::replay::EsrCycleInput>()) {
    return false;
  }
  const auto* fb = flatbuffers::GetRoot<esr::replay::EsrCycleInput>(bytes.data());
  out->cycle_index = fb->cycle_index();
  out->cycle_start_time_s = fb->cycle_start_time_s();
  out->dt_sec = fb->dt_sec();
  out->platform_entity_id = fb->platform_entity_id();
  out->has_platform_ecef_kinematics = fb->has_platform_ecef_kinematics();
  out->interference = FromRfSceneFrame(fb->interference());
  if (fb->platform_position_ecef_m()) {
    out->platform_position_ecef_m.x_m = fb->platform_position_ecef_m()->x();
    out->platform_position_ecef_m.y_m = fb->platform_position_ecef_m()->y();
    out->platform_position_ecef_m.z_m = fb->platform_position_ecef_m()->z();
  }
  if (fb->platform_velocity_ecef_mps()) {
    out->platform_velocity_ecef_mps.x_mps = fb->platform_velocity_ecef_mps()->x();
    out->platform_velocity_ecef_mps.y_mps = fb->platform_velocity_ecef_mps()->y();
    out->platform_velocity_ecef_mps.z_mps = fb->platform_velocity_ecef_mps()->z();
  }
  if (fb->platform_attitude_deg()) {
    out->platform_attitude_deg.yaw_deg = fb->platform_attitude_deg()->yaw_deg();
    out->platform_attitude_deg.pitch_deg = fb->platform_attitude_deg()->pitch_deg();
    out->platform_attitude_deg.roll_deg = fb->platform_attitude_deg()->roll_deg();
  }
  out->environment = {};
  if (fb->environment()) {
    const auto* e = fb->environment();
    out->environment.propagation_profile =
        static_cast<EsrPropagationEnvironmentProfile>(e->propagation_profile());
    out->environment.clutter_density =
        static_cast<EsrClutterDensityLevel>(e->clutter_density());
    out->environment.spectrum_occupancy_ratio = e->spectrum_occupancy_ratio();
    if (e->atmospheric_observation()) {
      out->environment.atmospheric_observation.relative_humidity_ratio =
          e->atmospheric_observation()->relative_humidity_ratio();
      out->environment.atmospheric_observation.precipitation_rate_mmph =
          e->atmospheric_observation()->precipitation_rate_mmph();
      out->environment.atmospheric_observation.visibility_km =
          e->atmospheric_observation()->visibility_km();
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
    esr::replay::EmitterObservationBuilder builder(fbb);
    builder.add_observation_id(o.observation_id);
    builder.add_timestamp_s(o.timestamp_s);
    builder.add_aoa_az_deg(o.aoa_az_deg);
    builder.add_aoa_el_deg(o.aoa_el_deg);
    builder.add_rf_hz(o.rf_hz);
    builder.add_pulse_width_s(o.pulse_width_s);
    builder.add_amplitude_db(o.amplitude_db);
    builder.add_snr_db(o.snr_db);
    builder.add_quality(static_cast<int32_t>(o.quality));
    builder.add_bandwidth_hz(o.bandwidth_hz);
    builder.add_pri_s(o.pri_s);
    builder.add_rf_std_hz(o.rf_std_hz);
    builder.add_bandwidth_std_hz(o.bandwidth_std_hz);
    builder.add_pri_std_s(o.pri_std_s);
    builder.add_pulse_width_std_s(o.pulse_width_std_s);
    obs_vec.push_back(builder.Finish());
  }
  esr::replay::ObservationOutputBuilder observation_builder(fbb);
  observation_builder.add_raw_observation_count(
      static_cast<std::uint32_t>(v.observation_output.raw_observation_count));
  observation_builder.add_cluster_count(
      static_cast<std::uint32_t>(v.observation_output.cluster_count));
  observation_builder.add_observations(fbb.CreateVector(obs_vec));
  observation_builder.add_receiver_center_frequency_hz(
      v.observation_output.receiver_center_frequency_hz);
  observation_builder.add_receiver_bandwidth_hz(v.observation_output.receiver_bandwidth_hz);
  observation_builder.add_receiver_saturated(v.observation_output.receiver_saturated);
  auto obs_out = observation_builder.Finish();

  // emitter output
  std::vector<flatbuffers::Offset<esr::replay::EmitterHypothesis>> hyp_vec;
  for (const auto& h : v.emitter_output.hypotheses) {
    std::vector<flatbuffers::Offset<flatbuffers::String>> cls_vec;
    for (const auto& c : h.candidate_classes) {
      cls_vec.push_back(fbb.CreateString(c));
    }
    esr::replay::EmitterHypothesisBuilder builder(fbb);
    builder.add_hypothesis_id(h.hypothesis_id);
    builder.add_candidate_classes(fbb.CreateVector(cls_vec));
    builder.add_mode(static_cast<int32_t>(h.mode));
    builder.add_threat_level(static_cast<int32_t>(h.threat_level));
    builder.add_bearing_az_deg(h.bearing_az_deg);
    builder.add_bearing_el_deg(h.bearing_el_deg);
    builder.add_bearing_std_deg(h.bearing_std_deg);
    builder.add_confidence(h.confidence);
    builder.add_last_seen_cycle(h.last_seen_cycle);
    builder.add_estimated_center_frequency_hz(h.estimated_center_frequency_hz);
    builder.add_estimated_bandwidth_hz(h.estimated_bandwidth_hz);
    builder.add_estimated_pri_s(h.estimated_pri_s);
    builder.add_estimated_pulse_width_s(h.estimated_pulse_width_s);
    builder.add_center_frequency_std_hz(h.center_frequency_std_hz);
    builder.add_bandwidth_std_hz(h.bandwidth_std_hz);
    builder.add_pri_std_s(h.pri_std_s);
    builder.add_pulse_width_std_s(h.pulse_width_std_s);
    hyp_vec.push_back(builder.Finish());
  }
  auto em_out = esr::replay::CreateEmitterOutput(fbb, fbb.CreateVector(hyp_vec));

  return esr::replay::CreateEsrOutputFrame(fbb, v.cycle_index, v.batch_id, obs_out, em_out);
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
    out->observation_output.receiver_center_frequency_hz = o->receiver_center_frequency_hz();
    out->observation_output.receiver_bandwidth_hz = o->receiver_bandwidth_hz();
    out->observation_output.receiver_saturated = o->receiver_saturated();
    if (o->observations()) {
      for (const auto* obs : *o->observations()) {
        session::EmitterObservation rec{};
        rec.observation_id = obs->observation_id();
        rec.timestamp_s = obs->timestamp_s();
        rec.aoa_az_deg = obs->aoa_az_deg();
        rec.aoa_el_deg = obs->aoa_el_deg();
        rec.rf_hz = obs->rf_hz();
        rec.bandwidth_hz = obs->bandwidth_hz();
        rec.pri_s = obs->pri_s();
        rec.pulse_width_s = obs->pulse_width_s();
        rec.rf_std_hz = obs->rf_std_hz();
        rec.bandwidth_std_hz = obs->bandwidth_std_hz();
        rec.pri_std_s = obs->pri_std_s();
        rec.pulse_width_std_s = obs->pulse_width_std_s();
        rec.amplitude_db = obs->amplitude_db();
        rec.snr_db = obs->snr_db();
        rec.quality = static_cast<session::EsrObservationQuality>(obs->quality());
        out->observation_output.observations.push_back(rec);
      }
    }
  }
  if (fb->emitter_output()) {
    const auto* e = fb->emitter_output();
    if (e->hypotheses()) {
      for (const auto* h : *e->hypotheses()) {
        session::EmitterHypothesis hyp{};
        hyp.hypothesis_id = h->hypothesis_id();
        hyp.mode = static_cast<session::EsrEmitterMode>(h->mode());
        hyp.threat_level = static_cast<session::EsrThreatLevel>(h->threat_level());
        hyp.bearing_az_deg = h->bearing_az_deg();
        hyp.bearing_el_deg = h->bearing_el_deg();
        hyp.bearing_std_deg = h->bearing_std_deg();
        hyp.estimated_center_frequency_hz = h->estimated_center_frequency_hz();
        hyp.estimated_bandwidth_hz = h->estimated_bandwidth_hz();
        hyp.estimated_pri_s = h->estimated_pri_s();
        hyp.estimated_pulse_width_s = h->estimated_pulse_width_s();
        hyp.center_frequency_std_hz = h->center_frequency_std_hz();
        hyp.bandwidth_std_hz = h->bandwidth_std_hz();
        hyp.pri_std_s = h->pri_std_s();
        hyp.pulse_width_std_s = h->pulse_width_std_s();
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
}

}  // namespace

std::string EncodeEsrOutputFrame(const session::EsrOutputFrame& v) {
  flatbuffers::FlatBufferBuilder fbb(512);
  fbb.Finish(CreateEsrOutputFrameTable(fbb, v));
  return oneq::common::replay::CopyFinishedFlatbuffer(fbb);
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
      static_cast<int32_t>(v.status), static_cast<int32_t>(v.abort_reason)));
  return oneq::common::replay::CopyFinishedFlatbuffer(fbb);
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
  out->status = static_cast<EsrCycleExecutionStatus>(fb->status());
  out->abort_reason = static_cast<session::EsrPipelineAbortReason>(fb->abort_reason());
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
  std::vector<flatbuffers::Offset<esr::replay::EsrTuningWindow>> tuning_plan;
  for (const config::EsrTuningWindow& window : v.hardware.tuning_plan) {
    tuning_plan.push_back(esr::replay::CreateEsrTuningWindow(
        fbb, window.center_frequency_hz, window.bandwidth_hz, window.dwell_cycles));
  }
  std::vector<flatbuffers::Offset<esr::replay::EsrCoSiteIsolationPath>> co_site_paths;
  for (const config::EsrCoSiteIsolationPath& path : v.hardware.co_site_paths) {
    co_site_paths.push_back(esr::replay::CreateEsrCoSiteIsolationPath(
        fbb, path.transmitter_equipment_id, path.isolation_db));
  }
  esr::replay::EsrHardwareConfigBuilder hardware_builder(fbb);
  hardware_builder.add_receiver_equipment_id(v.hardware.receiver_equipment_id);
  hardware_builder.add_receiver_band_lower_hz(v.hardware.receiver_band_lower_hz);
  hardware_builder.add_receiver_band_upper_hz(v.hardware.receiver_band_upper_hz);
  hardware_builder.add_receiver_sensitivity_w(v.hardware.receiver_sensitivity_w);
  hardware_builder.add_receiver_noise_figure_db(v.hardware.receiver_noise_figure_db);
  hardware_builder.add_receiver_reference_temperature_k(
      v.hardware.receiver_reference_temperature_k);
  hardware_builder.add_integrated_receive_loss_db(v.hardware.integrated_receive_loss_db);
  hardware_builder.add_beam_az_width_deg(v.hardware.beam_az_width_deg);
  hardware_builder.add_beam_el_width_deg(v.hardware.beam_el_width_deg);
  hardware_builder.add_az_scan_range_deg(v.hardware.az_scan_range_deg);
  hardware_builder.add_el_scan_range_deg(v.hardware.el_scan_range_deg);
  hardware_builder.add_antenna_mount_az_deg(v.hardware.antenna_mount_az_deg);
  hardware_builder.add_antenna_mount_el_deg(v.hardware.antenna_mount_el_deg);
  hardware_builder.add_antenna_peak_gain_dbi(v.hardware.antenna_peak_gain_dbi);
  hardware_builder.add_antenna_sidelobe_level_db(v.hardware.antenna_sidelobe_level_db);
  hardware_builder.add_antenna_backlobe_level_db(v.hardware.antenna_backlobe_level_db);
  hardware_builder.add_polarization(static_cast<int32_t>(v.hardware.polarization));
  hardware_builder.add_cross_polarization_isolation_db(
      v.hardware.cross_polarization_isolation_db);
  hardware_builder.add_minimum_far_field_range_m(v.hardware.minimum_far_field_range_m);
  hardware_builder.add_co_site_paths(fbb.CreateVector(co_site_paths));
  hardware_builder.add_maximum_linear_input_power_w(v.hardware.maximum_linear_input_power_w);
  hardware_builder.add_tuning_plan(fbb.CreateVector(tuning_plan));
  auto hw = hardware_builder.Finish();
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
  return oneq::common::replay::CopyFinishedFlatbuffer(fbb);
}

bool DecodeEsrSessionConfig(const std::string& bytes, config::EsrSessionConfig* out) {
  flatbuffers::Verifier ver(reinterpret_cast<const uint8_t*>(bytes.data()), bytes.size());
  if (!ver.VerifyBuffer<esr::replay::EsrSessionConfig>()) {
    return false;
  }
  const auto* fb = flatbuffers::GetRoot<esr::replay::EsrSessionConfig>(bytes.data());
  if (fb->hardware()) {
    const auto* h = fb->hardware();
    out->hardware.receiver_equipment_id = h->receiver_equipment_id();
    out->hardware.receiver_band_lower_hz = h->receiver_band_lower_hz();
    out->hardware.receiver_band_upper_hz = h->receiver_band_upper_hz();
    out->hardware.receiver_sensitivity_w = h->receiver_sensitivity_w();
    out->hardware.receiver_noise_figure_db = h->receiver_noise_figure_db();
    out->hardware.receiver_reference_temperature_k = h->receiver_reference_temperature_k();
    out->hardware.integrated_receive_loss_db = h->integrated_receive_loss_db();
    out->hardware.beam_az_width_deg = h->beam_az_width_deg();
    out->hardware.beam_el_width_deg = h->beam_el_width_deg();
    out->hardware.az_scan_range_deg = h->az_scan_range_deg();
    out->hardware.el_scan_range_deg = h->el_scan_range_deg();
    out->hardware.antenna_mount_az_deg = h->antenna_mount_az_deg();
    out->hardware.antenna_mount_el_deg = h->antenna_mount_el_deg();
    out->hardware.antenna_peak_gain_dbi = h->antenna_peak_gain_dbi();
    out->hardware.antenna_sidelobe_level_db = h->antenna_sidelobe_level_db();
    out->hardware.antenna_backlobe_level_db = h->antenna_backlobe_level_db();
    out->hardware.polarization =
        static_cast<oneq::electromagnetics::RfScenePolarization>(h->polarization());
    out->hardware.cross_polarization_isolation_db = h->cross_polarization_isolation_db();
    out->hardware.minimum_far_field_range_m = h->minimum_far_field_range_m();
    out->hardware.co_site_paths.clear();
    if (h->co_site_paths()) {
      for (const esr::replay::EsrCoSiteIsolationPath* path : *h->co_site_paths()) {
        config::EsrCoSiteIsolationPath decoded;
        decoded.transmitter_equipment_id = path->transmitter_equipment_id();
        decoded.isolation_db = path->isolation_db();
        out->hardware.co_site_paths.push_back(decoded);
      }
    }
    out->hardware.maximum_linear_input_power_w = h->maximum_linear_input_power_w();
    out->hardware.tuning_plan.clear();
    if (h->tuning_plan()) {
      for (const esr::replay::EsrTuningWindow* window : *h->tuning_plan()) {
        config::EsrTuningWindow decoded;
        decoded.center_frequency_hz = window->center_frequency_hz();
        decoded.bandwidth_hz = window->bandwidth_hz();
        decoded.dwell_cycles = window->dwell_cycles();
        out->hardware.tuning_plan.push_back(decoded);
      }
    }
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
  return oneq::common::replay::CopyFinishedFlatbuffer(fbb);
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

  return oneq::common::replay::CopyFinishedFlatbuffer(builder);
}

bool DecodeEsrFailureMarker(const std::string& payload_bytes,
                            oneq::replay::ReplayTraceFailure* failure, std::string* error) {
  return oneq::common::replay::DecodeFailureMarkerPayload<esr::replay::FailureMarker>(
      payload_bytes, failure, error);
}

}  // namespace session
}  // namespace electronic_surveillance_radar
