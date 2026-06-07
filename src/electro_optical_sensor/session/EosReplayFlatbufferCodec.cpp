#include "EosReplayFlatbufferCodec.h"

#include <cstdint>
#include <string>
#include <vector>

#include "1q/electro_optical_sensor/environment/EosEnvironmentConfig.h"
#include "1q/electro_optical_sensor/extension/EosPipelineTypes.h"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "electro_optical_sensor/runtime/EosPipelineConfigMapper.h"
#include "flatbuffers/flatbuffers.h"
#include "generated/eos_replay_generated.h"
#include "generated/eos_session_replay_generated.h"

namespace electro_optical_sensor {
namespace session {
namespace {

// ---- 辅助构建函数 ----

eos::replay::Vec3 ToFbVec3(const oneq::foundation::Vector3f& v) {
  return eos::replay::Vec3(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
}

eos::replay::EulerDeg ToFbEuler(const oneq::foundation::EulerAnglesDeg& e) {
  return eos::replay::EulerDeg(static_cast<float>(e.yaw_deg), static_cast<float>(e.pitch_deg),
                               static_cast<float>(e.roll_deg));
}

flatbuffers::Offset<eos::replay::PoseState> BuildPoseState(flatbuffers::FlatBufferBuilder& fbb,
                                                           const oneq::foundation::PoseState& v) {
  eos::replay::PoseStateBuilder b(fbb);
  auto pos = ToFbVec3(v.position_m);
  auto vel = ToFbVec3(v.velocity_mps);
  auto att = ToFbEuler(v.attitude_deg);
  b.add_position_m(&pos);
  b.add_velocity_mps(&vel);
  b.add_attitude_deg(&att);
  return b.Finish();
}

oneq::foundation::PoseState FromFbPoseState(const eos::replay::PoseState* fb) {
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

// ---- EosCycleInput ----

}  // namespace

std::string EncodeEosCycleInput(const EosCycleInput& v) {
  flatbuffers::FlatBufferBuilder fbb(512);

  std::vector<flatbuffers::Offset<eos::replay::EosTargetState>> targets_vec;
  targets_vec.reserve(v.scene.size());
  for (const auto& t : v.scene) {
    auto b = eos::replay::CreateEosTargetState(
        fbb, static_cast<std::uint32_t>(t.target_id), t.range_m, t.azimuth_deg, t.elevation_deg,
        t.appearance.apparent_temperature_k, t.appearance.emissivity, t.appearance.reflectance,
        t.appearance.projected_area_m2);
    targets_vec.push_back(b);
  }
  auto targets = fbb.CreateVector(targets_vec);
  auto pose = BuildPoseState(fbb, v.platform_pose);
  auto env = eos::replay::CreateEosEnvironmentInput(
      fbb, v.environment.solar_altitude_deg, v.environment.solar_azimuth_deg,
      v.environment.solar_irradiance_w_m2, v.environment.cloud_coverage_ratio,
      v.environment.ambient_wind_speed_mps, static_cast<int32_t>(v.environment.day_night_type),
      v.environment.background_temperature_k);

  eos::replay::EosCycleInputBuilder b(fbb);
  b.add_cycle_index(v.cycle_index);
  b.add_dt_sec(v.dt_sec);
  b.add_platform_pose(pose);
  b.add_environment(env);
  b.add_scene_targets(targets);
  b.add_platform_altitude_m(v.platform_altitude_m);
  fbb.Finish(b.Finish());
  const uint8_t* buf = fbb.GetBufferPointer();
  return std::string(reinterpret_cast<const char*>(buf), fbb.GetSize());
}

bool DecodeEosCycleInput(const std::string& bytes, EosCycleInput* out) {
  flatbuffers::Verifier verifier(reinterpret_cast<const uint8_t*>(bytes.data()), bytes.size());
  if (!verifier.VerifyBuffer<eos::replay::EosCycleInput>()) {
    return false;
  }
  const auto* fb = flatbuffers::GetRoot<eos::replay::EosCycleInput>(bytes.data());
  out->cycle_index = fb->cycle_index();
  out->dt_sec = fb->dt_sec();
  out->platform_altitude_m = fb->platform_altitude_m();
  out->platform_pose = FromFbPoseState(fb->platform_pose());
  if (fb->environment()) {
    const auto* env = fb->environment();
    out->environment.solar_altitude_deg = env->solar_altitude_deg();
    out->environment.solar_azimuth_deg = env->solar_azimuth_deg();
    out->environment.solar_irradiance_w_m2 = env->solar_irradiance_w_m2();
    out->environment.cloud_coverage_ratio = env->cloud_coverage_ratio();
    out->environment.ambient_wind_speed_mps = env->ambient_wind_speed_mps();
    out->environment.day_night_type =
        static_cast<::electro_optical_sensor::session::DayNightType>(env->day_night_type());
    out->environment.background_temperature_k = env->background_temperature_k();
  }
  out->scene.clear();
  if (fb->scene_targets()) {
    for (const auto* t : *fb->scene_targets()) {
      EosSceneTarget ts{};
      ts.target_id = t->target_id();
      ts.range_m = t->range_m();
      ts.azimuth_deg = t->azimuth_deg();
      ts.elevation_deg = t->elevation_deg();
      ts.appearance.apparent_temperature_k = t->apparent_temperature_k();
      ts.appearance.emissivity = t->emissivity();
      ts.appearance.reflectance = t->reflectance();
      ts.appearance.projected_area_m2 = t->projected_area_m2();
      out->scene.push_back(ts);
    }
  }
  return true;
}

// ---- EosOutputFrame ----

std::string EncodeEosOutputFrame(const session::EosOutputFrame& v) {
  flatbuffers::FlatBufferBuilder fbb(256);
  std::vector<flatbuffers::Offset<eos::replay::EosDetectionRecord>> det_vec;
  det_vec.reserve(v.detections.size());
  for (const auto& d : v.detections) {
    det_vec.push_back(eos::replay::CreateEosDetectionRecord(
        fbb, static_cast<std::uint32_t>(d.target_id), d.range_m, d.azimuth_deg, d.elevation_deg,
        d.infrared_snr_linear, d.visible_snr_linear, d.fused_snr_linear, d.fused_snr_db,
        d.detected));
  }
  auto dets = fbb.CreateVector(det_vec);
  auto frame = eos::replay::CreateEosOutputFrame(fbb, v.cycle_index, v.scan_azimuth_deg, dets);
  fbb.Finish(frame);
  const uint8_t* buf = fbb.GetBufferPointer();
  return std::string(reinterpret_cast<const char*>(buf), fbb.GetSize());
}

bool DecodeEosOutputFrame(const std::string& bytes, session::EosOutputFrame* out) {
  flatbuffers::Verifier verifier(reinterpret_cast<const uint8_t*>(bytes.data()), bytes.size());
  if (!verifier.VerifyBuffer<eos::replay::EosOutputFrame>()) {
    return false;
  }
  const auto* fb = flatbuffers::GetRoot<eos::replay::EosOutputFrame>(bytes.data());
  out->cycle_index = fb->cycle_index();
  out->scan_azimuth_deg = fb->scan_azimuth_deg();
  out->detections.clear();
  if (fb->detections()) {
    for (const auto* d : *fb->detections()) {
      output::EosDetectionRecord rec{};
      rec.target_id = d->target_id();
      rec.range_m = d->range_m();
      rec.azimuth_deg = d->azimuth_deg();
      rec.elevation_deg = d->elevation_deg();
      rec.infrared_snr_linear = d->infrared_snr_linear();
      rec.visible_snr_linear = d->visible_snr_linear();
      rec.fused_snr_linear = d->fused_snr_linear();
      rec.fused_snr_db = d->fused_snr_db();
      rec.detected = d->detected();
      out->detections.push_back(rec);
    }
  }
  return true;
}

// ---- EosCycleResult ----

std::string EncodeEosCycleResult(const ::electro_optical_sensor::session::EosCycleResult& v) {
  flatbuffers::FlatBufferBuilder fbb(512);

  // 先编码 output_frame
  std::vector<flatbuffers::Offset<eos::replay::EosDetectionRecord>> det_vec;
  for (const auto& d : v.output_frame.detections) {
    det_vec.push_back(eos::replay::CreateEosDetectionRecord(
        fbb, static_cast<std::uint32_t>(d.target_id), d.range_m, d.azimuth_deg, d.elevation_deg,
        d.infrared_snr_linear, d.visible_snr_linear, d.fused_snr_linear, d.fused_snr_db,
        d.detected));
  }
  auto frame = eos::replay::CreateEosOutputFrame(
      fbb, v.output_frame.cycle_index, v.output_frame.scan_azimuth_deg, fbb.CreateVector(det_vec));

  std::vector<flatbuffers::Offset<eos::replay::ValidationIssue>> issue_vec;
  for (const auto& i : v.validation_issues) {
    const std::size_t encoded_entity_index =
        i.location.kind == session::ValidationLocationKind::kSceneEntity
            ? i.location.entity_index
            : static_cast<std::size_t>(-1);
    auto field_str = fbb.CreateString(i.field);
    auto msg = fbb.CreateString(i.message);
    issue_vec.push_back(eos::replay::CreateValidationIssue(
        fbb, static_cast<int32_t>(i.severity), static_cast<int32_t>(i.code),
        static_cast<int32_t>(i.location.kind), static_cast<int32_t>(encoded_entity_index),
        field_str, msg));
  }

  auto result = eos::replay::CreateEosCycleResult(
      fbb, v.input_cycle_index, frame, fbb.CreateVector(issue_vec), v.has_validation_error,
      v.executed_this_cycle, v.reused_previous_output, static_cast<int32_t>(v.abort_reason));
  fbb.Finish(result);
  const uint8_t* buf = fbb.GetBufferPointer();
  return std::string(reinterpret_cast<const char*>(buf), fbb.GetSize());
}

bool DecodeEosCycleResult(const std::string& bytes,
                          ::electro_optical_sensor::session::EosCycleResult* out) {
  flatbuffers::Verifier verifier(reinterpret_cast<const uint8_t*>(bytes.data()), bytes.size());
  if (!verifier.VerifyBuffer<eos::replay::EosCycleResult>()) {
    return false;
  }
  const auto* fb = flatbuffers::GetRoot<eos::replay::EosCycleResult>(bytes.data());
  out->input_cycle_index = fb->input_cycle_index();
  if (fb->output_frame()) {
    // 直接从 fb 还原
    const auto* frm = fb->output_frame();
    out->output_frame.cycle_index = frm->cycle_index();
    out->output_frame.scan_azimuth_deg = frm->scan_azimuth_deg();
    out->output_frame.detections.clear();
    if (frm->detections()) {
      for (const auto* d : *frm->detections()) {
        output::EosDetectionRecord rec{};
        rec.target_id = d->target_id();
        rec.range_m = d->range_m();
        rec.azimuth_deg = d->azimuth_deg();
        rec.elevation_deg = d->elevation_deg();
        rec.infrared_snr_linear = d->infrared_snr_linear();
        rec.visible_snr_linear = d->visible_snr_linear();
        rec.fused_snr_linear = d->fused_snr_linear();
        rec.fused_snr_db = d->fused_snr_db();
        rec.detected = d->detected();
        out->output_frame.detections.push_back(rec);
      }
    }
  }
  out->has_validation_error = fb->has_validation_error();
  out->executed_this_cycle = fb->executed_this_cycle();
  out->reused_previous_output = fb->reused_previous_output();
  out->abort_reason = static_cast<extension::EosPipelineAbortReason>(fb->abort_reason());
  out->validation_issues.clear();
  if (fb->validation_issues()) {
    for (const auto* i : *fb->validation_issues()) {
      session::ValidationIssue iss{};
      iss.severity = static_cast<session::ValidationSeverity>(i->severity());
      iss.code = static_cast<session::ValidationCode>(i->code());
      iss.location.kind = static_cast<session::ValidationLocationKind>(i->location_kind());
      iss.location.entity_index = static_cast<std::size_t>(i->entity_index());
      if (i->entity_index() < 0) {
        iss.location.kind = session::ValidationLocationKind::kGlobal;
        iss.location.entity_index = static_cast<std::size_t>(-1);
      }
      if (i->field()) {
        iss.field = i->field()->str();
      }
      if (i->message()) {
        iss.message = i->message()->str();
      }
      out->validation_issues.push_back(iss);
    }
  }
  return true;
}

// ---- config::EosSessionConfig ----

std::string EncodeEosSessionConfig(const config::EosSessionConfig& v) {
  flatbuffers::FlatBufferBuilder fbb(512);

  // hardware
  auto hw = eos::replay::CreateEosHardwareConfig(
      fbb, v.hardware.wavelength_lower_um, v.hardware.wavelength_upper_um,
      v.hardware.optical_aperture_m, v.hardware.focal_length_m);

  // mission
  auto mission = eos::replay::CreateEosMissionConfig(
      fbb, static_cast<int32_t>(v.mission.work_mode), v.mission.horizontal_fov_deg,
      v.mission.vertical_fov_deg, v.mission.scan_rate_deg_per_sec, v.mission.frame_rate_hz,
      v.mission.scan_start_az_deg, v.mission.scan_end_az_deg, v.mission.scan_center_el_deg,
      v.mission.boresight_depression_deg);

  // policy detection
  auto pd = eos::replay::CreateEosPolicyDetectionConfig(
      fbb, v.policy.detection.minimum_snr_db,
      v.policy.detection.detection_sensitivity_w,
      v.policy.detection.visible_reference_irradiance_w_m2);
  auto ps = eos::replay::CreateEosPolicyStrayLightConfig(
      fbb, v.policy.stray_light.enable_straylight_filter,
      v.policy.stray_light.hood_inner_half_angle_deg,
      v.policy.stray_light.hood_outer_half_angle_deg,
      v.policy.stray_light.hood_min_suppression_ratio,
      v.policy.stray_light.hood_max_suppression_ratio);
  auto policy = eos::replay::CreateEosPolicyConfig(fbb, pd, ps);

  // environment
  const auto& sc = v.environment.scenario_config;
  const auto model_cfg = environment::BuildModelConfigFromScenario(sc);
  auto co = eos::replay::CreateEosEnvironmentCustomOverrides(
      fbb, static_cast<int32_t>(sc.custom_overrides.radiative_transfer_model),
      sc.custom_overrides.aerosol_density_factor, sc.custom_overrides.turbulence_factor,
      sc.custom_overrides.enable_optical_countermeasure_extension);
  auto env = eos::replay::CreateEosEnvironmentConfig(
      fbb, static_cast<int32_t>(sc.model_type), static_cast<int32_t>(sc.preset),
      sc.has_custom_overrides, co, static_cast<int32_t>(model_cfg.radiative_transfer_model),
      model_cfg.aerosol_density_factor, model_cfg.turbulence_factor,
      model_cfg.enable_optical_countermeasure_extension);

  fbb.Finish(eos::replay::CreateEosSessionConfig(fbb, hw, mission, policy, env));
  const uint8_t* buf = fbb.GetBufferPointer();
  return std::string(reinterpret_cast<const char*>(buf), fbb.GetSize());
}

bool DecodeEosSessionConfig(const std::string& bytes, config::EosSessionConfig* out) {
  flatbuffers::Verifier verifier(reinterpret_cast<const uint8_t*>(bytes.data()), bytes.size());
  if (!verifier.VerifyBuffer<eos::replay::EosSessionConfig>()) {
    return false;
  }
  const auto* fb = flatbuffers::GetRoot<eos::replay::EosSessionConfig>(bytes.data());

  if (fb->hardware()) {
    out->hardware.wavelength_lower_um = fb->hardware()->wavelength_lower_um();
    out->hardware.wavelength_upper_um = fb->hardware()->wavelength_upper_um();
    out->hardware.optical_aperture_m = fb->hardware()->optical_aperture_m();
    out->hardware.focal_length_m = fb->hardware()->focal_length_m();
  }
  if (fb->mission()) {
    const auto* m = fb->mission();
    out->mission.work_mode = static_cast<config::EosWorkMode>(m->work_mode());
    out->mission.horizontal_fov_deg = m->horizontal_fov_deg();
    out->mission.vertical_fov_deg = m->vertical_fov_deg();
    out->mission.scan_rate_deg_per_sec = m->scan_rate_deg_per_sec();
    out->mission.frame_rate_hz = m->frame_rate_hz();
    out->mission.scan_start_az_deg = m->scan_start_az_deg();
    out->mission.scan_end_az_deg = m->scan_end_az_deg();
    out->mission.scan_center_el_deg = m->scan_center_el_deg();
    out->mission.boresight_depression_deg = m->boresight_depression_deg();
  }
  if (fb->policy() && fb->policy()->detection()) {
    const auto* pd = fb->policy()->detection();
    out->policy.detection.minimum_snr_db = pd->minimum_snr_db();
    out->policy.detection.detection_sensitivity_w = pd->detection_sensitivity_w();
    out->policy.detection.visible_reference_irradiance_w_m2 =
        pd->visible_reference_irradiance_w_m2();
  }
  if (fb->policy() && fb->policy()->stray_light()) {
    const auto* ps = fb->policy()->stray_light();
    out->policy.stray_light.enable_straylight_filter = ps->enable_straylight_filter();
    out->policy.stray_light.hood_inner_half_angle_deg = ps->hood_inner_half_angle_deg();
    out->policy.stray_light.hood_outer_half_angle_deg = ps->hood_outer_half_angle_deg();
    out->policy.stray_light.hood_min_suppression_ratio = ps->hood_min_suppression_ratio();
    out->policy.stray_light.hood_max_suppression_ratio = ps->hood_max_suppression_ratio();
  }
  if (fb->environment()) {
    const auto* e = fb->environment();
    auto& sc = out->environment.scenario_config;
    sc.model_type = static_cast<environment::EosEnvironmentModelType>(e->model_type());
    sc.preset = static_cast<environment::EosEnvironmentPreset>(e->preset());
    sc.has_custom_overrides = e->has_custom_overrides();
    if (e->custom_overrides()) {
      const auto* co = e->custom_overrides();
      sc.custom_overrides.radiative_transfer_model =
          static_cast<foundation::radiative_transfer::RadiativeTransferModel>(
              co->radiative_transfer_model());
      sc.custom_overrides.aerosol_density_factor = co->aerosol_density_factor();
      sc.custom_overrides.turbulence_factor = co->turbulence_factor();
      sc.custom_overrides.enable_optical_countermeasure_extension =
          co->enable_optical_countermeasure_extension();
    }
  }
  return true;
}

// ---- config::EosRuntimeConfigPatch ----

std::string EncodeEosRuntimeConfigPatch(const config::EosRuntimeConfigPatch& v) {
  flatbuffers::FlatBufferBuilder fbb(256);
  auto mission = eos::replay::CreateEosMissionConfig(
      fbb, static_cast<int32_t>(v.mission.work_mode), v.mission.horizontal_fov_deg,
      v.mission.vertical_fov_deg, v.mission.scan_rate_deg_per_sec, v.mission.frame_rate_hz,
      v.mission.scan_start_az_deg, v.mission.scan_end_az_deg, v.mission.scan_center_el_deg,
      v.mission.boresight_depression_deg);
  auto pd = eos::replay::CreateEosPolicyDetectionConfig(
      fbb, v.policy.detection.minimum_snr_db,
      v.policy.detection.detection_sensitivity_w,
      v.policy.detection.visible_reference_irradiance_w_m2);
  auto ps = eos::replay::CreateEosPolicyStrayLightConfig(
      fbb, v.policy.stray_light.enable_straylight_filter,
      v.policy.stray_light.hood_inner_half_angle_deg,
      v.policy.stray_light.hood_outer_half_angle_deg,
      v.policy.stray_light.hood_min_suppression_ratio,
      v.policy.stray_light.hood_max_suppression_ratio);
  auto policy = eos::replay::CreateEosPolicyConfig(fbb, pd, ps);
  const auto& ep = v.environment;
  flatbuffers::Offset<eos::replay::EosEnvironmentConfig> env = 0;
  if (ep.has_scenario_config) {
    auto co = eos::replay::CreateEosEnvironmentCustomOverrides(
        fbb, static_cast<int32_t>(ep.scenario_config.custom_overrides.radiative_transfer_model),
        ep.scenario_config.custom_overrides.aerosol_density_factor,
        ep.scenario_config.custom_overrides.turbulence_factor,
        ep.scenario_config.custom_overrides.enable_optical_countermeasure_extension);
    env = eos::replay::CreateEosEnvironmentConfig(
        fbb, static_cast<int32_t>(ep.scenario_config.model_type),
        static_cast<int32_t>(ep.scenario_config.preset), ep.scenario_config.has_custom_overrides,
        co, 0, 0.0f, 0.0f, false);  // derived fields set to 0/false for patch
  } else {
    // Write an empty environment config just to satisfy the struct
    auto co = eos::replay::CreateEosEnvironmentCustomOverrides(fbb, 0, 1.0f, 1.0f, false);
    env = eos::replay::CreateEosEnvironmentConfig(fbb, 0, 0, false, co, 0, 0.0f, 0.0f, false);
  }
  fbb.Finish(eos::replay::CreateEosRuntimeConfigPatch(
      fbb, v.has_mission, mission, v.has_policy, policy, v.has_environment, env, v.has_work_mode,
      static_cast<int32_t>(v.work_mode), v.has_scan_rate_deg_per_sec, v.scan_rate_deg_per_sec,
      v.has_frame_rate_hz, v.frame_rate_hz));
  const uint8_t* buf = fbb.GetBufferPointer();
  return std::string(reinterpret_cast<const char*>(buf), fbb.GetSize());
}

bool DecodeEosRuntimeConfigPatch(const std::string& bytes, config::EosRuntimeConfigPatch* out) {
  flatbuffers::Verifier verifier(reinterpret_cast<const uint8_t*>(bytes.data()), bytes.size());
  if (!verifier.VerifyBuffer<eos::replay::EosRuntimeConfigPatch>()) {
    return false;
  }
  const auto* fb = flatbuffers::GetRoot<eos::replay::EosRuntimeConfigPatch>(bytes.data());
  out->has_mission = fb->has_mission();
  out->has_policy = fb->has_policy();
  out->has_environment = fb->has_environment();
  if (out->has_environment && fb->environment()) {
    const auto* e = fb->environment();
    out->environment.has_scenario_config = true;
    out->environment.scenario_config.model_type =
        static_cast<environment::EosEnvironmentModelType>(e->model_type());
    out->environment.scenario_config.preset =
        static_cast<environment::EosEnvironmentPreset>(e->preset());
    out->environment.scenario_config.has_custom_overrides = e->has_custom_overrides();
    if (e->custom_overrides()) {
      out->environment.scenario_config.custom_overrides.radiative_transfer_model =
          static_cast<foundation::radiative_transfer::RadiativeTransferModel>(
              e->custom_overrides()->radiative_transfer_model());
      out->environment.scenario_config.custom_overrides.aerosol_density_factor =
          e->custom_overrides()->aerosol_density_factor();
      out->environment.scenario_config.custom_overrides.turbulence_factor =
          e->custom_overrides()->turbulence_factor();
      out->environment.scenario_config.custom_overrides.enable_optical_countermeasure_extension =
          e->custom_overrides()->enable_optical_countermeasure_extension();
    }
  }
  if (fb->policy()) {
    const auto* p = fb->policy();
    if (p->detection()) {
      out->policy.detection.minimum_snr_db = p->detection()->minimum_snr_db();
      out->policy.detection.detection_sensitivity_w = p->detection()->detection_sensitivity_w();
      out->policy.detection.visible_reference_irradiance_w_m2 =
          p->detection()->visible_reference_irradiance_w_m2();
    }
    if (p->stray_light()) {
      out->policy.stray_light.enable_straylight_filter =
          p->stray_light()->enable_straylight_filter();
      out->policy.stray_light.hood_inner_half_angle_deg =
          p->stray_light()->hood_inner_half_angle_deg();
      out->policy.stray_light.hood_outer_half_angle_deg =
          p->stray_light()->hood_outer_half_angle_deg();
      out->policy.stray_light.hood_min_suppression_ratio =
          p->stray_light()->hood_min_suppression_ratio();
      out->policy.stray_light.hood_max_suppression_ratio =
          p->stray_light()->hood_max_suppression_ratio();
    }
  }
  out->has_work_mode = fb->has_work_mode();
  out->work_mode = static_cast<config::EosWorkMode>(fb->work_mode());
  out->has_scan_rate_deg_per_sec = fb->has_scan_rate_deg_per_sec();
  out->scan_rate_deg_per_sec = fb->scan_rate_deg_per_sec();
  out->has_frame_rate_hz = fb->has_frame_rate_hz();
  out->frame_rate_hz = fb->frame_rate_hz();
  if (fb->mission()) {
    const auto* m = fb->mission();
    out->mission.work_mode = static_cast<config::EosWorkMode>(m->work_mode());
    out->mission.horizontal_fov_deg = m->horizontal_fov_deg();
    out->mission.vertical_fov_deg = m->vertical_fov_deg();
    out->mission.scan_rate_deg_per_sec = m->scan_rate_deg_per_sec();
    out->mission.frame_rate_hz = m->frame_rate_hz();
    out->mission.scan_start_az_deg = m->scan_start_az_deg();
    out->mission.scan_end_az_deg = m->scan_end_az_deg();
    out->mission.scan_center_el_deg = m->scan_center_el_deg();
    out->mission.boresight_depression_deg = m->boresight_depression_deg();
  }
  return true;
}

}  // namespace session
}  // namespace electro_optical_sensor
