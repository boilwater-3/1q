#include "EosReplayFlatbufferCodec.h"

#include <cstdint>
#include <string>
#include <vector>

#include "1q/electro_optical_sensor/config/EosEnvironmentConfig.h"
#include "1q/electro_optical_sensor/session/EosOutputTypes.h"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "electro_optical_sensor/runtime/EosPipelineConfigMapper.h"
#include "common/replay/ReplayFlatbufferCodecSupport.h"
#include "flatbuffers/flatbuffers.h"
#include "electro_optical_sensor/session/generated/eos_replay_generated.h"
#include "electro_optical_sensor/session/generated/eos_session_replay_generated.h"

namespace electro_optical_sensor {
namespace session {
namespace {

// ---- 辅助构建函数 ----

// 单条 detection record 的 encode/decode 单一源。
// EncodeEosOutputFrame / EncodeEosCycleResult 与对应的 decode 都复用这两个 helper，
// 避免同一份 9 字段映射在四处重复（加字段时只改这里）。
flatbuffers::Offset<eos::replay::EosDetectionRecord> EncodeOneDetection(
    flatbuffers::FlatBufferBuilder& fbb, const output::EosDetectionRecord& d) {
  return eos::replay::CreateEosDetectionRecord(
      fbb, d.detection_id, d.range_m, d.azimuth_deg, d.elevation_deg, d.infrared_snr_linear,
      d.visible_snr_linear, d.fused_snr_linear, d.fused_snr_db, d.detected);
}

void DecodeOneDetection(const eos::replay::EosDetectionRecord& d, output::EosDetectionRecord* out) {
  out->detection_id = d.detection_id();
  out->range_m = d.range_m();
  out->azimuth_deg = d.azimuth_deg();
  out->elevation_deg = d.elevation_deg();
  out->infrared_snr_linear = d.infrared_snr_linear();
  out->visible_snr_linear = d.visible_snr_linear();
  out->fused_snr_linear = d.fused_snr_linear();
  out->fused_snr_db = d.fused_snr_db();
  out->detected = d.detected();
}

eos::replay::Vec3 ToFbVec3(const oneq::foundation::Vector3f& v) {
  return eos::replay::Vec3(v.x, v.y, v.z);
}

eos::replay::EulerDeg ToFbEuler(const oneq::foundation::EulerAnglesDeg& e) {
  return eos::replay::EulerDeg(e.yaw_deg, e.pitch_deg, e.roll_deg);
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
    const auto target_name = fbb.CreateString(t.target_name);
    auto b = eos::replay::CreateEosTargetState(fbb, t.target_id, t.range_m, t.azimuth_deg,
                                               t.elevation_deg, t.appearance.apparent_temperature_k,
                                               t.appearance.emissivity, t.appearance.reflectance,
                                               t.appearance.projected_area_m2, target_name);
    targets_vec.push_back(b);
  }
  auto targets = fbb.CreateVector(targets_vec);
  auto pose = BuildPoseState(fbb, v.platform_pose);

  eos::replay::EosCycleInputBuilder b(fbb);
  b.add_cycle_index(v.cycle_index);
  b.add_dt_sec(v.dt_sec);
  b.add_platform_pose(pose);
  b.add_scene_targets(targets);
  b.add_platform_altitude_m(v.platform_altitude_m);
  fbb.Finish(b.Finish());
  return oneq::common::replay::CopyFinishedFlatbuffer(fbb);
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
  out->scene.clear();
  if (fb->scene_targets()) {
    for (const auto* t : *fb->scene_targets()) {
      EosSceneTarget ts{};
      ts.target_id = t->target_id();
      ts.target_name = t->target_name() ? t->target_name()->str() : std::string();
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
    det_vec.push_back(EncodeOneDetection(fbb, d));
  }
  auto dets = fbb.CreateVector(det_vec);
  auto frame = eos::replay::CreateEosOutputFrame(fbb, v.cycle_index, v.scan_azimuth_deg, dets);
  fbb.Finish(frame);
  return oneq::common::replay::CopyFinishedFlatbuffer(fbb);
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
      DecodeOneDetection(*d, &rec);
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
    det_vec.push_back(EncodeOneDetection(fbb, d));
  }
  // 向量创建前置：CreateVector 必须在 CreateEosOutputFrame 打开之前。
  const auto dets_fb = fbb.CreateVector(det_vec);
  auto frame = eos::replay::CreateEosOutputFrame(
      fbb, v.output_frame.cycle_index, v.output_frame.scan_azimuth_deg, dets_fb);

  std::vector<flatbuffers::Offset<eos::replay::EosDetectionAttributionRecord>> attr_vec;
  attr_vec.reserve(v.detection_attributions.size());
  for (const auto& attribution : v.detection_attributions) {
    const auto target_name = fbb.CreateString(attribution.target_name);
    attr_vec.push_back(eos::replay::CreateEosDetectionAttributionRecord(
        fbb, attribution.detection_id, attribution.target_id, target_name));
  }

  std::vector<flatbuffers::Offset<eos::replay::EosIssue>> issue_vec;
  for (const auto& i : v.issues) {
    const std::size_t encoded_entity_index =
        i.location.kind == oneq::foundation::ValidationLocationKind::kSceneEntity
            ? i.location.entity_index
            : static_cast<std::size_t>(-1);
    auto code_str = fbb.CreateString(i.code);
    auto msg = fbb.CreateString(i.message);
    auto field_str = fbb.CreateString(i.field);
    issue_vec.push_back(eos::replay::CreateEosIssue(
        fbb, static_cast<int32_t>(i.severity), static_cast<int32_t>(i.phase), code_str, msg,
        static_cast<int32_t>(i.location.kind), static_cast<int64_t>(encoded_entity_index),
        field_str, static_cast<int32_t>(i.cause)));
  }

  // 向量创建前置：CreateVector 必须在 CreateEosCycleResult 打开之前。
  const auto attr_fb = fbb.CreateVector(attr_vec);
  const auto issue_fb = fbb.CreateVector(issue_vec);
  auto result = eos::replay::CreateEosCycleResult(
      fbb, v.input_cycle_index, frame, attr_fb,
      static_cast<int32_t>(v.abort_reason), static_cast<std::uint8_t>(v.status), issue_fb);
  fbb.Finish(result);
  return oneq::common::replay::CopyFinishedFlatbuffer(fbb);
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
        DecodeOneDetection(*d, &rec);
        out->output_frame.detections.push_back(rec);
      }
    }
  }
  out->detection_attributions.clear();
  if (fb->detection_attributions()) {
    for (const auto* attr : *fb->detection_attributions()) {
      attribution::EosDetectionAttributionRecord record;
      record.detection_id = attr->detection_id();
      record.target_id = attr->target_id();
      record.target_name = attr->target_name() ? attr->target_name()->str() : std::string();
      out->detection_attributions.push_back(record);
    }
  }
  out->issues.clear();
  if (fb->issues()) {
    for (const auto* i : *fb->issues()) {
      session::EosIssue iss{};
      iss.severity = static_cast<session::EosIssueSeverity>(i->severity());
      iss.phase = static_cast<session::EosIssuePhase>(i->phase());
      if (i->code()) {
        iss.code = i->code()->str();
      }
      if (i->message()) {
        iss.message = i->message()->str();
      }
      // location.kind 独立编码、范围校验后无条件还原（kPlatform 等非 kGlobal 定位保真）；
      // entity_index 仅 kSceneEntity 有效，-1 哨兵还原为无效值。
      if (i->location_kind() <=
          static_cast<int32_t>(oneq::foundation::ValidationLocationKind::kSceneEntity)) {
        iss.location.kind =
            static_cast<oneq::foundation::ValidationLocationKind>(i->location_kind());
        iss.location.entity_index = i->entity_index() >= 0
                                        ? static_cast<std::size_t>(i->entity_index())
                                        : static_cast<std::size_t>(-1);
      } else {
        iss.location.kind = oneq::foundation::ValidationLocationKind::kGlobal;
        iss.location.entity_index = static_cast<std::size_t>(-1);
      }
      if (i->field()) {
        iss.field = i->field()->str();
      }
      iss.cause = static_cast<session::EosIssueCause>(i->cause());
      out->issues.push_back(iss);
    }
  }
  out->abort_reason = static_cast<session::EosPipelineAbortReason>(fb->abort_reason());
  out->status = static_cast<session::EosCycleStatus>(fb->status());
  return true;
}

// ---- config::EosSessionConfig ----

std::string EncodeEosSessionConfig(const config::EosSessionConfig& v) {
  flatbuffers::FlatBufferBuilder fbb(512);

  // hardware
  auto hw = eos::replay::CreateEosHardwareConfig(
      fbb, v.hardware.wavelength_lower_um, v.hardware.wavelength_upper_um,
      v.hardware.optical_aperture_m,
      v.hardware.detector_detectivity_cm_sqrt_hz_per_w, v.hardware.detector_area_cm2,
      v.hardware.min_detection_depression_deg, v.hardware.max_detection_depression_deg);

  // mission
  auto mission = eos::replay::CreateEosMissionConfig(
      fbb, static_cast<int32_t>(v.mission.work_mode), v.mission.horizontal_fov_deg,
      v.mission.vertical_fov_deg, v.mission.scan_rate_deg_per_sec, v.mission.frame_rate_hz,
      v.mission.scan_start_az_deg, v.mission.scan_end_az_deg, v.mission.scan_center_el_deg,
      v.mission.boresight_depression_deg);

  // policy detection
  auto pd = eos::replay::CreateEosPolicyDetectionConfig(
      fbb, v.policy.detection.minimum_snr_db, v.policy.detection.detection_sensitivity_w,
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
  auto atmospheric_physics = eos::replay::CreateEosAtmosphericObservation(
      fbb, sc.atmospheric_physics.enable_physical_model,
      sc.atmospheric_physics.pressure_hpa, sc.atmospheric_physics.temperature_k,
      sc.atmospheric_physics.relative_humidity);
  auto env = eos::replay::CreateEosEnvironmentConfig(
      fbb, static_cast<int32_t>(sc.preset), atmospheric_physics,
      sc.solar_altitude_deg, sc.solar_azimuth_deg, sc.solar_irradiance_w_m2,
      sc.cloud_coverage_ratio, sc.ambient_wind_speed_mps,
      static_cast<int32_t>(sc.day_night_type), sc.background_temperature_k);

  fbb.Finish(
      eos::replay::CreateEosSessionConfig(fbb, hw, mission, policy, env, v.sensor_enabled));
  return oneq::common::replay::CopyFinishedFlatbuffer(fbb);
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
    out->hardware.detector_detectivity_cm_sqrt_hz_per_w =
        fb->hardware()->detector_detectivity_cm_sqrt_hz_per_w();
    out->hardware.detector_area_cm2 = fb->hardware()->detector_area_cm2();
    out->hardware.min_detection_depression_deg = fb->hardware()->min_detection_depression_deg();
    out->hardware.max_detection_depression_deg = fb->hardware()->max_detection_depression_deg();
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
  out->sensor_enabled = fb->sensor_enabled();
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
    sc.preset = static_cast<config::EosEnvironmentPreset>(e->preset());
    if (e->atmospheric_physics()) {
      sc.atmospheric_physics.enable_physical_model =
          e->atmospheric_physics()->enable_physical_model();
      sc.atmospheric_physics.pressure_hpa = e->atmospheric_physics()->pressure_hpa();
      sc.atmospheric_physics.temperature_k = e->atmospheric_physics()->temperature_k();
      sc.atmospheric_physics.relative_humidity =
          e->atmospheric_physics()->relative_humidity();
    }
    sc.solar_altitude_deg = e->solar_altitude_deg();
    sc.solar_azimuth_deg = e->solar_azimuth_deg();
    sc.solar_irradiance_w_m2 = e->solar_irradiance_w_m2();
    sc.cloud_coverage_ratio = e->cloud_coverage_ratio();
    sc.ambient_wind_speed_mps = e->ambient_wind_speed_mps();
    sc.day_night_type = static_cast<config::DayNightType>(e->day_night_type());
    sc.background_temperature_k = e->background_temperature_k();
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
      fbb, v.policy.detection.minimum_snr_db, v.policy.detection.detection_sensitivity_w,
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
    auto atmospheric_physics = eos::replay::CreateEosAtmosphericObservation(
        fbb, ep.scenario_config.atmospheric_physics.enable_physical_model,
        ep.scenario_config.atmospheric_physics.pressure_hpa,
        ep.scenario_config.atmospheric_physics.temperature_k,
        ep.scenario_config.atmospheric_physics.relative_humidity);
    env = eos::replay::CreateEosEnvironmentConfig(
        fbb, static_cast<int32_t>(ep.scenario_config.preset), atmospheric_physics,
        ep.scenario_config.solar_altitude_deg, ep.scenario_config.solar_azimuth_deg,
        ep.scenario_config.solar_irradiance_w_m2, ep.scenario_config.cloud_coverage_ratio,
        ep.scenario_config.ambient_wind_speed_mps,
        static_cast<int32_t>(ep.scenario_config.day_night_type),
        ep.scenario_config.background_temperature_k);
  } else {
    env = eos::replay::CreateEosEnvironmentConfig(
        fbb, 0, flatbuffers::Offset<eos::replay::EosAtmosphericObservation>{},
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0.0f);
  }
  fbb.Finish(eos::replay::CreateEosRuntimeConfigPatch(
      fbb, v.has_mission, mission, v.has_policy, policy, v.has_environment, env,
      v.environment.has_scenario_config, v.has_work_mode, static_cast<int32_t>(v.work_mode),
      v.has_scan_rate_deg_per_sec, v.scan_rate_deg_per_sec, v.has_frame_rate_hz, v.frame_rate_hz,
      v.has_sensor_enabled, v.sensor_enabled));
  return oneq::common::replay::CopyFinishedFlatbuffer(fbb);
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
    out->environment.has_scenario_config = fb->has_scenario_config_in_environment();
    out->environment.scenario_config.preset =
        static_cast<config::EosEnvironmentPreset>(e->preset());
    if (e->atmospheric_physics()) {
      out->environment.scenario_config.atmospheric_physics.enable_physical_model =
          e->atmospheric_physics()->enable_physical_model();
      out->environment.scenario_config.atmospheric_physics.pressure_hpa =
          e->atmospheric_physics()->pressure_hpa();
      out->environment.scenario_config.atmospheric_physics.temperature_k =
          e->atmospheric_physics()->temperature_k();
      out->environment.scenario_config.atmospheric_physics.relative_humidity =
          e->atmospheric_physics()->relative_humidity();
    }
    out->environment.scenario_config.solar_altitude_deg = e->solar_altitude_deg();
    out->environment.scenario_config.solar_azimuth_deg = e->solar_azimuth_deg();
    out->environment.scenario_config.solar_irradiance_w_m2 = e->solar_irradiance_w_m2();
    out->environment.scenario_config.cloud_coverage_ratio = e->cloud_coverage_ratio();
    out->environment.scenario_config.ambient_wind_speed_mps = e->ambient_wind_speed_mps();
    out->environment.scenario_config.day_night_type =
        static_cast<config::DayNightType>(e->day_night_type());
    out->environment.scenario_config.background_temperature_k = e->background_temperature_k();
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
  out->has_sensor_enabled = fb->has_sensor_enabled();
  out->sensor_enabled = fb->sensor_enabled();
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

std::string EncodeEosFailureMarker(const oneq::replay::ReplayTraceFailure& failure) {
  flatbuffers::FlatBufferBuilder builder;
  const flatbuffers::Offset<eos::replay::FailureMarker> root =
      eos::replay::CreateFailureMarkerDirect(
          builder, failure.error_code.c_str(), failure.message.c_str(), failure.location.c_str(),
          failure.has_cycle_index, failure.cycle_index, failure.has_sim_time_sec,
          failure.sim_time_sec, failure.diagnostics_payload.c_str(), false, 0U);
  builder.Finish(root);

  return oneq::common::replay::CopyFinishedFlatbuffer(builder);
}

bool DecodeEosFailureMarker(const std::string& payload_bytes,
                            oneq::replay::ReplayTraceFailure* failure, std::string* error) {
  return oneq::common::replay::DecodeFailureMarkerPayload<eos::replay::FailureMarker>(
      payload_bytes, failure, error);
}

}  // namespace session
}  // namespace electro_optical_sensor
