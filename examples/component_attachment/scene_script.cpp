/**
 * @file scene_script.cpp
 * @brief 世界模型目标真值脚本实现（见 scene_script.h）。
 */

#include "scene_script.h"

#include <algorithm>
#include <cmath>

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/velocity_transform.h"

namespace component_attachment {
namespace demo {

std::vector<TargetEcefState> MakeTargetStates(
    const std::vector<ScriptedTarget>& script,
    const oneq::coordinate::LlaPositionDegM& platform_origin) {
  std::vector<TargetEcefState> states;
  states.reserve(script.size());
  for (const auto& entry : script) {
    TargetEcefState state;
    state.id = entry.id;
    state.type = entry.type;
    oneq::coordinate::EnuPositionM offset;
    // 目标脚本（方位/距离/高度）→ ENU 水平偏移；高度由场景文件显式给出
    // （TryBearingRangeToEnuOffset 清零 up，须在此重设）。投影必然成功，
    // 失败时位置留默认零向量。
    if (oneq::coordinate::TryBearingRangeToEnuOffset(entry.azimuth_deg, entry.range_m,
                                                     &offset)) {
      offset.up_m = entry.altitude_m;
      oneq::coordinate::EcefPositionM target;
      if (oneq::coordinate::TryEnuToEcef(offset, platform_origin, &target)) {
        state.position = target;
      }
    }
    oneq::coordinate::EnuVelocityMps enu_velocity;
    enu_velocity.east_mps = entry.v_east_mps;
    enu_velocity.north_mps = entry.v_north_mps;
    enu_velocity.up_mps = 0.0;
    // 脚本输入合法（有限/非负），投影必然成功；失败时 velocity 留默认零向量。
    oneq::coordinate::TryEnuToEcefVelocity(enu_velocity, platform_origin, &state.velocity);
    state.rcs = static_cast<float>(entry.rcs);
    state.temperature_k = static_cast<float>(entry.temperature_k);
    state.projected_area_m2 = static_cast<float>(entry.projected_area_m2);
    state.radiant_intensity_w_per_sr = entry.radiant_intensity_w_per_sr;
    state.emitter_center_frequency_hz = entry.emitter_center_frequency_hz;
    state.has_rir_features = entry.has_rir_features;
    state.has_rir_polarization = entry.has_rir_polarization;
    state.rir_rcs_dbsm = entry.rir_rcs_dbsm;
    state.rir_pol_ch1_dbsm = entry.rir_pol_ch1_dbsm;
    state.rir_pol_ch2_dbsm = entry.rir_pol_ch2_dbsm;
    state.rir_truth_model = entry.rir_truth_model;
    state.rir_scatterers = entry.rir_scatterers;
    state.maneuvers = entry.maneuvers;
    states.push_back(state);
  }
  return states;
}

std::vector<airborne_radar::session::ArTargetInput> MakeArTargetInputs(
    const std::vector<TargetEcefState>& states) {
  std::vector<airborne_radar::session::ArTargetInput> targets;
  targets.reserve(states.size());
  for (const auto& state : states) {
    airborne_radar::session::ArTargetInput target;
    target.target_id = state.id;
    target.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
    target.kinematics.position_ecef_m = state.position;
    target.kinematics.velocity_mps = state.velocity;
    target.rcs = state.rcs;
    target.swerling_type = 0;
    targets.push_back(target);
  }
  return targets;
}

std::vector<oneq::electromagnetics::RfSceneEmission> MakeEmitterTruths(
    const std::vector<TargetEcefState>& states, const EsrEmitterParams& esr,
    double window_start_time_s) {
  std::vector<oneq::electromagnetics::RfSceneEmission> emitters;
  emitters.reserve(states.size());
  for (const auto& state : states) {
    if (state.emitter_center_frequency_hz <= 0.0) {
      continue;  // 该目标不配辐射源（场景级开关）
    }
    oneq::electromagnetics::RfSceneEmission emitter;
    emitter.identity.platform_id = state.id;
    emitter.identity.equipment_id = 1U;
    emitter.identity.emission_id = 1U;
    emitter.position_ecef_m = state.position;
    emitter.velocity_ecef_mps = state.velocity;
    emitter.antenna.peak_gain_dbi = esr.peak_gain_dbi;
    emitter.polarization = oneq::electromagnetics::RfScenePolarization::kHorizontal;
    // 脉冲列 @ 10 GHz 级：单周期积分脉冲数足够，统计检测概率趋近 1。
    if (!oneq::electromagnetics::TryCreateRfPulseTrainWaveform(
            window_start_time_s, state.emitter_center_frequency_hz, esr.bandwidth_hz,
            esr.peak_power_w, esr.pulse_width_s, esr.pri_s, esr.pulse_count, 0.0,
            /*timing_seed=*/esr.timing_seed, /*timing_epoch=*/1U, &emitter.waveform)) {
      continue;  // 波形构造失败：该辐射源本周期不发射
    }
    emitters.push_back(emitter);
  }
  return emitters;
}

std::vector<electro_optical_sensor::session::EosExternalTargetInput> MakeOpticalTargets(
    const std::vector<TargetEcefState>& states) {
  std::vector<electro_optical_sensor::session::EosExternalTargetInput> targets;
  targets.reserve(states.size());
  for (const auto& state : states) {
    electro_optical_sensor::session::EosExternalTargetInput target;
    target.target_id = state.id;
    target.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
    target.kinematics.position_ecef_m = state.position;
    target.kinematics.velocity_mps = state.velocity;
    target.appearance.apparent_temperature_k = state.temperature_k;
    target.appearance.emissivity = 0.92f;
    target.appearance.reflectance = 0.35f;
    target.appearance.projected_area_m2 = state.projected_area_m2;
    targets.push_back(target);
  }
  return targets;
}

std::vector<sbirs_sensor::session::SbirsSceneTarget> MakeSbirsTargetInputs(
    const std::vector<TargetEcefState>& states) {
  std::vector<sbirs_sensor::session::SbirsSceneTarget> targets;
  targets.reserve(states.size());
  for (const auto& state : states) {
    sbirs_sensor::session::SbirsSceneTarget target;
    target.target_id = state.id;
    target.target_name = "ir_target_" + std::to_string(state.id);
    target.position_ecef_m.x = state.position.x_m;
    target.position_ecef_m.y = state.position.y_m;
    target.position_ecef_m.z = state.position.z_m;
    target.radiant_intensity_w_per_sr = state.radiant_intensity_w_per_sr;
    target.velocity_ecef_m_per_s.x = state.velocity.x_mps;
    target.velocity_ecef_m_per_s.y = state.velocity.y_mps;
    target.velocity_ecef_m_per_s.z = state.velocity.z_mps;
    target.has_velocity_ecef_m_per_s = true;
    target.active = true;
    targets.push_back(target);
  }
  return targets;
}

std::vector<sar::session::SarPointTarget> MakeSarPointTargets(
    const std::vector<TargetEcefState>& states) {
  std::vector<sar::session::SarPointTarget> targets;
  targets.reserve(states.size());
  for (const auto& state : states) {
    oneq::coordinate::LlaPositionDegM lla;
    if (!oneq::coordinate::TryEcefToLla(state.position, &lla)) {
      continue;  // 坐标转换失败：该点目标本周期不入 SAR 场景
    }
    sar::session::SarPointTarget target;
    target.target_id = state.id;
    target.target_name = "sar_target_" + std::to_string(state.id);
    target.latitude_deg = lla.latitude_deg;
    target.longitude_deg = lla.longitude_deg;
    target.altitude_m = lla.altitude_m;
    target.radar_cross_section_dbsm =
        10.0 * std::log10(std::max(1.0e-6, static_cast<double>(state.rcs)));
    targets.push_back(target);
  }
  return targets;
}

std::vector<remote_identification_radar::session::RirSceneTarget> MakeRirSceneTargets(
    const std::vector<TargetEcefState>& states,
    const oneq::coordinate::LlaPositionDegM& site_origin) {
  namespace rir = remote_identification_radar::session;
  std::vector<rir::RirSceneTarget> targets;
  targets.reserve(states.size());
  for (const auto& state : states) {
    oneq::coordinate::EnuPositionM enu;
    if (!oneq::coordinate::TryEcefToEnu(state.position, site_origin, &enu)) {
      continue;  // 坐标转换失败：该目标本周期不入 RIR 场景
    }
    oneq::coordinate::EnuVelocityMps enu_velocity;
    if (!oneq::coordinate::TryEcefToEnuVelocity(state.velocity, site_origin, &enu_velocity)) {
      enu_velocity = {};  // 速度变换失败：按静止目标供（位置几何仍有效）
    }
    rir::RirSceneTarget target;
    target.external_target_id = state.id;
    target.target_name = state.rir_truth_model;
    target.position_x = static_cast<float>(enu.east_m);
    target.position_y = static_cast<float>(enu.north_m);
    target.position_z = static_cast<float>(enu.up_m);
    target.velocity_x = static_cast<float>(enu_velocity.east_mps);
    target.velocity_y = static_cast<float>(enu_velocity.north_mps);
    target.velocity_z = static_cast<float>(enu_velocity.up_mps);
    target.rcs = state.rcs;
    target.range_m = static_cast<float>(
        std::sqrt(enu.east_m * enu.east_m + enu.north_m * enu.north_m + enu.up_m * enu.up_m));
    target.target_swerling_type = rir::RirSwerlingType::kSwerling0;
    if (state.has_rir_features) {
      // 特征真值铺样（仿集成测试配方）：视角网格方位 ±5°/步 5°、俯仰 5°~30°/步
      // 10°，RCS 恒为脚本标量；散射器逐条透传。极化仅在场景显式给值时铺样——
      // 缺省 0 dBsm 不是"未提供"（合法物理值），无值硬铺会把错误极化维带进
      // 识别匹配（实测拖低综合分致长时间无法确认）。
      for (float az = -5.0f; az <= 5.0f; az += 5.0f) {
        for (float el = 5.0f; el <= 30.0f; el += 10.0f) {
          rir::RirAspectRcsSample aspect;
          aspect.aspect_az_deg = az;
          aspect.aspect_el_deg = el;
          aspect.rcs_dbsm = static_cast<float>(state.rir_rcs_dbsm);
          target.aspect_rcs_samples.push_back(aspect);
          if (state.has_rir_polarization) {
            rir::RirPolarizationRcsSample polarization;
            polarization.aspect_az_deg = az;
            polarization.aspect_el_deg = el;
            polarization.channel_1_rcs_dbsm = static_cast<float>(state.rir_pol_ch1_dbsm);
            polarization.channel_2_rcs_dbsm = static_cast<float>(state.rir_pol_ch2_dbsm);
            target.polarization_rcs_samples.push_back(polarization);
          }
        }
      }
      for (const auto& scatterer : state.rir_scatterers) {
        rir::RirRangeRcsScatterer entry;
        entry.range_offset_m = static_cast<float>(scatterer.offset_m);
        entry.rcs_dbsm = static_cast<float>(scatterer.rcs_dbsm);
        target.range_rcs_scatterers.push_back(entry);
      }
    }
    targets.push_back(target);
  }
  return targets;
}

void AdvanceTargetStates(std::vector<TargetEcefState>& states, std::uint32_t cycle,
                         double dt_s,
                         const oneq::coordinate::LlaPositionDegM& platform_origin) {
  for (auto& state : states) {
    // 变速机动：start_cycle 严格递增（加载器校验），逐条对比取生效条目。
    for (const auto& maneuver : state.maneuvers) {
      if (maneuver.start_cycle == cycle) {
        // 机动速度为局部 ENU（东/北），投影回 ECEF（与 MakeTargetStates
        // 初始速度投影一致；投影输入合法必然成功，失败时速度留零向量）。
        oneq::coordinate::EnuVelocityMps enu_velocity;
        enu_velocity.east_mps = maneuver.v_east_mps;
        enu_velocity.north_mps = maneuver.v_north_mps;
        enu_velocity.up_mps = 0.0;
        oneq::coordinate::TryEnuToEcefVelocity(enu_velocity, platform_origin,
                                               &state.velocity);
        break;
      }
    }
    state.position.x_m += state.velocity.x_mps * dt_s;
    state.position.y_m += state.velocity.y_mps * dt_s;
    state.position.z_m += state.velocity.z_mps * dt_s;
  }
}

}  // namespace demo
}  // namespace component_attachment
