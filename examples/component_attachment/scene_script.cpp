/**
 * @file scene_script.cpp
 * @brief 世界模型目标真值脚本实现（见 scene_script.h）。
 */

#include "scene_script.h"

#include <algorithm>
#include <cmath>

#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/velocity_transform.h"
#include "demo_config.h"

namespace component_attachment {
namespace demo {
namespace {

/// 目标脚本：2 个空中目标（正北前方 12/14 km）。目标与平台同速东飞
/// （v_east = 巡航地速 ~47 m/s，FD 性能面实际地速，保持平台与目标相对
/// 经度不变）→ 目标恒在平台正北侧方，SAR 视线垂直航迹（正侧视，
/// squint ≈ 0° 全程成立）；北速 ±5 m/s 提供两目标间分离与 AR 径向速度
/// （多普勒）。方位（北偏东 0° = 正北）落在 EOS 扫描覆盖内（平台局部系
/// az 0 = 东，扫描 50°~130°）。巡航段平台 alt 400 m 的 EOS 探测距离窗
/// ≈ [11.5, 22.9] km（min/max 探测俯仰角 2°/1°），目标斜距全程稳定在
/// 窗内。
struct ScriptedTarget {
  double azimuth_deg;       /**< 真方位（北偏东，deg） */
  double range_m;           /**< 斜距（m） */
  double v_east_mps;        /**< 局部东向速度（m/s） */
  double v_north_mps;       /**< 局部北向速度（m/s） */
  double temperature_k;     /**< 等效温度（EOS 外观） */
  float rcs;                /**< 雷达截面积（m²） */
  float projected_area_m2;  /**< 等效投影面积（EOS 外观，m²） */
};

const ScriptedTarget kTargetScript[] = {
    {0.0, 12000.0, 47.0, 5.0, 520.0, 2.2f, 18.0f},
    {0.0, 14000.0, 47.0, -5.0, 540.0, 1.4f, 15.0f},
};

}  // namespace

std::vector<TargetEcefState> MakeTargetStates(
    const oneq::coordinate::EcefPositionM& platform_ecef,
    const oneq::coordinate::LlaPositionDegM& platform_origin) {
  std::vector<TargetEcefState> states;
  states.reserve(std::size(kTargetScript));
  for (const auto& script : kTargetScript) {
    TargetEcefState state;
    oneq::coordinate::EnuPositionM offset;
    oneq::coordinate::EcefPositionM target;
    if (oneq::coordinate::TryBearingRangeToEnuOffset(script.azimuth_deg, script.range_m,
                                                     &offset) &&
        oneq::coordinate::TryEnuToEcef(offset, platform_origin, &target)) {
      state.position.x_m = target.x_m;
      state.position.y_m = target.y_m;
    }
    state.position.z_m = platform_ecef.z_m + kCruiseAltitudeM;
    oneq::coordinate::EnuVelocityMps enu_velocity;
    enu_velocity.east_mps = script.v_east_mps;
    enu_velocity.north_mps = script.v_north_mps;
    enu_velocity.up_mps = 0.0;
    // 脚本输入合法（有限/非负），投影必然成功；失败时 velocity 留默认零向量。
    oneq::coordinate::TryEnuToEcefVelocity(enu_velocity, platform_origin, &state.velocity);
    state.rcs = script.rcs;
    states.push_back(state);
  }
  return states;
}

std::vector<airborne_radar::session::ArTargetInput> MakeArTargetInputs(
    const std::vector<TargetEcefState>& states) {
  std::vector<airborne_radar::session::ArTargetInput> targets;
  targets.reserve(states.size());
  for (std::size_t i = 0U; i < states.size(); ++i) {
    airborne_radar::session::ArTargetInput target;
    target.target_id = 1001U + i;
    target.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
    target.kinematics.position_ecef_m = states[i].position;
    target.kinematics.velocity_mps = states[i].velocity;
    target.rcs = states[i].rcs;
    target.swerling_type = 0;
    targets.push_back(target);
  }
  return targets;
}

std::vector<oneq::electromagnetics::RfSceneEmission> MakeEmitterTruths(
    const std::vector<TargetEcefState>& states, double window_start_time_s) {
  // 中心频率与目标脚本一一对应（每物理目标一个辐射源）。
  const double kCenterFrequencyHz[std::size(kTargetScript)] = {9.5e9, 10.0e9};
  std::vector<oneq::electromagnetics::RfSceneEmission> emitters;
  emitters.reserve(states.size());
  for (std::size_t i = 0U; i < states.size(); ++i) {
    oneq::electromagnetics::RfSceneEmission emitter;
    emitter.identity.platform_id = 1001U + i;
    emitter.identity.equipment_id = 1U;
    emitter.identity.emission_id = 1U;
    emitter.position_ecef_m = states[i].position;
    emitter.velocity_ecef_mps = states[i].velocity;
    emitter.antenna.peak_gain_dbi = 30.0;
    emitter.polarization = oneq::electromagnetics::RfScenePolarization::kHorizontal;
    // 200 脉冲 @ 10 GHz 级：单周期积分脉冲数足够，统计检测概率趋近 1。
    if (!oneq::electromagnetics::TryCreateRfPulseTrainWaveform(
            window_start_time_s, kCenterFrequencyHz[i], 2.0e6, 5.0e7, 1.0e-6, 1.0e-3,
            200U, 0.0, /*timing_seed=*/42U, /*timing_epoch=*/1U, &emitter.waveform)) {
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
  for (std::size_t i = 0U; i < states.size(); ++i) {
    electro_optical_sensor::session::EosExternalTargetInput target;
    target.target_id = 1001U + i;
    target.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
    target.kinematics.position_ecef_m = states[i].position;
    target.kinematics.velocity_mps = states[i].velocity;
    target.appearance.apparent_temperature_k = static_cast<float>(kTargetScript[i].temperature_k);
    target.appearance.emissivity = 0.92f;
    target.appearance.reflectance = 0.35f;
    target.appearance.projected_area_m2 = kTargetScript[i].projected_area_m2;
    targets.push_back(target);
  }
  return targets;
}

std::vector<sbirs_sensor::session::SbirsSceneTarget> MakeSbirsTargetInputs(
    const std::vector<TargetEcefState>& states) {
  std::vector<sbirs_sensor::session::SbirsSceneTarget> targets;
  targets.reserve(states.size());
  for (std::size_t i = 0U; i < states.size(); ++i) {
    sbirs_sensor::session::SbirsSceneTarget target;
    target.target_id = 1001U + i;
    target.target_name = "ir_target_" + std::to_string(1001U + i);
    target.position_ecef_m.x = states[i].position.x_m;
    target.position_ecef_m.y = states[i].position.y_m;
    target.position_ecef_m.z = states[i].position.z_m;
    target.temperature_k = static_cast<float>(kTargetScript[i].temperature_k);
    target.emissivity = 0.92f;
    target.projected_area_m2 = kTargetScript[i].projected_area_m2;
    target.velocity_ecef_m_per_s.x = states[i].velocity.x_mps;
    target.velocity_ecef_m_per_s.y = states[i].velocity.y_mps;
    target.velocity_ecef_m_per_s.z = states[i].velocity.z_mps;
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
  for (std::size_t i = 0U; i < states.size(); ++i) {
    oneq::coordinate::LlaPositionDegM lla;
    if (!oneq::coordinate::TryEcefToLla(states[i].position, &lla)) {
      continue;  // 坐标转换失败：该点目标本周期不入 SAR 场景
    }
    sar::session::SarPointTarget target;
    target.target_id = 1001U + i;
    target.target_name = "sar_target_" + std::to_string(1001U + i);
    target.latitude_deg = lla.latitude_deg;
    target.longitude_deg = lla.longitude_deg;
    target.altitude_m = lla.altitude_m;
    target.radar_cross_section_dbsm =
        10.0 * std::log10(std::max(1.0e-6, static_cast<double>(states[i].rcs)));
    targets.push_back(target);
  }
  return targets;
}

void AdvanceTargetStates(std::vector<TargetEcefState>& states, double dt_s) {
  for (auto& state : states) {
    state.position.x_m += state.velocity.x_mps * dt_s;
    state.position.y_m += state.velocity.y_mps * dt_s;
    state.position.z_m += state.velocity.z_mps * dt_s;
  }
}

}  // namespace demo
}  // namespace component_attachment
