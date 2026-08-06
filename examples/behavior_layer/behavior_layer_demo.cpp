/**
 * @file behavior_layer_demo.cpp
 * @brief 行为层参考实现主程序（EnTT registry + 每周期系统调用序）。
 *
 * 三传感器（AR / ESR / EOS）端到端全链演示：
 *   AR 轨迹 / ESR 辐射源假设 / EOS 光电探测 → recon 适配 → fusion 引擎
 *   → 融合态势（AR 身份直挂；ESR 身份直挂 + 方位 + 射频特征；EOS 无身份
 *     仅方位经方位相干并入 ESR 航迹）→ maneuver 规划航路 → jam 构造
 *     ECM 输入帧 → decision 产出命令帧 → 消费方读取并驱动执行面。
 *
 * 平台动力学：flight_system 驱动（ONEQ_ENABLE_FLIGHT_DYNAMIC=ON 时为 JSBSim
 * c172x 真实飞行仿真，RoutePlan → FlightManager 机动队列适配；关闭或数据
 * 缺失时回退运动学近似），每周期同步到三传感器实体。
 *
 * 事件报告节奏：每周期迭代 entt::observer（新目标/消失事件），
 * 命令 = 写 CommandFrameComponent（无全局事件总线，冻结契约 §5）。
 *
 * 坐标系约定：三会话共享零姿态平台局部系（az 0 = 东，atan2(north, east)），
 * 目标脚本方位（北偏东）经 ENU 基转换到 ECEF 偏移；EOS 扫描 ±40° 覆盖。
 */

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <entt/entt.hpp>

#include "1q/airborne_radar/session/ArSession.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/types.h"
#include "1q/coordinate/velocity_transform.h"
#include "1q/electro_optical_sensor/session/EosExternalInputAdapter.h"
#include "1q/electromagnetics/RfScene.h"
#include "1q/fusion/FusedTarget.h"
#include "1q/navigation/RoutePoint.h"
// 三域配置加载器现位于 examples/common/config_loaders/<域>/（经 ONEQ_EXAMPLE_COMMON_DIR 解析）。
#include "config_loaders/airborne_radar/config_loader.h"
#include "config_loaders/electro_optical/config_loader.h"
#include "config_loaders/electronic_warfare/config_loader.h"
#include "assembly.h"
#include "components.h"
#include "flight_system.h"
#include "systems.h"
#include "viz_recorder.h"

namespace ar = airborne_radar;
namespace ar_session = airborne_radar::session;
namespace bl = behavior_layer;

namespace {

constexpr std::uint32_t kNumCycles = 200U;

/// 可视化 CSV 默认输出目录（可用 --output-dir 覆盖）。
constexpr char kDefaultOutputDir[] = "/tmp/behavior_layer_viz";

/// 打印命令行用法。
void PrintUsage(const char* program) {
  std::cout << "Usage: " << program << " [--output-dir <dir>]\n"
            << "  --output-dir <dir>  可视化 CSV 输出目录（默认 " << kDefaultOutputDir << "）\n"
            << "  运行后用 build_viewer.py 构建交互式 HTML 查看器：\n"
            << "    python3 examples/behavior_layer/build_viewer.py <dir>\n";
}

/// 加载三份会话配置（复用各域 config_loader 与 examples/configs/ 同源 JSON）。
bl::BehaviorLayerConfig LoadConfigs() {
  bl::BehaviorLayerConfig configs;
  std::string error;
  if (!examples::LoadArSessionConfigFromFile(SCENE_CONFIG_DIR "/airborne_radar.json",
                                              &configs.ar, &error)) {
    std::cerr << "Failed to load AR config: " << error << "\n";
    std::exit(1);
  }
  if (!examples::LoadEsrSessionConfigFromFile(SCENE_CONFIG_DIR "/electronic_warfare.json",
                                              &configs.esr, &error)) {
    std::cerr << "Failed to load ESR config: " << error << "\n";
    std::exit(1);
  }
  if (!examples::LoadEosSessionConfigFromFile(SCENE_CONFIG_DIR "/electro_optical.json",
                                              &configs.eos, &error)) {
    std::cerr << "Failed to load EOS config: " << error << "\n";
    std::exit(1);
  }
  // 跨会话时间对齐与视场适配（业务层调参，不改共享 JSON）：
  // - EOS 周期校验要求 dt ≤ 10/frame_rate_hz（帧率 30 → 上限 0.33 s），
  //   演示按 1 s/周期推进 → 帧率覆写为 10 Hz；
  // - 原配置为下视地面监视（扫描 az ±10°、俯仰中心 -48°），与空中目标
  //   场景不匹配 → 覆写为水平扫描（az ±40°，扫描速率 20°/s 保持覆盖节奏）。
  configs.eos.mission.frame_rate_hz = 10.0f;
  configs.eos.mission.scan_rate_deg_per_sec = 20.0f;
  configs.eos.mission.scan_start_az_deg = -40.0f;
  configs.eos.mission.scan_end_az_deg = 40.0f;
  configs.eos.mission.scan_center_el_deg = 0.0f;
  configs.eos.mission.boresight_depression_deg = 0.0f;
  return configs;
}

/// 目标脚本：3 个空中目标（正东前方 16-20 km，与平台同速东移）。
/// 方位（北偏东 90° = 正东）落在 EOS 扫描覆盖内（平台局部系 az 0 = 东，
/// 扫描 ±40°）；平台以 ~54-65 m/s 东飞 200 周期，目标 v_east 与其匹配 →
/// 相对斜距稳定在 EOS 探测距离窗（[10, 40] km）内，三传感器全程同见
/// 同一物理目标。
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
    {90.0, 16000.0, 60.0, 10.0, 520.0, 2.2f, 18.0f},
    {90.0, 18000.0, 62.0, -5.0, 540.0, 1.4f, 15.0f},
    {90.0, 20000.0, 58.0, 0.0, 560.0, 3.0f, 20.0f},
};

/// 目标 ECEF 运动学状态（三通道共享同一物理目标）。
struct TargetEcefState {
  oneq::coordinate::EcefPositionM position{};
  oneq::coordinate::EcefVelocityMps velocity{};
  float rcs{0.0f};
};

/// 目标脚本 → ECEF 状态（方位/距离经库内 ENU 偏移函数投影到 ECEF，速度经
/// ENU 速度函数投影；z 随 ENU 偏移投影（目标与平台同高基准，az=90° 时
/// north=0 → z=平台基准高度）。脚本为编译期合法常量，投影调用不会失败。
std::vector<TargetEcefState> MakeTargetStates(
    const oneq::coordinate::EcefPositionM& platform_ecef,
    const oneq::coordinate::LlaPositionDegM& platform_origin) {
  std::vector<TargetEcefState> states;
  states.reserve(3U);
  for (const auto& script : kTargetScript) {
    TargetEcefState state;
    oneq::coordinate::EnuPositionM offset;
    oneq::coordinate::EcefPositionM target;
    if (oneq::coordinate::TryBearingRangeToEnuOffset(script.azimuth_deg, script.range_m,
                                                     &offset) &&
        oneq::coordinate::TryEnuToEcef(offset, platform_origin, &target)) {
      state.position = target;
    }
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

/// AR 世界目标事实（ECEF 运动学 + RCS）。
std::vector<ar_session::ArTargetInput> MakeArTargetInputs(
    const std::vector<TargetEcefState>& states) {
  std::vector<ar_session::ArTargetInput> targets;
  targets.reserve(states.size());
  for (std::size_t i = 0U; i < states.size(); ++i) {
    ar_session::ArTargetInput target;
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

/// ESR 辐射源真值：与 AR 目标同一物理目标（脉冲列波形，供统计检测门限
/// 在 pfa=1e-6 下以多脉冲积分过检；噪声波形有效脉冲数为 1，无法过检）。
/// 三辐射源中心频率互异（9.5/10.0/10.5 GHz），保证 ESR 分选聚簇
/// 能稳定分离出 3 条假设航迹（同频会聚簇合并）。
std::vector<oneq::electromagnetics::RfSceneEmission> MakeEmitterTruths(
    const std::vector<TargetEcefState>& states, double window_start_time_s) {
  const double kCenterFrequencyHz[] = {9.5e9, 10.0e9, 10.5e9};
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

/// EOS 光学目标真值：同一物理目标（外观参数仿 electro_optical 示例）。
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

/// 目标 ECEF 欧拉推进（消费方世界模型，与 session_usage 一致）。
void AdvanceTargetStates(std::vector<TargetEcefState>& states, double dt_s) {
  for (auto& state : states) {
    state.position.x_m += state.velocity.x_mps * dt_s;
    state.position.y_m += state.velocity.y_mps * dt_s;
    state.position.z_m += state.velocity.z_mps * dt_s;
  }
}

/// 打印单周期摘要：三会话状态 + 平台飞行 + 融合态势（按通道构成）+ 命令帧。
void PrintCycleSummary(std::uint32_t cycle, const bl::BehaviorContext& context,
                       const bl::FleetStatusComponent& fleet, std::size_t waypoint_index,
                       std::size_t waypoint_count,
                       const bl::FusedSituationComponent& situation,
                       const bl::CommandFrameComponent& command) {
  const auto& ar = context.last_ar_result;
  const auto& esr = context.esr_last_result;
  const auto& eos = context.eos_last_result;
  const std::size_t ecm_obs_count =
      command.ecm_inputs.empty() ? 0U
                                  : command.ecm_inputs[0].sensor_observation_frame.observations.size();
  std::cout << "cycle=" << cycle
            << " plat[alt=" << fleet.position.altitude_m
            << " hdg=" << fleet.heading_deg << " spd=" << fleet.speed_mps
            << " wp=" << waypoint_index << "/" << waypoint_count << "]"
            << " ar[st=" << static_cast<int>(ar.status)
            << " tracks=" << ar.track_output_frame.tracks.size() << "]"
            << " esr[st=" << static_cast<int>(esr.status)
            << " hyp=" << esr.output_frame.emitter_output.hypotheses.size()
            << " obs=" << esr.output_frame.observation_output.observations.size() << "]"
            << " eos[st=" << static_cast<int>(eos.status)
            << " det=" << eos.output_frame.detections.size() << "]"
            << " fused=" << situation.targets.size()
            << " commands=" << command.ar_commands.size()
            << " ecm_obs=" << ecm_obs_count
            << " validation_error="
            << ((ar.has_validation_error || esr.has_validation_error || eos.has_validation_error)
                    ? "true"
                    : "false")
            << "\n";
  if (!situation.targets.empty()) {
    std::cout << "  fused:";
    for (const auto& target : situation.targets) {
      std::cout << " key=" << target.key << " conf=" << target.confidence << " ch[";
      for (const auto& channel : target.channels) {
        std::cout << channel.source_id << ":" << channel.sample_count << " ";
      }
      std::cout << "]";
    }
    std::cout << "\n";
  }
  for (const auto& cmd : command.ar_commands) {
    std::cout << "  command type=" << static_cast<int>(cmd.type)
              << " source=" << static_cast<int>(cmd.source) << "\n";
  }
}

/// 打印航路计划（重规划后调用一次）。
void PrintRoute(const bl::RoutePlanComponent& route) {
  std::cout << "route waypoints=" << route.route.size() << ":\n";
  for (std::size_t i = 0U; i < route.route.size(); ++i) {
    const auto& wp = route.route[i];
    std::cout << "  [" << i << "] lat=" << wp.position.latitude_deg
              << " lon=" << wp.position.longitude_deg << " alt=" << wp.position.altitude_m
              << " speed=" << wp.speed_mps << " radius=" << wp.radius_m << "\n";
  }
}

}  // namespace

int main(int argc, char* argv[]) {
  // 命令行参数：--output-dir <dir> 覆盖可视化 CSV 输出目录（默认 /tmp/behavior_layer_viz）。
  std::string output_dir = kDefaultOutputDir;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--output-dir") {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for --output-dir\n";
        PrintUsage(argv[0]);
        return 1;
      }
      output_dir = argv[++i];
    } else if (arg == "--help" || arg == "-h") {
      PrintUsage(argv[0]);
      return 0;
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      PrintUsage(argv[0]);
      return 1;
    }
  }

  entt::registry registry;
  const entt::entity lead = bl::AssembleBehaviorLayer(registry, LoadConfigs());
  auto situation_observer = bl::MakeSituationObserver(registry);

  auto& context = registry.ctx().get<bl::BehaviorContext>();

  // 可视化记录器：FD 初始化成功 = JSBSim 真实飞行，否则运动学回退（model 列区分）。
  bl::VizRecorder recorder(output_dir, bl::GetFlightDynamics(registry) != nullptr);

  // 世界真值脚本：以平台初始 ECEF 与 LLA 为基准（消费方场景编排）。
  oneq::coordinate::EcefPositionM platform_ecef;
  if (!oneq::coordinate::TryLlaToEcef(
          registry.get<bl::FleetStatusComponent>(lead).position, &platform_ecef)) {
    std::cerr << "Invalid platform LLA\n";
    return 1;
  }
  std::vector<TargetEcefState> target_states = MakeTargetStates(
      platform_ecef, registry.get<bl::FleetStatusComponent>(lead).position);

  std::uint32_t validation_error_count = 0U;
  bool route_printed = false;
  for (std::uint32_t cycle = 1U; cycle <= kNumCycles; ++cycle) {
    // 消费方每周期注入三套世界真值（ESR 辐射源波形窗口随周期推进）。
    context.world_targets = MakeArTargetInputs(target_states);
    context.emitter_truths = MakeEmitterTruths(target_states, static_cast<double>(cycle));
    context.optical_targets = MakeOpticalTargets(target_states);

    bl::StepBehaviorLayer(registry);

    const auto& situation = registry.get<bl::FusedSituationComponent>(lead);
    const auto& route = registry.get<bl::RoutePlanComponent>(lead);
    const auto& command = registry.get<bl::CommandFrameComponent>(lead);

    if (route.version > 0U && !route_printed) {
      PrintRoute(route);
      recorder.RecordRoute(route);  // 可视化：航路计划（一次）
      route_printed = true;
    }
    const auto& fleet = registry.get<bl::FleetStatusComponent>(lead);
    PrintCycleSummary(cycle, context, fleet, route.next_index, route.route.size(), situation,
                      command);

    // 可视化数据导出：本周期平台/目标真值/三传感器/融合态势 + 航点完成事件增量。
    std::vector<bl::TruthTargetRow> truth_rows;
    truth_rows.reserve(target_states.size());
    for (std::size_t i = 0U; i < target_states.size(); ++i) {
      bl::TruthTargetRow row;
      row.target_id = 1001U + i;
      row.position = target_states[i].position;
      row.rcs = target_states[i].rcs;
      truth_rows.push_back(row);
    }
    const double t_sec = static_cast<double>(cycle) * bl::kBehaviorDtSec;
    recorder.RecordCycle(cycle, t_sec, fleet, route, situation, context.last_ar_result,
                         context.esr_last_result, context.eos_last_result, truth_rows);
    recorder.RecordWaypointEvents(bl::CollectWaypointEvents(registry));

    // 事件触发报告（entt::observer）：新目标/消失目标（报告节奏属业务层）。
    for (const auto entity : *situation_observer) {
      const auto& observed = registry.get<bl::FusedSituationComponent>(entity);
      if (observed.new_target_count > 0U || observed.lost_target_count > 0U) {
        std::cout << "  event: new=" << observed.new_target_count
                  << " lost=" << observed.lost_target_count << "\n";
      }
    }
    situation_observer->clear();

    // AR 决策观测（去真值化输出）可供决策参考，本示例仅透出批号。
    if (context.last_ar_result.has_decision_observation) {
      std::cout << "  decision_observation batch="
                << context.last_ar_result.decision_observation.input_frame.batch_id << "\n";
    }

    if (context.last_ar_result.has_validation_error ||
        context.esr_last_result.has_validation_error ||
        context.eos_last_result.has_validation_error) {
      ++validation_error_count;
    }

    // 消费方世界模型推进（在 Step 之后，与 session_usage 周期语义一致）；
    // 平台推进与传感器实体同步由 flight_system 在 StepBehaviorLayer 内完成。
    AdvanceTargetStates(target_states, bl::kBehaviorDtSec);
  }

  recorder.Flush();  // 确保全部 CSV 落盘后再打印摘要
  std::cout << "\n=== Behavior Layer Summary ===\n"
            << "cycles=" << kNumCycles << " validation_errors=" << validation_error_count << "\n"
            << "visualization data -> " << recorder.output_dir()
            << " (platform_track/target_truth/ar_tracks/eos_detections/esr_hypotheses/"
               "fused_tracks/route_plan/waypoint_events.csv)\n"
            << "build interactive viewer: python3 examples/behavior_layer/build_viewer.py "
            << recorder.output_dir() << "\n";
  return validation_error_count == 0U ? 0 : 1;
}
