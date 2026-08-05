/**
 * @file behavior_layer_demo.cpp
 * @brief 行为层参考实现主程序（EnTT registry + 每周期系统调用序）。
 *
 * 单域（AR）端到端全链演示：
 *   AR 会话周期输出 → recon 适配 → fusion 引擎 → 融合态势
 *   → maneuver 规划航路 → jam 构造 ECM 帧 → decision 产出命令帧
 *   → 消费方读取命令帧驱动执行面（flight_dynamic/ESR 等接线位见 README.md）。
 *
 * 事件报告节奏：每周期迭代 entt::observer（新目标/消失事件），
 * 命令 = 写 CommandFrameComponent（无全局事件总线，冻结契约 §5）。
 */

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <entt/entt.hpp>

#include "1q/airborne_radar/session/ArSession.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/types.h"
#include "1q/fusion/FusedTarget.h"
#include "1q/navigation/RoutePoint.h"
#include "config_loader.h"
#include "assembly.h"
#include "components.h"

namespace ar = airborne_radar;
namespace ar_session = airborne_radar::session;
namespace bl = behavior_layer;

namespace {

constexpr std::uint32_t kNumCycles = 50U;
constexpr double kDtSec = 1.0;
constexpr double kEarthRadiusM = 6371000.0;

/// 加载 AR 会话配置（复用 AR 域 config_loader 与 examples/configs/ 同源 JSON）。
ar::config::ArSessionConfig LoadConfig() {
  ar::config::ArSessionConfig config;
  std::string error;
  if (!examples::LoadArSessionConfigFromFile(SCENE_CONFIG_DIR "/airborne_radar.json",
                                              &config, &error)) {
    std::cerr << "Failed to load AR config: " << error << "\n";
    std::exit(1);
  }
  return config;
}

/// 脚本化世界目标事实：3 个空中目标（ECEF 位置/速度，仿 airborne_radar scene 数值）。
std::vector<ar_session::ArTargetInput> MakeScriptedTargets(
    const oneq::coordinate::EcefPositionM& platform_ecef) {
  const struct {
    double dx_m, dy_m, dz_m;
    double vx_mps, vy_mps, vz_mps;
    float rcs;
  } kScript[] = {
      {18000.0, 2500.0, 1200.0, -120.0, 8.0, 0.0, 2.2f},
      {24000.0, -4000.0, 2000.0, -90.0, -12.0, 0.0, 1.4f},
      {30000.0, 1000.0, 1500.0, -150.0, 0.0, -5.0, 3.0f},
  };
  std::vector<ar_session::ArTargetInput> targets;
  targets.reserve(3U);
  std::uint64_t target_id = 1001U;
  for (const auto& s : kScript) {
    ar_session::ArTargetInput target;
    target.target_id = target_id++;
    target.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
    target.kinematics.position_ecef_m = {platform_ecef.x_m + s.dx_m,
                                         platform_ecef.y_m + s.dy_m,
                                         platform_ecef.z_m + s.dz_m};
    target.kinematics.velocity_mps = {s.vx_mps, s.vy_mps, s.vz_mps};
    target.rcs = s.rcs;
    target.swerling_type = 0;
    targets.push_back(target);
  }
  return targets;
}

/// 平台 LLA 近似推进（消费方世界模型：heading/speed → 经度/纬度变化率）。
void AdvancePlatform(bl::FleetStatusComponent& fleet, double dt_s) {
  const double heading_rad = fleet.heading_deg * 3.14159265358979323846 / 180.0;
  const double vlat_rad_s = fleet.speed_mps * std::cos(heading_rad) / kEarthRadiusM;
  const double vlon_rad_s =
      fleet.speed_mps * std::sin(heading_rad) /
      (kEarthRadiusM * std::cos(fleet.position.latitude_deg * 3.14159265358979323846 / 180.0));
  const double rad_to_deg = 180.0 / 3.14159265358979323846;
  fleet.position.latitude_deg += vlat_rad_s * dt_s * rad_to_deg;
  fleet.position.longitude_deg += vlon_rad_s * dt_s * rad_to_deg;
}

/// 目标 ECEF 欧拉推进（消费方世界模型，与 session_usage 一致）。
void AdvanceTargets(std::vector<ar_session::ArTargetInput>& targets, double dt_s) {
  for (auto& target : targets) {
    target.kinematics.position_ecef_m.x_m += target.kinematics.velocity_mps.x_mps * dt_s;
    target.kinematics.position_ecef_m.y_m += target.kinematics.velocity_mps.y_mps * dt_s;
    target.kinematics.position_ecef_m.z_m += target.kinematics.velocity_mps.z_mps * dt_s;
  }
}

/// 打印单周期摘要：AR 结果 + 融合态势 + 命令帧。
void PrintCycleSummary(std::uint32_t cycle, const bl::BehaviorContext& context,
                       const bl::FusedSituationComponent& situation,
                       const bl::CommandFrameComponent& command) {
  const auto& result = context.last_ar_result;
  std::cout << "cycle=" << cycle << " ar_status=" << static_cast<int>(result.status)
            << " tracks=" << result.track_output_frame.tracks.size()
            << " fused=" << situation.targets.size()
            << " commands=" << command.ar_commands.size()
            << " ecm_frames=" << command.ecm_inputs.size()
            << " validation_error=" << (result.has_validation_error ? "true" : "false") << "\n";
  if (!situation.targets.empty()) {
    std::cout << "  fused:";
    for (const auto& target : situation.targets) {
      std::cout << " key=" << target.key << " conf=" << target.confidence;
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

int main() {
  entt::registry registry;
  const entt::entity lead = bl::AssembleBehaviorLayer(registry, LoadConfig());
  auto situation_observer = bl::MakeSituationObserver(registry);

  auto& context = registry.ctx().get<bl::BehaviorContext>();

  // 世界目标事实：以平台初始 ECEF 为基准的脚本化目标（消费方场景编排）。
  oneq::coordinate::EcefPositionM platform_ecef;
  if (!oneq::coordinate::TryLlaToEcef(
          registry.get<bl::FleetStatusComponent>(lead).position, &platform_ecef)) {
    std::cerr << "Invalid platform LLA\n";
    return 1;
  }
  context.world_targets = MakeScriptedTargets(platform_ecef);

  std::uint32_t validation_error_count = 0U;
  bool route_printed = false;
  for (std::uint32_t cycle = 1U; cycle <= kNumCycles; ++cycle) {
    bl::StepBehaviorLayer(registry);

    const auto& situation = registry.get<bl::FusedSituationComponent>(lead);
    const auto& route = registry.get<bl::RoutePlanComponent>(lead);
    const auto& command = registry.get<bl::CommandFrameComponent>(lead);

    if (route.version > 0U && !route_printed) {
      PrintRoute(route);
      route_printed = true;
    }
    PrintCycleSummary(cycle, context, situation, command);

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

    if (context.last_ar_result.has_validation_error) {
      ++validation_error_count;
    }

    // 消费方世界模型推进（在 Step 之后，与 session_usage 周期语义一致）。
    AdvancePlatform(registry.get<bl::FleetStatusComponent>(lead), kDtSec);
    AdvanceTargets(context.world_targets, kDtSec);
  }

  std::cout << "\n=== Behavior Layer Summary ===\n"
            << "cycles=" << kNumCycles << " validation_errors=" << validation_error_count << "\n";
  return validation_error_count == 0U ? 0 : 1;
}
