/**
 * @file assembly.cpp
 * @brief 行为层装配实现。
 */

#include "assembly.h"

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "1q/coordinate/types.h"
#include "components.h"
#include "systems.h"

namespace behavior_layer {

namespace {

/// 演示编队规模：长机 + 2 架僚机（层级显式注入，无发现机制）。
constexpr std::uint32_t kFleetSize = 3U;

/// 演示平台初始状态（中纬度某空域，约对应 examples/airborne_radar scene 场景）。
constexpr double kDemoLatDeg = 30.0;
constexpr double kDemoLonDeg = 120.0;
constexpr double kDemoAltM = 3000.0;
constexpr double kDemoHeadingDeg = 90.0;  // 正东
constexpr double kDemoSpeedMps = 100.0;

/// 演示覆盖区域多边形：平台东侧目标带（约 15~34 km 纵向，20 km 横向）。
std::vector<oneq::coordinate::LlaPositionDegM> MakeDemoPolygon() {
  std::vector<oneq::coordinate::LlaPositionDegM> vertices;
  vertices.push_back({kDemoLatDeg - 0.08, kDemoLonDeg + 0.15, kDemoAltM});
  vertices.push_back({kDemoLatDeg - 0.08, kDemoLonDeg + 0.35, kDemoAltM});
  vertices.push_back({kDemoLatDeg + 0.08, kDemoLonDeg + 0.35, kDemoAltM});
  vertices.push_back({kDemoLatDeg + 0.08, kDemoLonDeg + 0.15, kDemoAltM});
  return vertices;
}

}  // namespace

entt::entity AssembleBehaviorLayer(entt::registry& registry,
                                   const airborne_radar::config::ArSessionConfig& config) {
  // 1. 运行时上下文：AR 会话 + 融合引擎 + 规划器（生命周期归装配层）。
  BehaviorContext context;
  context.ar_session = std::make_unique<airborne_radar::session::ArSession>(
      airborne_radar::session::ArSession::Create(config));
  fusion::FusionConfig fusion_config;
  fusion_config.position_radius_m = 1000.0;
  fusion_config.bearing_beamwidth_deg = 5.0;
  fusion_config.feature_threshold = 0.0;  // 不启用特征门（AR 轨迹无特征向量）
  fusion_config.window_size = 10U;
  fusion_config.max_missed_cycles = 5U;
  // source_weights 留空：AR 源（source_id=1）按缺省权重 1.0 计。
  context.fusion_engine = std::make_unique<fusion::FusionEngine>(fusion_config);
  registry.ctx().emplace<BehaviorContext>(std::move(context));

  // 2. 长机实体先行创建（僚机需要在装配时引用其上级句柄）。
  const auto lead = registry.create();

  // 3. 僚机实体：任务/编队状态/航路（侦察与命令由长机承载，单域 AR）。
  std::vector<entt::entity> wingmen;
  for (std::uint32_t i = 0U; i < kFleetSize - 1U; ++i) {
    const auto wing = registry.create();
    FleetStatusComponent fleet;
    fleet.platform_entity_id = 2U + i;
    fleet.position = {kDemoLatDeg + (i == 1U ? 0.004 : -0.004), kDemoLonDeg, kDemoAltM};
    fleet.heading_deg = kDemoHeadingDeg;
    fleet.speed_mps = kDemoSpeedMps;
    registry.emplace<FleetStatusComponent>(wing, fleet);
    registry.emplace<RoutePlanComponent>(wing, RoutePlanComponent{});
    TaskingComponent tasking;
    tasking.role = Role::kWingman;
    tasking.superior = lead;  // 有上级：层级显式表达（冻结契约 §5）
    registry.emplace<TaskingComponent>(wing, tasking);
    wingmen.push_back(wing);
  }

  // 4. 长机实体组件栈（任务/侦察/航路/融合/命令帧）。

  FleetStatusComponent lead_fleet;
  lead_fleet.platform_entity_id = 1U;
  lead_fleet.position = {kDemoLatDeg, kDemoLonDeg, kDemoAltM};
  lead_fleet.heading_deg = kDemoHeadingDeg;
  lead_fleet.speed_mps = kDemoSpeedMps;
  registry.emplace<FleetStatusComponent>(lead, lead_fleet);

  SensorObservationComponent sensor;
  sensor.source_id = 1U;  // AR 源通道（与融合配置 source_weights 索引语义一致）
  registry.emplace<SensorObservationComponent>(lead, sensor);

  registry.emplace<RoutePlanComponent>(lead, RoutePlanComponent{});
  registry.emplace<FusedSituationComponent>(lead, FusedSituationComponent{});
  registry.emplace<CommandFrameComponent>(lead, CommandFrameComponent{});

  TaskingComponent tasking;
  tasking.role = Role::kLead;
  tasking.subordinates = wingmen;
  tasking.region.kind = navigation::CoverageAreaKind::kPolygon;
  tasking.region.polygon.vertices = MakeDemoPolygon();
  tasking.region_config.mode = navigation::CoverageMode::kScan;
  tasking.region_config.scan_heading_deg = 0.0;
  tasking.region_config.scan_spacing_m = 200.0;
  tasking.region_config.altitude_m = kDemoAltM;
  tasking.region_config.speed_mps = kDemoSpeedMps;
  tasking.region_config.arrival_radius_m = 50.0;
  registry.emplace<TaskingComponent>(lead, tasking);

  return lead;
}

void StepBehaviorLayer(entt::registry& registry) {
  auto& context = registry.ctx().get<BehaviorContext>();
  ++context.cycle;
  // 周期调用序对齐 session Step 语义（冻结契约 §5）。
  recon_system(registry);
  maneuver_system(registry);
  jam_system(registry);
  decision_system(registry);
}

std::unique_ptr<entt::observer> MakeSituationObserver(entt::registry& registry) {
  return std::make_unique<entt::observer>(
      registry, entt::collector.update<FusedSituationComponent>());
}

}  // namespace behavior_layer
