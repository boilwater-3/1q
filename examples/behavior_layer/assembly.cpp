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

/// 演示平台初始状态（中纬度某空域；700 m 高度使 EOS 探测距离窗
/// （alt/sin(视轴±半视场)）覆盖 10-40 km，与目标斜距匹配）。
constexpr double kDemoLatDeg = 30.0;
constexpr double kDemoLonDeg = 120.0;
constexpr double kDemoAltM = 700.0;
constexpr double kDemoHeadingDeg = 90.0;  // 正东
constexpr double kDemoSpeedMps = 100.0;

/// 构造同一演示平台的编队状态（长机与三传感器实体共享，由消费方每周期同步）。
FleetStatusComponent MakePlatformFleet() {
  FleetStatusComponent fleet;
  fleet.platform_entity_id = 1U;
  fleet.position = {kDemoLatDeg, kDemoLonDeg, kDemoAltM};
  fleet.heading_deg = kDemoHeadingDeg;
  fleet.speed_mps = kDemoSpeedMps;
  return fleet;
}

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
                                   const BehaviorLayerConfig& config) {
  // 1. 运行时上下文：三会话 + 融合引擎 + 规划器（生命周期归装配层）。
  BehaviorContext context;
  context.ar_session = std::make_unique<airborne_radar::session::ArSession>(
      airborne_radar::session::ArSession::Create(config.ar));
  context.esr_session = std::make_unique<electronic_surveillance_radar::session::EsrSession>(
      electronic_surveillance_radar::session::EsrSession::Create(config.esr));
  context.eos_session = std::make_unique<electro_optical_sensor::session::EosSession>(
      electro_optical_sensor::session::EosSession::Create(config.eos));
  fusion::FusionConfig fusion_config;
  fusion_config.position_radius_m = 1000.0;
  // 方位相干门限放宽到 8°：ESR 假设方位含平滑滞差、EOS 探测含扫描中心
  // 残差，同物理目标的跨源方位差实测可达 4-6°（业务层调参，非库内标准）。
  fusion_config.bearing_beamwidth_deg = 8.0;
  fusion_config.feature_threshold = 0.0;  // 不启用特征门（异构特征维度不匹配时引擎本就不约束）
  fusion_config.window_size = 10U;
  fusion_config.max_missed_cycles = 5U;
  // 源权重按 source_id 索引：AR=1.0、ESR=0.8、EOS=0.6（索引 0 未用）。
  fusion_config.source_weights = {0.0, 1.0, 0.8, 0.6};
  context.fusion_engine = std::make_unique<fusion::FusionEngine>(fusion_config);
  registry.ctx().emplace<BehaviorContext>(std::move(context));

  // 2. 长机实体先行创建（僚机需要在装配时引用其上级句柄）。
  const auto lead = registry.create();

  // 3. 僚机实体：任务/编队状态/航路（侦察与命令由长机与传感器实体承载，单平台）。
  std::vector<entt::entity> wingmen;
  for (std::uint32_t i = 0U; i < kFleetSize - 1U; ++i) {
    const auto wing = registry.create();
    FleetStatusComponent fleet = MakePlatformFleet();
    fleet.position.latitude_deg += (i == 1U ? 0.004 : -0.004);
    registry.emplace<FleetStatusComponent>(wing, fleet);
    registry.emplace<RoutePlanComponent>(wing, RoutePlanComponent{});
    TaskingComponent tasking;
    tasking.role = Role::kWingman;
    tasking.superior = lead;  // 有上级：层级显式表达（冻结契约 §5）
    registry.emplace<TaskingComponent>(wing, tasking);
    wingmen.push_back(wing);
  }

  // 4. 传感器实体：每传感器一个实体，复用 SensorObservationComponent +
  //    FleetStatusComponent（source_id 区分源通道，组件类型零新增）。
  const auto ar_sensor = registry.create();
  {
    SensorObservationComponent sensor;
    sensor.source_id = kArSourceId;
    registry.emplace<SensorObservationComponent>(ar_sensor, sensor);
  }
  registry.emplace<FleetStatusComponent>(ar_sensor, MakePlatformFleet());

  const auto esr_sensor = registry.create();
  {
    SensorObservationComponent sensor;
    sensor.source_id = kEsrSourceId;
    registry.emplace<SensorObservationComponent>(esr_sensor, sensor);
  }
  registry.emplace<FleetStatusComponent>(esr_sensor, MakePlatformFleet());

  const auto eos_sensor = registry.create();
  {
    SensorObservationComponent sensor;
    sensor.source_id = kEosSourceId;
    registry.emplace<SensorObservationComponent>(eos_sensor, sensor);
  }
  registry.emplace<FleetStatusComponent>(eos_sensor, MakePlatformFleet());

  // 5. 长机实体组件栈（任务/航路/融合/命令帧；平台位姿供 jam 取用）。
  registry.emplace<FleetStatusComponent>(lead, MakePlatformFleet());
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
