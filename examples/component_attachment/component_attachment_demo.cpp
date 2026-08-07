/**
 * @file component_attachment_demo.cpp
 * @brief 自定义实体-组件示例主程序（第二种示例模式）。
 *
 * 与 behavior_layer（EnTT ECS 开源库模式）对照：本示例不依赖 EnTT，
 * 采用自定义实体-组件框架（core/）：组件基类 → 各模块组件（飞行 / AR /
 * ESR / EOS / SBIRS / SAR / 融合）继承并挂载到平台实体，World 按挂载序
 * 周期步进；组件间事件通信使用 C++ 常见开源事件库 Boost.Signals2
 * （core/signals.h，零自定义分发层）。
 *
 * 数据流（单平台实体，挂载序 = 步进序）：
 *   Flight（推进位姿）→ AR / ESR / EOS / SBIRS（读 Flight 状态驱动会话，
 *   探测存自身组件）→ SAR（图像产品，不入融合）→ Fusion（聚合四传感器
 *   探测，一次 Update）→ 事件日志。
 *   事件：AR 首确认/失跟、ESR 假设、EOS 探测、SBIRS 探测、SAR 产品、
 *   融合更新、航点到达、决策指令（高置信威胁 → 指令下发，事件链演示）。
 *
 * 主程序只做装配与编排（世界/实体/会话创建、周期循环、查询演示、冒烟
 * 断言）；世界模型真值脚本见 scene_script.h，配置加载见 demo_config.h，
 * 输出落盘与事件消费见 demo_output.h。
 */

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "1q/airborne_radar/session/ArSession.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/types.h"
#include "1q/electro_optical_sensor/session/EosSession.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"
#include "1q/fusion/FusionEngine.h"
#include "1q/fusion/FusedTarget.h"
#include "1q/navigation/RoutePoint.h"
#include "1q/sar/session/SarSession.h"
#include "1q/sbirs_sensor/session/SbirsSession.h"

#include "components/ar_sensor_component.h"
#include "components/demo_log.h"
#include "components/eos_sensor_component.h"
#include "components/esr_sensor_component.h"
#include "components/flight_component.h"
#include "components/fusion_component.h"
#include "components/sar_sensor_component.h"
#include "components/sbirs_sensor_component.h"
#include "components/scene_types.h"
#include "core/world.h"
#include "demo_config.h"
#include "demo_output.h"
#include "scene_script.h"

namespace ca = component_attachment;
namespace demo = component_attachment::demo;

int main(int argc, char* argv[]) {
  // 命令行参数：--cycles / --output-dir。
  std::uint32_t num_cycles = demo::kNumCycles;
  std::string output_dir = demo::kDefaultOutputDir;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--cycles") {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for --cycles\n";
        demo::PrintUsage(argv[0]);
        return 1;
      }
      num_cycles = static_cast<std::uint32_t>(std::atoi(argv[++i]));
    } else if (arg == "--output-dir") {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for --output-dir\n";
        demo::PrintUsage(argv[0]);
        return 1;
      }
      output_dir = argv[++i];
    } else if (arg == "--help" || arg == "-h") {
      demo::PrintUsage(argv[0]);
      return 0;
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      demo::PrintUsage(argv[0]);
      return 1;
    }
  }
  if (num_cycles == 0U) {
    std::cerr << "Invalid --cycles value: must be > 0\n";
    return 1;
  }
  std::filesystem::create_directories(output_dir);

  // 集成端日志初始化（两个日志模块的输出文件：库日志 1q_library.log + 集成
  // 端日志 integration.log；见 components/demo_log.h）。须在会话创建之前调用：
  // 库内 PROJECT_LOG_* 走 spdlog 默认 logger，装配后库日志即入文件而非 stdout。
  demo::InitIntegrationLog(output_dir);

  // 装配：共享场景状态 → World → 平台实体 + 6 组件（挂载序 = 步进序）。
  ca::DemoSceneState scene;
  ca::World world(scene);
  ca::Entity& platform = world.CreateEntity("platform");

  // 平台初始状态：机场地面（alt 0，六自由度机动从起飞开始）→ 巡航高度
  // kCruiseAltitudeM，沿正东 3 个航点（间距约 5 km）。
  const oneq::coordinate::LlaPositionDegM platform_origin{30.0, 120.0, 0.0};
  std::vector<navigation::RoutePoint> route;
  for (int i = 1; i <= 3; ++i) {
    navigation::RoutePoint wp;
    wp.position.latitude_deg = 30.0;
    wp.position.longitude_deg = 120.0 + 0.05 * i;
    wp.position.altitude_m = demo::kCruiseAltitudeM;
    wp.speed_mps = demo::kCruiseSpeedMps;
    wp.radius_m = 500.0;
    route.push_back(wp);
  }
  platform.Attach(std::make_unique<ca::FlightComponent>(platform_origin, 90.0,
                                                        demo::kCruiseSpeedMps,
                                                        demo::kCruiseAltitudeM,
                                                        std::move(route)));

  const demo::ComponentAttachmentConfigs configs = demo::LoadConfigs();
  platform.Attach(std::make_unique<ca::ArSensorComponent>(
      airborne_radar::session::ArSession::Create(configs.ar)));
  platform.Attach(std::make_unique<ca::EsrSensorComponent>(
      electronic_surveillance_radar::session::EsrSession::Create(configs.esr)));
  platform.Attach(std::make_unique<ca::EosSensorComponent>(
      electro_optical_sensor::session::EosSession::Create(configs.eos)));
  platform.Attach(std::make_unique<ca::SbirsSensorComponent>(
      sbirs_sensor::session::SbirsSession::Create(configs.sbirs)));
  platform.Attach(std::make_unique<ca::SarSensorComponent>(
      sar::session::SarSession::Create(configs.sar)));

  fusion::FusionConfig fusion_config;
  fusion_config.position_radius_m = 1000.0;
  // 方位相干门限放宽到 8°：ESR 假设方位含平滑滞差、EOS 探测含扫描中心
  // 残差，同物理目标的跨源方位差实测可达 4-6°（业务层调参，非库内标准）。
  fusion_config.bearing_beamwidth_deg = 8.0;
  fusion_config.feature_threshold = 0.0;  // 不启用特征门
  fusion_config.window_size = 10U;
  fusion_config.max_missed_cycles = 5U;
  // 源权重按 source_id 索引：AR=1.0、ESR=0.8、EOS=0.6、SBIRS=0.5（索引 0 未用）。
  fusion_config.source_weights = {0.0, 1.0, 0.8, 0.6, 0.5};
  platform.Attach(std::make_unique<ca::FusionComponent>(
      std::make_unique<fusion::FusionEngine>(fusion_config)));

  // 事件接线：决策监听器（订阅融合信号）+ 周期落盘输出（平台轨迹 CSV；
  // 集成端日志已由 InitIntegrationLog 装配，组件在 Step 内直写视图与事件）。
  demo::DecisionListener decision(world);
  demo::DemoOutputs outputs(output_dir);

  oneq::coordinate::EcefPositionM platform_ecef;
  if (!oneq::coordinate::TryLlaToEcef(platform_origin, &platform_ecef)) {
    std::cerr << "Invalid platform LLA\n";
    return 1;
  }
  std::vector<demo::TargetEcefState> target_states =
      demo::MakeTargetStates(platform_ecef, platform_origin);

  std::size_t max_fused_targets = 0U;
  // 天基平台（卫星）位置：凝视模式，固定于目标群中心正上方 +500 km
  // （ECEF z 轴），目标始终位于星下点附近（SBIRS az/el 为 ECEF 极坐标，
  // 全向扫描 span 360° + 下视 el −90° 覆盖；消费方每周期注入世界模型）。
  constexpr double kSbirsSatelliteAltitudeM = 500000.0;
  for (std::uint32_t cycle = 1U; cycle <= num_cycles; ++cycle) {
    // 消费方每周期注入共享场景状态（周期号/时间/四通道世界真值）。
    scene.cycle = cycle;
    scene.t_sec = static_cast<double>(cycle) * demo::kDtSec;
    scene.ar_targets = demo::MakeArTargetInputs(target_states);
    scene.emitters = demo::MakeEmitterTruths(target_states, scene.t_sec);
    scene.optical_targets = demo::MakeOpticalTargets(target_states);
    scene.sbirs_targets = demo::MakeSbirsTargetInputs(target_states);
    scene.sar_point_targets = demo::MakeSarPointTargets(target_states);
    scene.sbirs_satellite_position_ecef_m.x =
        0.5 * (target_states[0].position.x_m + target_states[1].position.x_m);
    scene.sbirs_satellite_position_ecef_m.y =
        0.5 * (target_states[0].position.y_m + target_states[1].position.y_m);
    scene.sbirs_satellite_position_ecef_m.z =
        0.5 * (target_states[0].position.z_m + target_states[1].position.z_m) +
        kSbirsSatelliteAltitudeM;

    world.Step(demo::kDtSec);  // 按挂载序步进：Flight → AR → ESR → EOS → SBIRS → SAR → Fusion

    // 周期摘要 + 平台轨迹/调试视图落盘。
    const auto* flight = platform.Find<ca::FlightComponent>();
    const auto* fusion = platform.Find<ca::FusionComponent>();
    max_fused_targets = std::max(max_fused_targets, fusion->targets().size());
    std::cout << "cycle=" << cycle
              << " plat[alt=" << flight->position().altitude_m
              << " hdg=" << flight->heading_deg() << " spd=" << flight->speed_mps()
              << " wp=" << flight->next_waypoint_index() << "/" << flight->route().size() << "]"
              << " fused=" << fusion->targets().size() << "\n";
    outputs.RecordPlatformRow(cycle, scene.t_sec, *flight);
    // 调试视图落盘由各传感器组件在 Step 内直写（取视图 → 人读摘要行 → 集成
    // 端日志；见 components/demo_log.h 的 CA_LOG_VIEW），主程序不再收拢。

    // 消费方世界模型推进（在 Step 之后，与 behavior_layer 周期语义一致）。
    demo::AdvanceTargetStates(target_states, demo::kDtSec);
  }

  demo::FlushIntegrationLog();
  outputs.Flush();
  std::cout << "\n=== Component Attachment Summary ===\n"
            << "cycles=" << num_cycles
            << " entities=" << world.entity_count()
            << " components=" << platform.component_count()
            << " events=" << demo::EventCount()
            << " sbirs_events=" << demo::SbirsEventCount()
            << " sar_products=" << demo::SarProductEventCount()
            << " command_issued=" << (decision.issued() ? "true" : "false")
            << " ar_views=" << demo::ArViewCount()
            << " eos_views=" << demo::EosViewCount()
            << " sbirs_views=" << demo::SbirsViewCount() << "\n"
            << "log output -> " << output_dir
            << " (integration.log / 1q_library.log / platform_track.csv)\n";

  // 外置查询演示：按实体名/类型查找平台实体，读取各传感器开关机与当前扫描
  // 方位（查询逻辑 = 组件 const getter；外部系统选定实体后按名/ID 拉取
  // 最新快照即可）。AR/SAR 无扫描方位语义，不提供角度字段。
  const auto* ar = platform.Find<ca::ArSensorComponent>();
  const auto* esr = platform.Find<ca::EsrSensorComponent>();
  const auto* eos = platform.Find<ca::EosSensorComponent>();
  const auto* sbirs = platform.Find<ca::SbirsSensorComponent>();
  const auto* sar = platform.Find<ca::SarSensorComponent>();
  std::cout << "\n=== Platform Sensor States (entity query) ===\n"
            << "  ar    powered=" << (ar->powered_on() ? "on" : "off") << "\n"
            << "  esr   powered=" << (esr->powered_on() ? "on" : "off")
            << " scan_az=" << esr->scan_azimuth_deg() << " deg\n"
            << "  eos   powered=" << (eos->powered_on() ? "on" : "off")
            << " scan_az=" << eos->scan_azimuth_deg() << " deg\n"
            << "  sbirs powered=" << (sbirs->powered_on() ? "on" : "off")
            << " scan_az=" << sbirs->scan_azimuth_deg() << " deg\n"
            << "  sar   powered=" << (sar->powered_on() ? "on" : "off") << "\n";

  // 冒烟断言：端到端链路必须有产出（每周期平台状态事件、SBIRS 探测事件、
  // SAR 图像产品事件、至少一个融合目标、平台轨迹行数 = 周期数、AR/EOS/SBIRS
  // 调试视图行数各 = 周期数（组件每周期直写集成端日志）、五传感器全程开机），
  // 否则视为链路断裂（ctest 失败）。
  // 前置条件：视图行数由组件 Step 内计数，依赖每周期到达 CA_LOG_VIEW 调用点
  // （本场景 Flight 恒挂载、坐标适配恒成功；场景改动需同步检查该断言）。
  if (demo::EventCount() < num_cycles || demo::SbirsEventCount() == 0U ||
      demo::SarProductEventCount() == 0U || max_fused_targets == 0U ||
      outputs.platform_rows() != num_cycles || demo::ArViewCount() != num_cycles ||
      demo::EosViewCount() != num_cycles || demo::SbirsViewCount() != num_cycles ||
      !ar->powered_on() || !esr->powered_on() ||
      !eos->powered_on() || !sbirs->powered_on() || !sar->powered_on()) {
    std::cerr << "SMOKE FAILED: events=" << demo::EventCount()
              << " (>= " << num_cycles << " required), sbirs_events="
              << demo::SbirsEventCount() << " (>0 required), sar_products="
              << demo::SarProductEventCount() << " (>0 required), max_fused="
              << max_fused_targets << " (>0 required), platform_rows="
              << outputs.platform_rows() << " (== " << num_cycles << " required), "
              << "ar_views=" << demo::ArViewCount()
              << " (== " << num_cycles << " required), eos_views="
              << demo::EosViewCount() << " (== " << num_cycles << " required), sbirs_views="
              << demo::SbirsViewCount() << " (== " << num_cycles << " required), sensor_powered="
              << (ar->powered_on() && esr->powered_on() && eos->powered_on() &&
                  sbirs->powered_on() && sar->powered_on() ? "true" : "false")
              << " (all required)\n";
    return 1;
  }
  return 0;
}
