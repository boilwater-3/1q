/**
 * @file component_attachment_demo.cpp
 * @brief 自定义实体-组件示例主程序。
 *
 * 1. 装配：场景描述文件（--scene，默认 scenes/baseline_takeoff_east/baseline_takeoff_east.json）
 *    → SceneData → 共享场景状态 → World → 平台实体 + 8 组件（挂载序 =
 *    步进序 Flight → AR → ESR → EOS → SBIRS → SAR → Fusion），组件间事件
 *    通信用 Boost.Signals2（core/，零自定义分发层）；
 * 2. 编排：每周期注入四通道世界真值 → World::Step → 周期摘要与平台轨迹落盘；
 * 3. 收尾：集成端日志刷盘、按实体查询传感器状态演示、冒烟断言（下限来自
 *    场景文件 smoke 块）。
 * 场景数据见 scene_data.h，世界模型真值脚本见 scene_script.h，配置加载见
 * demo_config.h，输出落盘与事件消费见 demo_output.h。
 */

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// std::filesystem 为 C++17；VS2015（msvc 190）下同头文件提供 TR2 的
// std::experimental::filesystem，别名统一调用面。
#if defined(_MSC_VER) && _MSC_VER < 1910
#include <filesystem>
namespace demo_fs = std::experimental::filesystem;
#else
#include <filesystem>
namespace demo_fs = std::filesystem;
#endif

#include "1q/airborne_radar/session/ArSession.h"
#include "1q/coordinate/types.h"
#include "1q/electro_optical_sensor/session/EosSession.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"
#include "1q/fusion/FusionEngine.h"
#include "1q/fusion/FusedTarget.h"
#include "1q/sar/session/SarSession.h"
#include "1q/sbirs_sensor/session/SbirsSession.h"

#include "components/ar_sensor_component.h"
#include "components/ecm_sensor_component.h"
#include "logger/logger.h"
#include "components/eos_sensor_component.h"
#include "components/esr_sensor_component.h"
#include "components/flight_component.h"
#include "components/fusion_component.h"
#include "components/inference_component.h"
#include "components/rir_sensor_component.h"
#include "components/sar_sensor_component.h"
#include "components/sbirs_sensor_component.h"
#include "rf_world_broker.h"
#include "scene_types.h"
#include "components/threat_component.h"
#include "core/world.h"
#include "acceptance_timing.h"
#include "demo_config.h"
#include "demo_output.h"
#include "scene_data.h"
#include "scene_script.h"

namespace ca = component_attachment;
namespace demo = component_attachment::demo;

namespace {

/// 默认场景文件：CMake 注入的场景目录（examples/component_attachment/scenes/）。
constexpr char kDefaultSceneFile[] =
    CA_SCENE_DIR "/baseline_takeoff_east/baseline_takeoff_east.json";

}  // namespace

int main(int argc, char* argv[]) {
  // 命令行参数：--scene / --cycles / --view-every / --output-dir。
  std::string scene_path = kDefaultSceneFile;
  int cycles_override = -1;  // < 0 = 未指定，用场景文件值
  int view_every_override = -1;
  std::string output_dir = demo::kDefaultOutputDir;
  bool output_dir_overridden = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--scene") {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for --scene\n";
        demo::PrintUsage(argv[0]);
        return 1;
      }
      scene_path = argv[++i];
    } else if (arg == "--cycles") {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for --cycles\n";
        demo::PrintUsage(argv[0]);
        return 1;
      }
      cycles_override = std::atoi(argv[++i]);
    } else if (arg == "--view-every") {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for --view-every\n";
        demo::PrintUsage(argv[0]);
        return 1;
      }
      view_every_override = std::atoi(argv[++i]);
    } else if (arg == "--output-dir") {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for --output-dir\n";
        demo::PrintUsage(argv[0]);
        return 1;
      }
      output_dir = argv[++i];
      output_dir_overridden = true;
    } else if (arg == "--help" || arg == "-h") {
      demo::PrintUsage(argv[0]);
      return 0;
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      demo::PrintUsage(argv[0]);
      return 1;
    }
  }

  // 场景描述加载（平台/目标脚本/业务覆写/冒烟下限；缺省字段静默默认，
  // 必填几何字段缺失或 JSON 语法错误 → 报错退出）。
  demo::SceneData scene_data;
  std::string scene_error;
  if (!demo::LoadSceneData(scene_path.c_str(), &scene_data, &scene_error)) {
    std::cerr << "Failed to load scene '" << scene_path << "': " << scene_error << "\n";
    return 1;
  }
  const std::uint32_t num_cycles =
      cycles_override > 0 ? static_cast<std::uint32_t>(cycles_override) : scene_data.cycles;
  if (num_cycles == 0U) {
    std::cerr << "Invalid cycle count: must be > 0 (scene '" << scene_path
              << "' or --cycles)\n";
    return 1;
  }
  const std::uint32_t view_every =
      view_every_override > 0 ? static_cast<std::uint32_t>(view_every_override)
                              : scene_data.view_log_every_cycles;
  if (view_every == 0U) {
    std::cerr << "Invalid --view-every / view_log_every_cycles: must be > 0\n";
    return 1;
  }
  demo::SetViewLogEveryCycles(view_every);
  if (!output_dir_overridden) {
    output_dir = std::string(demo::kDefaultOutputDir) + "/" + demo::SceneSlugFromPath(scene_path);
  }
  demo_fs::create_directories(output_dir);

  // 集成端日志初始化（三个日志文件：库日志 1q_library.log + 集成端事件行
  // integration_events.log + 视图行 integration_views.log；见 logger/logger.h）。
  // 须在会话创建之前调用：库内 PROJECT_LOG_* 走 spdlog 默认 logger，装配后库
  // 日志即入文件而非 stdout。
  demo::InitIntegrationLog(output_dir);

  // 装配：共享场景状态 → World → 平台实体 + 8 组件（挂载序 = 步进序）。
  ca::DemoSceneState scene;
  ca::World world(scene);

  // 六会话配置：JSON 基线（examples/configs/）+ 场景业务覆写（EOS 扫描/SAR
  // 任务几何与链路，见 ApplySceneOverrides）。
  demo::ComponentAttachmentConfigs configs = demo::LoadConfigs();
  demo::ApplySceneOverrides(scene_data, &configs);
  if (scene_data.ecm_enabled) {
    // 同平台 ECM 发射链与 AR 接收链 co-site 隔离（演示层与跨域集成测试同量级）。
    configs.ar.hardware.receiver.co_site_paths.push_back(
        {ca::kDemoEcmTransmitterEquipmentId, configs.ar.hardware.receiver.equipment_id, 100.0});
  }

  // RIR 地基识别雷达站点（可选，场景 rir.enabled）：独立实体（固定站点，
  // S 波段识别雷达的物理摆放——非机载），会话配置经 LoadConfigs 从
  // examples/configs/remote_identification_radar.json 加载（识别库路径由
  // CMake 注入 CA_RIR_DATABASE_PATH 解析）。实体按创建序步进：站点先于平台
  // 创建，其本周期特征量测在平台融合组件 Step 时已就绪（同周期聚合，无跨
  // 周期滞后）。
  // 验收行（多模型并行加载）的装配段墙钟起点：从这里到威胁组件挂载完毕
  // 覆盖全部模块会话创建（库内无并行加载实现，真实加载状态由 example 层给出）。
  const std::chrono::steady_clock::time_point models_load_begin =
      std::chrono::steady_clock::now();
  ca::Entity* rir_site = nullptr;
  if (scene_data.rir_enabled) {
    rir_site = &world.CreateEntity(ca::kRirSiteEntityName);
    const std::chrono::steady_clock::time_point rir_create_begin =
        std::chrono::steady_clock::now();
    remote_identification_radar::session::RirSession rir_session =
        remote_identification_radar::session::RirSession::Create(configs.rir);
    const double rir_create_ms = demo::SteadyElapsedMs(rir_create_begin);
    demo::LogAcceptanceMs(0, 0.0, "初始化时间", "RIR", rir_create_ms);
    demo::LogAcceptanceMs(0, 0.0, "单个模型加载时间", "RIR",
                          rir_session.LastRecognitionDatabaseLoadMs());
    rir_site->Attach(std::make_unique<ca::RirSensorComponent>(
        std::move(rir_session), scene_data.rir_site_origin, scene_data.rir_designated_target_id,
        scene_data.rir_designation_duration_cycles, configs.rir.sensor_platform_id,
        configs.rir.mission.recognition_dwell_sec));
  }

  ca::Entity& platform = world.CreateEntity("platform");

  // 平台初始状态来自场景文件：机场地面（alt 0，六自由度机动从起飞开始）→
  // 巡航高度 → 沿场景航路巡航；区域巡逻场景（coverage 块）航路来自
  // AreaCoveragePlanner 规划，循环巡逻（loop_route）。
  const oneq::coordinate::LlaPositionDegM platform_origin = scene_data.platform_origin;
  platform.Attach(std::make_unique<ca::FlightComponent>(
      platform_origin, scene_data.initial_heading_deg, scene_data.cruise_speed_mps,
      scene_data.cruise_altitude_m, scene_data.waypoints,
      /*loop_route=*/scene_data.coverage.planned));

  // RF 链挂载序：Flight → ESR → [ECM] → AR → …。场景 sensors.* = false 则不挂，
  // 该通道不写视图/排除原因。
  if (scene_data.esr_enabled) {
    platform.Attach(std::make_unique<ca::EsrSensorComponent>(
        electronic_surveillance_radar::session::EsrSession::Create(configs.esr)));
  }
  if (scene_data.ecm_enabled && scene_data.esr_enabled) {
    platform.Attach(std::make_unique<ca::EcmSensorComponent>(
        electronic_countermeasure::session::EcmSession::Create(configs.ecm)));
  }
  if (scene_data.ar_enabled) {
    platform.Attach(std::make_unique<ca::ArSensorComponent>(
        airborne_radar::session::ArSession::Create(configs.ar), ca::kDemoPlatformEntityId,
        configs.ar.hardware.transmitter.equipment_id));
  }
  if (scene_data.eos_enabled) {
    platform.Attach(std::make_unique<ca::EosSensorComponent>(
        electro_optical_sensor::session::EosSession::Create(configs.eos)));
  }
  if (scene_data.sbirs_enabled) {
    const std::chrono::steady_clock::time_point sbirs_create_begin =
        std::chrono::steady_clock::now();
    sbirs_sensor::session::SbirsSession sbirs_session =
        sbirs_sensor::session::SbirsSession::Create(configs.sbirs);
    demo::LogAcceptanceMs(0, 0.0, "初始化时间", "SBIRS",
                          demo::SteadyElapsedMs(sbirs_create_begin));
    platform.Attach(std::make_unique<ca::SbirsSensorComponent>(std::move(sbirs_session)));
  }
  if (scene_data.sar_enabled) {
    platform.Attach(std::make_unique<ca::SarSensorComponent>(
        sar::session::SarSession::Create(configs.sar)));
  }

  // 融合配置来自场景文件（空间门限/方位相干门限/特征门/窗口/失跟周期/源权重；
  // 基线场景放宽方位相干门限到 8°：ESR 假设方位含平滑滞差、EOS 探测含扫描
  // 中心残差，同物理目标的跨源方位差实测可达 4-6°——业务层调参，非库内标准）。
  // 逐航迹滤波库内默认开启（AR 位置通道有运动学估计；SBIRS 角度-only 无量测
  // 原点时仍走关联通道）。
  platform.Attach(std::make_unique<ca::FusionComponent>(
      std::make_unique<fusion::FusionEngine>(scene_data.fusion)));

  // 目标推演组件（P3 链路）：读融合运动学估计，输出轨迹/发射点/落点/类型概率
  // （含误差预算）。挂载序在 Fusion 之后、Threat 之前。
  platform.Attach(std::make_unique<ca::InferenceComponent>());

  // 威胁评估配置来自场景文件（threat 块；缺省 = 库内默认权重/断点/阈值）。
  // 挂载序在 Fusion 之后：威胁组件每周期读融合输出与 AR 调试视图组装输入。
  platform.Attach(std::make_unique<ca::ThreatComponent>(scene_data.threat));

  // 验收行（多模型并行加载）：初始化结束时显示动态库（模块）加载完毕——
  // 按实际挂载的模块列清单 + 装配段真实墙钟，写入 integration_events.log。
  {
    std::vector<const char*> loaded_models;
    if (scene_data.rir_enabled) {
      loaded_models.push_back("RIR");
    }
    if (scene_data.esr_enabled) {
      loaded_models.push_back("ESR");
    }
    if (scene_data.ecm_enabled && scene_data.esr_enabled) {
      loaded_models.push_back("ECM");
    }
    if (scene_data.ar_enabled) {
      loaded_models.push_back("AR");
    }
    if (scene_data.eos_enabled) {
      loaded_models.push_back("EOS");
    }
    if (scene_data.sbirs_enabled) {
      loaded_models.push_back("SBIRS");
    }
    if (scene_data.sar_enabled) {
      loaded_models.push_back("SAR");
    }
    loaded_models.push_back("Fusion");
    loaded_models.push_back("Inference");
    loaded_models.push_back("Threat");
    std::string model_names;
    for (const char* name : loaded_models) {
      if (!model_names.empty()) {
        model_names += ",";
      }
      model_names += name;
    }
    demo::LogAcceptanceText(
        0, 0.0, "多模型并行加载",
        CA_FMT_FORMAT("动态库加载完毕 模块数={} 模块=[{}] 装配墙钟={:.3f}ms",
                      loaded_models.size(), model_names,
                      demo::SteadyElapsedMs(models_load_begin)));
  }

  // 事件接线：决策监听器（订阅融合信号，门限来自场景）+ 周期落盘输出
  // （统一契约 v2：多机平台轨迹/目标真值/航路/巡逻区域 CSV；集成端日志
  // 已由 InitIntegrationLog 装配，组件在 Step 内直写视图与事件）。
  demo::DecisionListener decision(world, scene_data.high_threat_confidence);
  demo::DemoOutputs outputs(output_dir);

  // 多机编队：主平台（platform 块，挂传感器/融合）+ 从机（platforms[] 数组，
  // 纯飞行）。每架飞行器各自的航路/区域任务 = "不同指令"；aircraft_id =
  // 1（主） + 2..N（按数组序）。FlightComponent 自包含（每实例一个
  // FlightManager），多机无结构改动。
  std::vector<ca::Entity*> wingmen;
  for (const auto& sp : scene_data.platforms) {
    ca::Entity& wing = world.CreateEntity(sp.name);
    wing.Attach(std::make_unique<ca::FlightComponent>(
        sp.origin, sp.initial_heading_deg, sp.cruise_speed_mps, sp.cruise_altitude_m,
        sp.waypoints, /*loop_route=*/sp.coverage.planned));
    wingmen.push_back(&wing);
  }
  const std::uint32_t aircraft_count =
      1U + static_cast<std::uint32_t>(scene_data.platforms.size());

  // 装配后写一次航路与巡逻区域（航路来自场景 waypoints/coverage 规划，
  // FlightComponent 构造时已就绪；zones 仅巡逻场景有区域）。
  outputs.RecordRoute(1U, scene_data.waypoints);
  if (scene_data.coverage.planned) {
    outputs.RecordZones("patrol_area", scene_data.coverage.area);
  }
  for (std::size_t i = 0U; i < scene_data.platforms.size(); ++i) {
    const auto& sp = scene_data.platforms[i];
    outputs.RecordRoute(static_cast<std::uint32_t>(i + 2U), sp.waypoints);
    if (sp.coverage.planned) {
      outputs.RecordZones(sp.name, sp.coverage.area);
    }
  }

  std::vector<demo::TargetEcefState> target_states =
      demo::MakeTargetStates(scene_data.targets, platform_origin);

  std::size_t max_fused_targets = 0U;
  // 天基平台（卫星）位置：凝视模式，固定于目标群质心正上方 + 场景高度
  // （ECEF z 轴），目标恒位于星下点附近（SBIRS az/el 为 ECI 极坐标——
  // 输入仍为 ECEF，库内按 GMST 旋转到 ECI，全向扫描 span 360° + 下视
  // el −90° 覆盖，GMST 平移不影响探测；消费方每周期注入世界模型）。
  // 无目标时保持上一周期位置（初始零向量 = 场景占位）。
  for (std::uint32_t cycle = 1U; cycle <= num_cycles; ++cycle) {
    demo::BeginViewLogCycle(cycle);
    // 消费方每周期注入共享场景状态（周期号/时间/四通道世界真值）。
    scene.cycle = cycle;
    scene.t_sec = static_cast<double>(cycle) * scene_data.dt_sec;
    scene.world_targets = target_states;
    scene.emitters = demo::MakeEmitterTruths(target_states, scene_data.esr, scene.t_sec);
    scene.sbirs_targets = demo::MakeSbirsTargetInputs(target_states);
    scene.sbirs_utc_julian_day = scene_data.sbirs_utc_julian_day;  // SBIRS ECI 输出参考系（UTC 儒略日）
    scene.sar_point_targets = demo::MakeSarPointTargets(target_states);
    if (rir_site != nullptr) {
      // RIR 场景目标：世界 ECEF → 站点局部 ENU（含识别特征真值铺样）。
      scene.rir_targets = demo::MakeRirSceneTargets(target_states, scene_data.rir_site_origin);
    }
    BeginRfWorldCycle(&scene, scene_data.dt_sec);
    if (!target_states.empty()) {
      double centroid_x = 0.0;
      double centroid_y = 0.0;
      double centroid_z = 0.0;
      for (const auto& target : target_states) {
        centroid_x += target.position.x_m;
        centroid_y += target.position.y_m;
        centroid_z += target.position.z_m;
      }
      const double count = static_cast<double>(target_states.size());
      scene.sbirs_satellite_position_ecef_m.x = centroid_x / count;
      scene.sbirs_satellite_position_ecef_m.y = centroid_y / count;
      scene.sbirs_satellite_position_ecef_m.z =
          centroid_z / count + scene_data.sbirs_satellite_altitude_m;
    }

    world.Step(scene_data.dt_sec);  // 步进序：Flight → ESR → [ECM] → AR → EOS → … → Fusion

    // 周期摘要 + 平台轨迹/调试视图落盘（多机：主平台 + 每架从机一行，
    // aircraft_id 区分；目标真值每目标一行，entity_type 透出空中/地面）。
    const auto* flight = platform.Find<ca::FlightComponent>();
    const auto* fusion = platform.Find<ca::FusionComponent>();
    max_fused_targets = std::max(max_fused_targets, fusion->targets().size());
    std::cout << "cycle=" << cycle
              << " plat[alt=" << flight->position().altitude_m
              << " hdg=" << flight->heading_deg() << " spd=" << flight->speed_mps()
              << " wp=" << flight->next_waypoint_index() << "/" << flight->route().size() << "]"
              << " fused=" << fusion->targets().size() << "\n";
    outputs.RecordPlatformRow(cycle, scene.t_sec, 1U, *flight);
    for (std::size_t i = 0U; i < wingmen.size(); ++i) {
      const auto* wing_flight = wingmen[i]->Find<ca::FlightComponent>();
      outputs.RecordPlatformRow(cycle, scene.t_sec,
                                static_cast<std::uint32_t>(i + 2U), *wing_flight);
    }
    for (const auto& target : target_states) {
      outputs.RecordTruthRow(cycle, scene.t_sec, target);
    }
    // 调试视图落盘由各传感器组件在 Step 内直写（取视图 → 人读摘要行 → 集成
    // 端日志；见 logger/logger.h 的 CA_LOG_VIEW），主程序不再收拢。

    // 消费方世界模型推进（在 Step 之后；周期号用于应用变速机动表）。
    demo::AdvanceTargetStates(target_states, cycle, scene_data.dt_sec, platform_origin);
  }

  demo::FlushIntegrationLog();
  outputs.Flush();
  // RIR 站点组件（未启用场景为 nullptr；摘要与冒烟按挂载与否条件化）。
  const ca::RirSensorComponent* rir_sensor =
      rir_site != nullptr ? rir_site->Find<ca::RirSensorComponent>() : nullptr;
  std::cout << "\n=== Component Attachment Summary ===\n"
            << "scene=" << scene_data.name
            << " cycles=" << num_cycles
            << " view_every=" << view_every
            << " aircraft=" << aircraft_count
            << " entities=" << world.entity_count()
            << " components=" << platform.component_count()
            << " patrol=" << (scene_data.coverage.planned ? "planned" : "off")
            << (scene_data.coverage.planned ? " (loop, waypoints="
                                               + std::to_string(scene_data.waypoints.size()) + ")"
                                            : "")
            << " events=" << demo::EventCount()
            << " sbirs_events=" << demo::SbirsEventCount()
            << " sar_products=" << demo::SarProductEventCount()
            << " command_issued=" << (decision.issued() ? "true" : "false")
            << " ar_views=" << demo::ArViewCount()
            << " eos_views=" << demo::EosViewCount()
            << " sbirs_views=" << demo::SbirsViewCount()
            << " sar_views=" << demo::SarViewCount()
            << " threat_views=" << demo::ThreatViewCount()
            << (rir_sensor != nullptr
                    ? " rir_views=" + std::to_string(demo::RirViewCount()) +
                          " rir_confirmed_cycles=" +
                          std::to_string(rir_sensor->confirmed_recognition_outputs())
                    : "")
            << "\n"
            << "log output -> " << output_dir
            << " (integration_events.log / integration_views.log / "
               "*_acceptance.log / rir_antenna_pattern.csv / rir_scan_pattern.csv / "
               "platform_track.csv / target_truth.csv / route_plan.csv / zones.csv)\n";

  // 外置查询演示：按实体名/类型查找平台实体，读取各传感器开关机与当前扫描
  // 方位（查询逻辑 = 组件 const getter；外部系统选定实体后按名/ID 拉取
  // 最新快照即可）。AR/SAR 无扫描方位语义，不提供角度字段。
  const auto* ar = platform.Find<ca::ArSensorComponent>();
  const auto* esr = platform.Find<ca::EsrSensorComponent>();
  const auto* eos = platform.Find<ca::EosSensorComponent>();
  const auto* sbirs = platform.Find<ca::SbirsSensorComponent>();
  const auto* sar = platform.Find<ca::SarSensorComponent>();
  std::cout << "\n=== Platform Sensor States (entity query) ===\n";
  if (ar != nullptr) {
    std::cout << "  ar    powered=" << (ar->powered_on() ? "on" : "off") << "\n";
  }
  if (esr != nullptr) {
    std::cout << "  esr   powered=" << (esr->powered_on() ? "on" : "off")
              << " scan_az=" << esr->scan_azimuth_deg() << " deg\n";
  }
  if (eos != nullptr) {
    std::cout << "  eos   powered=" << (eos->powered_on() ? "on" : "off")
              << " scan_az=" << eos->scan_azimuth_deg() << " deg\n";
  }
  if (sbirs != nullptr) {
    std::cout << "  sbirs powered=" << (sbirs->powered_on() ? "on" : "off")
              << " scan_az=" << sbirs->scan_azimuth_deg() << " deg\n";
  }
  if (sar != nullptr) {
    std::cout << "  sar   powered=" << (sar->powered_on() ? "on" : "off") << "\n";
  }
  if (rir_sensor != nullptr) {
    std::cout << "  rir   powered=" << (rir_sensor->powered_on() ? "on" : "off")
              << " confirmed_cycles=" << rir_sensor->confirmed_recognition_outputs() << "\n";
  }

  // 冒烟：视图按摘要间隔求余后的拍数断言（默认每周期一行；--view-every N
  // 时为 cycles/N）。事件仍按场景 smoke 下限。
  const auto& smoke = scene_data.smoke;
  const std::uint32_t view_ticks = demo::ViewLogExpectedTicks(num_cycles, view_every);
  const bool views_ok =
      (ar == nullptr || demo::ArViewCount() >= view_ticks) &&
      (eos == nullptr || demo::EosViewCount() >= view_ticks) &&
      (sbirs == nullptr || demo::SbirsViewCount() >= view_ticks) &&
      (sar == nullptr || demo::SarViewCount() == view_ticks) &&
      demo::ThreatViewCount() >= view_ticks;
  const bool powered_ok =
      (ar == nullptr || ar->powered_on()) && (esr == nullptr || esr->powered_on()) &&
      (eos == nullptr || eos->powered_on()) && (sbirs == nullptr || sbirs->powered_on()) &&
      (sar == nullptr || sar->powered_on()) &&
      (rir_sensor == nullptr || rir_sensor->powered_on());
  if (demo::EventCount() < smoke.min_key_events ||
      (sbirs != nullptr && demo::SbirsEventCount() < smoke.min_sbirs_events) ||
      (sar != nullptr && demo::SarProductEventCount() < smoke.min_sar_products) ||
      max_fused_targets < smoke.min_fused_targets ||
      (rir_sensor != nullptr &&
       rir_sensor->confirmed_recognition_outputs() < smoke.min_rir_recognition_outputs) ||
      outputs.platform_rows() != num_cycles * aircraft_count || !views_ok || !powered_ok) {
    std::cerr << "SMOKE FAILED: events=" << demo::EventCount()
              << " (>=" << smoke.min_key_events << " required), sbirs_events="
              << demo::SbirsEventCount() << " (>=" << smoke.min_sbirs_events
              << " required), sar_products=" << demo::SarProductEventCount()
              << " (>=" << smoke.min_sar_products << " required), max_fused="
              << max_fused_targets << " (>=" << smoke.min_fused_targets
              << " required), rir_confirmed_cycles="
              << (rir_sensor != nullptr
                      ? std::to_string(rir_sensor->confirmed_recognition_outputs())
                      : "n/a")
              << " (>=" << smoke.min_rir_recognition_outputs << " required), platform_rows="
              << outputs.platform_rows() << " (== " << num_cycles << " × " << aircraft_count
              << " 机 required), "
              << "ar_views=" << demo::ArViewCount()
              << " (>= " << view_ticks << " required), eos_views="
              << demo::EosViewCount() << " (>= " << view_ticks << " required), sbirs_views="
              << demo::SbirsViewCount() << " (>= " << view_ticks << " required), sar_views="
              << demo::SarViewCount() << " (== " << view_ticks << " required), threat_views="
              << demo::ThreatViewCount() << " (>= " << view_ticks << " required), sensor_powered="
              << (powered_ok ? "true" : "false") << " (mounted only)\n";
    return 1;
  }
  return 0;
}
