/**
 * @file component_attachment_demo.cpp
 * @brief 自定义实体-组件示例主程序（第二种示例模式）。
 *
 * 1. 装配：场景描述文件（--scene，默认 scenes/baseline_takeoff_east.json）
 *    → SceneData → 共享场景状态 → World → 平台实体 + 7 组件（挂载序 =
 *    步进序 Flight → AR → ESR → EOS → SBIRS → SAR → Fusion），组件间事件
 *    通信用 Boost.Signals2（core/，零自定义分发层）；
 * 2. 编排：每周期注入四通道世界真值 → World::Step → 周期摘要与平台轨迹落盘；
 * 3. 收尾：集成端日志刷盘、按实体查询传感器状态演示、冒烟断言（下限来自
 *    场景文件 smoke 块）。
 * 场景数据见 scene_data.h，世界模型真值脚本见 scene_script.h，配置加载见
 * demo_config.h，输出落盘与事件消费见 demo_output.h。
 */

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "1q/airborne_radar/session/ArSession.h"
#include "1q/coordinate/types.h"
#include "1q/electro_optical_sensor/session/EosSession.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"
#include "1q/fusion/FusionEngine.h"
#include "1q/fusion/FusedTarget.h"
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
#include "scene_data.h"
#include "scene_script.h"

namespace ca = component_attachment;
namespace demo = component_attachment::demo;

namespace {

/// 默认场景文件：CMake 注入的场景目录（examples/component_attachment/scenes/）。
constexpr char kDefaultSceneFile[] = CA_SCENE_DIR "/baseline_takeoff_east.json";

}  // namespace

int main(int argc, char* argv[]) {
  // 命令行参数：--scene（场景描述文件）/ --cycles（覆盖场景周期数）/
  // --output-dir（输出目录）。
  std::string scene_path = kDefaultSceneFile;
  int cycles_override = -1;  // < 0 = 未指定，用场景文件值
  std::string output_dir = demo::kDefaultOutputDir;
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
  std::filesystem::create_directories(output_dir);

  // 集成端日志初始化（三个日志文件：库日志 1q_library.log + 集成端事件行
  // integration_events.log + 视图行 integration_views.log；见 components/demo_log.h）。
  // 须在会话创建之前调用：库内 PROJECT_LOG_* 走 spdlog 默认 logger，装配后库
  // 日志即入文件而非 stdout。
  demo::InitIntegrationLog(output_dir);

  // 装配：共享场景状态 → World → 平台实体 + 7 组件（挂载序 = 步进序）。
  ca::DemoSceneState scene;
  ca::World world(scene);
  ca::Entity& platform = world.CreateEntity("platform");

  // 平台初始状态来自场景文件：机场地面（alt 0，六自由度机动从起飞开始）→
  // 巡航高度 → 沿场景航路巡航。
  const oneq::coordinate::LlaPositionDegM platform_origin = scene_data.platform_origin;
  platform.Attach(std::make_unique<ca::FlightComponent>(
      platform_origin, scene_data.initial_heading_deg, scene_data.cruise_speed_mps,
      scene_data.cruise_altitude_m, scene_data.waypoints));

  // 五会话配置：JSON 基线（examples/configs/）+ 场景业务覆写（EOS 扫描/SAR
  // 任务几何与链路，见 ApplySceneOverrides）。
  demo::ComponentAttachmentConfigs configs = demo::LoadConfigs();
  demo::ApplySceneOverrides(scene_data, &configs);
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

  // 融合配置来自场景文件（空间门限/方位相干门限/特征门/窗口/失跟周期/源权重；
  // 基线场景放宽方位相干门限到 8°：ESR 假设方位含平滑滞差、EOS 探测含扫描
  // 中心残差，同物理目标的跨源方位差实测可达 4-6°——业务层调参，非库内标准）。
  platform.Attach(std::make_unique<ca::FusionComponent>(
      std::make_unique<fusion::FusionEngine>(scene_data.fusion)));

  // 事件接线：决策监听器（订阅融合信号，门限来自场景）+ 周期落盘输出
  // （平台轨迹 CSV；集成端日志已由 InitIntegrationLog 装配，组件在 Step 内
  // 直写视图与事件）。
  demo::DecisionListener decision(world, scene_data.high_threat_confidence);
  demo::DemoOutputs outputs(output_dir);

  std::vector<demo::TargetEcefState> target_states =
      demo::MakeTargetStates(scene_data.targets, platform_origin);

  std::size_t max_fused_targets = 0U;
  // 天基平台（卫星）位置：凝视模式，固定于目标群质心正上方 + 场景高度
  // （ECEF z 轴），目标恒位于星下点附近（SBIRS az/el 为 ECEF 极坐标，
  // 全向扫描 span 360° + 下视 el −90° 覆盖；消费方每周期注入世界模型）。
  // 无目标时保持上一周期位置（初始零向量 = 场景占位）。
  for (std::uint32_t cycle = 1U; cycle <= num_cycles; ++cycle) {
    // 消费方每周期注入共享场景状态（周期号/时间/四通道世界真值）。
    scene.cycle = cycle;
    scene.t_sec = static_cast<double>(cycle) * scene_data.dt_sec;
    scene.ar_targets = demo::MakeArTargetInputs(target_states);
    scene.emitters = demo::MakeEmitterTruths(target_states, scene_data.esr, scene.t_sec);
    scene.optical_targets = demo::MakeOpticalTargets(target_states);
    scene.sbirs_targets = demo::MakeSbirsTargetInputs(target_states);
    scene.sar_point_targets = demo::MakeSarPointTargets(target_states);
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

    world.Step(scene_data.dt_sec);  // 按挂载序步进：Flight → AR → ESR → EOS → SBIRS → SAR → Fusion

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
    demo::AdvanceTargetStates(target_states, scene_data.dt_sec);
  }

  demo::FlushIntegrationLog();
  outputs.Flush();
  std::cout << "\n=== Component Attachment Summary ===\n"
            << "scene=" << scene_data.name
            << " cycles=" << num_cycles
            << " entities=" << world.entity_count()
            << " components=" << platform.component_count()
            << " events=" << demo::EventCount()
            << " sbirs_events=" << demo::SbirsEventCount()
            << " sar_products=" << demo::SarProductEventCount()
            << " command_issued=" << (decision.issued() ? "true" : "false")
            << " ar_views=" << demo::ArViewCount()
            << " eos_views=" << demo::EosViewCount()
            << " sbirs_views=" << demo::SbirsViewCount()
            << " sar_views=" << demo::SarViewCount() << "\n"
            << "log output -> " << output_dir
            << " (integration_events.log / integration_views.log / 1q_library.log / platform_track.csv)\n";

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

  // 冒烟断言：端到端链路必须有产出（关键事件/SBIRS/SAR/融合目标下限来自
  // 场景文件 smoke 块，无目标等零产出场景显式置 0；平台轨迹行数 = 周期数；
  // 视图行数每周期 ≥ 1 行、五传感器全程开机），否则视为链路断裂（ctest 失败）。
  // 断言与日志模式无关：视图按"每周期至少一行"断言（默认跨周期增量模式下状态
  // 变化周期会写多行，故为 ≥ 周期数；SAR 阶段型摘要恒每周期一行，== 周期数）；
  // 事件按"关键事件存在"断言（默认只记关键模式下平台状态等重复事件不落盘）。
  // 前置条件：视图行数由组件 Step 内计数，依赖每周期到达视图日志调用点（本
  // 场景 Flight 恒挂载、坐标适配恒成功；场景改动需同步检查该断言）。
  const auto& smoke = scene_data.smoke;
  if (demo::EventCount() < smoke.min_key_events ||
      demo::SbirsEventCount() < smoke.min_sbirs_events ||
      demo::SarProductEventCount() < smoke.min_sar_products ||
      max_fused_targets < smoke.min_fused_targets ||
      outputs.platform_rows() != num_cycles || demo::ArViewCount() < num_cycles ||
      demo::EosViewCount() < num_cycles || demo::SbirsViewCount() < num_cycles ||
      demo::SarViewCount() != num_cycles ||
      !ar->powered_on() || !esr->powered_on() ||
      !eos->powered_on() || !sbirs->powered_on() || !sar->powered_on()) {
    std::cerr << "SMOKE FAILED: events=" << demo::EventCount()
              << " (>=" << smoke.min_key_events << " required), sbirs_events="
              << demo::SbirsEventCount() << " (>=" << smoke.min_sbirs_events
              << " required), sar_products=" << demo::SarProductEventCount()
              << " (>=" << smoke.min_sar_products << " required), max_fused="
              << max_fused_targets << " (>=" << smoke.min_fused_targets
              << " required), platform_rows="
              << outputs.platform_rows() << " (== " << num_cycles << " required), "
              << "ar_views=" << demo::ArViewCount()
              << " (>= " << num_cycles << " required), eos_views="
              << demo::EosViewCount() << " (>= " << num_cycles << " required), sbirs_views="
              << demo::SbirsViewCount() << " (>= " << num_cycles << " required), sar_views="
              << demo::SarViewCount() << " (== " << num_cycles << " required), sensor_powered="
              << (ar->powered_on() && esr->powered_on() && eos->powered_on() &&
                  sbirs->powered_on() && sar->powered_on() ? "true" : "false")
              << " (all required)\n";
    return 1;
  }
  return 0;
}
