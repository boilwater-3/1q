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
 *   融合更新、航点到达、决策指令（高置信威胁 → ECCM 反制，事件链演示）。
 *
 * 输出：控制台事件流摘要 + platform_track.csv / events.csv（复用
 * examples/common/csv_writer.h）。
 */

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/types.h"
#include "1q/coordinate/velocity_transform.h"
#include "1q/electro_optical_sensor/config/EosSessionConfig.h"
#include "1q/electro_optical_sensor/session/EosExternalInputAdapter.h"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfig.h"
#include "1q/electromagnetics/RfScene.h"
#include "1q/fusion/FusionEngine.h"
#include "1q/fusion/FusedTarget.h"
#include "1q/navigation/RoutePoint.h"
#include "1q/sar/config/SarSessionConfig.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"
// 四域配置加载器位于 examples/common/config_loaders/<域>/（经 ONEQ_EXAMPLE_COMMON_DIR 解析）。
#include "config_loaders/airborne_radar/config_loader.h"
#include "config_loaders/electro_optical/config_loader.h"
#include "config_loaders/electronic_warfare/config_loader.h"
#include "config_loaders/sar/config_loader.h"
#include "config_loaders/sbirs_sensor/config_loader.h"
#include "csv_writer.h"

#include "components/ar_sensor_component.h"
#include "components/eos_sensor_component.h"
#include "components/esr_sensor_component.h"
#include "components/flight_component.h"
#include "components/fusion_component.h"
#include "components/sar_sensor_component.h"
#include "components/sbirs_sensor_component.h"
#include "components/scene_types.h"
#include "core/events.h"
#include "core/world.h"

namespace ar_session = airborne_radar::session;
namespace ca = component_attachment;

namespace {

constexpr std::uint32_t kNumCycles = 400U;
constexpr double kDtSec = 1.0;
constexpr char kDefaultOutputDir[] = "/tmp/component_attachment_viz";
/// 平台巡航高度（m）：c172x 低空巡航量级；目标真值固定在此高度。
/// EOS 探测距离窗 ≈ 高度 / sin(俯仰角)（min/max 2°/1°）→ 400 m 时
/// [11.5, 22.9] km，目标斜距全程稳定在窗内（见 kTargetScript）。
constexpr double kCruiseAltitudeM = 400.0;
/// 巡航速度参考（m/s）：FD 模式以性能面 profile 覆盖（~47-50 m/s），
/// 运动学回退沿用本值；目标东速略低于本值（平台追近，距离窗内稳定）。
constexpr double kCruiseSpeedMps = 50.0;

/// 决策门限：融合置信度达到该值视为高置信威胁（示例业务策略，阈值与
/// 行为层一致；行为层每周期重发指令，本示例经事件链只下发一次）。
constexpr double kHighThreatConfidence = 3.0;

/// 五会话配置聚合（消费方装配输入）。
struct ComponentAttachmentConfigs {
  airborne_radar::config::ArSessionConfig ar{};
  electronic_surveillance_radar::config::EsrSessionConfig esr{};
  electro_optical_sensor::config::EosSessionConfig eos{};
  sbirs_sensor::config::SbirsSessionConfig sbirs{};
  sar::config::SarSessionConfig sar{};
};

/// 打印命令行用法。
void PrintUsage(const char* program) {
  std::cout << "Usage: " << program << " [--cycles <n>] [--output-dir <dir>]\n"
            << "  --cycles <n>        仿真周期数（默认 " << kNumCycles << "）\n"
            << "  --output-dir <dir>  CSV 输出目录（默认 " << kDefaultOutputDir << "）\n";
}

/// 加载四份会话配置（复用各域 config_loader 与 examples/configs/ 同源 JSON）。
ComponentAttachmentConfigs LoadConfigs() {
  ComponentAttachmentConfigs configs;
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
  if (!examples::LoadSbirsSessionConfigFromFile(SCENE_CONFIG_DIR "/sbirs.json",
                                                &configs.sbirs, &error)) {
    std::cerr << "Failed to load SBIRS config: " << error << "\n";
    std::exit(1);
  }
  if (!examples::LoadSarSessionConfigFromFile(SCENE_CONFIG_DIR "/sar.json",
                                              &configs.sar, &error)) {
    std::cerr << "Failed to load SAR config: " << error << "\n";
    std::exit(1);
  }
  // 跨会话时间对齐与视场适配（业务层调参，与 behavior_layer 同源）：
  // - EOS 周期校验要求 dt ≤ 10/frame_rate_hz（帧率 30 → 上限 0.33 s），
  //   演示按 1 s/周期推进 → 帧率覆写为 10 Hz；
  // - 原配置为下视地面监视（视轴下俯 45°），与空中目标场景不匹配 →
  //   覆写为水平扫描；目标在平台正北（平台局部系 az 90°）→ 扫描
  //   50°~130°（原 ±40° 覆盖正东）。
  configs.eos.mission.frame_rate_hz = 10.0f;
  configs.eos.mission.scan_rate_deg_per_sec = 20.0f;
  configs.eos.mission.scan_start_az_deg = 50.0f;
  configs.eos.mission.scan_end_az_deg = 130.0f;
  configs.eos.mission.scan_center_el_deg = 0.0f;
  configs.eos.mission.boresight_depression_deg = 0.0f;
  // - SAR 任务几何覆写适配演示场景（sar.json 为 100 km 斜距 / 180 m/s 的
  //   远程监视档）：场景中心 → 目标群中心，落在 FD 巡航段（~cycle 331 起
  //   正东直线，lon ≈ 120.05→120.08）的侧方 —— 目标与平台同速东飞（见
  //   kTargetScript），巡航段平台飞越场景中心正南时 squint ≈ 0°，成像窗
  //   口覆盖巡航段（10° 门限下 ±2.3 km 纵向窗口）；起飞/转弯段（cycle
  //   1-330）侧视几何不成立，被库内 squint 门控拒绝（预期行为，产品事件
  //   只在窗口期产生）。目标 RCS 仅 2.2/1.4 m²，链路预算在 10 kW 峰值
  //   功率下 SNR ≈ −29 dB（低于 minimum_snr_db）→ 功率提升至 1 MW、
  //   天线增益 30 → 40 dBi（SAR 常用量级），SNR ≈ +10 dB 过门限。
  //   孔径参数（1024 脉冲 @ PRF 100 Hz ≈ 10.24 s 积累）保持。
  configs.sar.hardware.peak_power_w = 1.0e6;
  configs.sar.hardware.antenna_gain_db = 40.0;
  configs.sar.policy.max_allowed_squint_angle_deg = 10.0;
  configs.sar.mission.scene_center_latitude_deg = 30.0 + 13.0e3 / 111.0e3;
  configs.sar.mission.scene_center_longitude_deg = 120.06;
  configs.sar.mission.scene_center_altitude_m = kCruiseAltitudeM;
  configs.sar.mission.nominal_slant_range_m = 13000.0;
  configs.sar.mission.platform_speed_mps = kCruiseSpeedMps;
  return configs;
}

/// 目标脚本：2 个空中目标（正北前方 12/14 km）。目标与平台同速东飞
/// （v_east = 巡航地速 ~47 m/s，FD 性能面实际地速，保持平台与目标相对
/// 经度不变）→ 目标恒在平台正北侧方，SAR 视线垂直航迹（正侧视，
/// squint ≈ 0° 全程成立）；北速 ±5 m/s 提供两目标间分离与 AR 径向速度
/// （多普勒）。方位（北偏东 0° = 正北）落在 EOS 扫描覆盖内（平台局部系
/// az 0 = 东，扫描 50°~130°）。巡航段平台 alt 400 m 的 EOS 探测距离窗
/// ≈ [11.5, 22.9] km（min/max 探测俯仰角 2°/1°），目标斜距全程稳定在
/// 窗内（见 kTargetScript）。
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

/// 目标 ECEF 运动学状态（三通道共享同一物理目标）。
struct TargetEcefState {
  oneq::coordinate::EcefPositionM position{};
  oneq::coordinate::EcefVelocityMps velocity{};
  float rcs{0.0f};
};

/// 目标脚本 → ECEF 状态（方位/距离经库内 ENU 偏移函数投影到 ECEF，速度经
/// ENU 速度函数投影；z 取平台基准高度 + 巡航高度偏移，目标恒在空中，不随
/// 平台起飞段高度变化）。脚本为编译期合法常量，投影调用不会失败。
std::vector<TargetEcefState> MakeTargetStates(
    const oneq::coordinate::EcefPositionM& platform_ecef,
    const oneq::coordinate::LlaPositionDegM& platform_origin) {
  std::vector<TargetEcefState> states;
  states.reserve(2U);
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
/// 在 pfa=1e-6 下以多脉冲积分过检）。两辐射源中心频率互异（9.5/10.0 GHz），
/// 保证 ESR 分选聚簇能稳定分离出 2 条假设航迹。
std::vector<oneq::electromagnetics::RfSceneEmission> MakeEmitterTruths(
    const std::vector<TargetEcefState>& states, double window_start_time_s) {
  const double kCenterFrequencyHz[] = {9.5e9, 10.0e9};
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

/// SBIRS 红外目标真值：同一物理目标（红外外观参数与 EOS 同源）。
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

/// SAR 点目标真值：同一物理目标（LLA 位置 + RCS，m² → dBsm）。
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

/// 目标 ECEF 欧拉推进（消费方世界模型，与 behavior_layer 一致）。
void AdvanceTargetStates(std::vector<TargetEcefState>& states, double dt_s) {
  for (auto& state : states) {
    state.position.x_m += state.velocity.x_mps * dt_s;
    state.position.y_m += state.velocity.y_mps * dt_s;
    state.position.z_m += state.velocity.z_mps * dt_s;
  }
}

/// snprintf 风格格式化辅助（事件 detail 文本；按返回值动态分配，避免截断）。
std::string Fmt(const char* format, ...) {
  va_list args;
  va_start(args, format);
  va_list args_copy;
  va_copy(args_copy, args);
  const int length = std::vsnprintf(nullptr, 0, format, args_copy);
  va_end(args_copy);
  std::string out(static_cast<std::size_t>(length), '\0');
  std::vsnprintf(out.data(), out.size() + 1, format, args);
  va_end(args);
  return out;
}

/// 事件日志器：订阅 World 全部信号，逐条打印并写 events.csv（事件类型
/// 名 → 稳定字符串，detail 为可读摘要文本；订阅连接以 scoped_connection
/// 持有，析构自动断开，避免 World 先于订阅者析构时的悬垂）。
class EventLogger {
 public:
  explicit EventLogger(const std::string& output_dir)
      : events_csv_(output_dir + "/events.csv",
                    "cycle,t_sec,event_type,detail") {}

  void Connect(ca::World& world) {
    connections_.push_back(world.signals().on_platform_state.connect(
        [this](const ca::PlatformStateEvent& e) {
          Record("platform_state",
                 Fmt("pos=(%.4f,%.4f,%.1f) hdg=%.1f spd=%.1f wp=%zu/%zu",
                     e.position_ecef_m.x_m, e.position_ecef_m.y_m, e.altitude_m,
                     e.heading_deg, e.speed_mps, e.waypoint_index, e.waypoint_count),
                 e.cycle, e.t_sec);
        }));
    connections_.push_back(world.signals().on_waypoint_reached.connect(
        [this](const ca::WaypointReachedEvent& e) {
          Record("waypoint_reached",
                 Fmt("index=%zu distance=%.1f", e.waypoint_index, e.distance_m),
                 std::uint64_t{0U}, e.t_sec);
        }));
    connections_.push_back(world.signals().on_target_confirmed.connect(
        [this](const ca::TargetConfirmedEvent& e) {
          Record("target_confirmed",
                 Fmt("target=%llu pos=(%.5f,%.5f)",
                     static_cast<unsigned long long>(e.target_id), e.position.latitude_deg,
                     e.position.longitude_deg),
                 e.cycle, 0.0);
        }));
    connections_.push_back(world.signals().on_target_lost.connect(
        [this](const ca::TargetLostEvent& e) {
          Record("target_lost",
                 Fmt("target=%llu reason=%s", static_cast<unsigned long long>(e.target_id),
                     e.reason.c_str()),
                 e.cycle, 0.0);
        }));
    connections_.push_back(world.signals().on_emitter_hypothesis.connect(
        [this](const ca::EmitterHypothesisEvent& e) {
          Record("emitter_hypothesis",
                 Fmt("hyp=%llu az=%.1f conf=%.2f mode=%d threat=%d",
                     static_cast<unsigned long long>(e.hypothesis_id), e.bearing_az_deg,
                     e.confidence, static_cast<int>(e.mode), static_cast<int>(e.threat_level)),
                 e.cycle, 0.0);
        }));
    connections_.push_back(world.signals().on_eos_detection.connect(
        [this](const ca::EosDetectionEvent& e) {
          Record("eos_detection",
                 Fmt("kind=%d det=%llu target=%llu snr=%.1fdB az=%.1f",
                     static_cast<int>(e.kind),
                     static_cast<unsigned long long>(e.detection_id),
                     static_cast<unsigned long long>(e.target_id), e.snr_db, e.az_deg),
                 e.cycle, 0.0);
        }));
    connections_.push_back(world.signals().on_sbirs_detection.connect(
        [this](const ca::SbirsDetectionEvent& e) {
          ++sbirs_event_count_;
          Record("sbirs_detection",
                 Fmt("kind=%d det=%llu target=%llu snr=%.1f az=%.1f",
                     static_cast<int>(e.kind),
                     static_cast<unsigned long long>(e.detection_id),
                     static_cast<unsigned long long>(e.target_id), e.infrared_snr_linear,
                     e.az_deg),
                 e.cycle, 0.0);
        }));
    connections_.push_back(world.signals().on_sar_product.connect(
        [this](const ca::SarProductEvent& e) {
          ++sar_product_event_count_;
          Record("sar_product",
                 Fmt("kind=%d stage=%d snr=%.1fdB%s%s", static_cast<int>(e.kind),
                     static_cast<int>(e.stage), e.estimated_snr_db,
                     e.abort_reason.empty() ? "" : " abort=", e.abort_reason.c_str()),
                 e.cycle, 0.0);
        }));
    connections_.push_back(world.signals().on_fusion_updated.connect(
        [this](const ca::FusionUpdatedEvent& e) {
          std::string channels;
          for (const auto& channel : e.channels) {
            if (!channels.empty()) channels += ",";
            channels += Fmt("%u:%zu", channel.first, channel.second);
          }
          Record("fusion_updated",
                 Fmt("key=%llu conf=%.2f new=%zu lost=%zu ch[%s]",
                     static_cast<unsigned long long>(e.key), e.confidence, e.new_targets,
                     e.lost_targets, channels.c_str()),
                 e.cycle, 0.0);
        }));
    connections_.push_back(world.signals().on_command_issued.connect(
        [this](const ca::CommandIssuedEvent& e) {
          Record("command_issued", "cmd=" + e.command, e.cycle, 0.0);
        }));
  }

  void Flush() { events_csv_.Flush(); }

  std::size_t event_count() const { return count_; }
  std::size_t sbirs_event_count() const { return sbirs_event_count_; }
  std::size_t sar_product_event_count() const { return sar_product_event_count_; }

 private:
  void Record(const char* type, const std::string& detail, std::uint64_t cycle, double t_sec) {
    ++count_;
    std::cout << "  [" << type << "] " << detail << "\n";
    // detail 为自由文本（含逗号/括号）：按 RFC 4180 转义，保证列结构完整。
    events_csv_.WriteRow(Fmt("%llu,%.2f,%s,%s", static_cast<unsigned long long>(cycle), t_sec,
                             type, examples::EscapeCsvField(detail).c_str()));
  }

  examples::CsvWriter events_csv_;
  std::vector<boost::signals2::scoped_connection> connections_{};
  std::size_t count_{0U};
  std::size_t sbirs_event_count_{0U};
  std::size_t sar_product_event_count_{0U};
};

/// 决策监听器：订阅融合更新事件，高置信威胁首次出现时发布指令事件
/// （事件链演示：Fusion → decision → command）。
class DecisionListener {
 public:
  explicit DecisionListener(ca::World& world) : world_(world) {
    world_.signals().on_fusion_updated.connect([this](const ca::FusionUpdatedEvent& e) {
      if (e.confidence >= kHighThreatConfidence && !issued_) {
        issued_ = true;
        ca::CommandIssuedEvent command;
        command.cycle = e.cycle;
        command.command = "ENABLE_ANTI_FALSE_TARGET_DISCRIMINATION";
        world_.signals().on_command_issued(command);
      }
    });
  }

  bool issued() const { return issued_; }

 private:
  ca::World& world_;
  bool issued_{false};
};

}  // namespace

int main(int argc, char* argv[]) {
  // 命令行参数：--cycles / --output-dir。
  std::uint32_t num_cycles = kNumCycles;
  std::string output_dir = kDefaultOutputDir;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--cycles") {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for --cycles\n";
        PrintUsage(argv[0]);
        return 1;
      }
      num_cycles = static_cast<std::uint32_t>(std::atoi(argv[++i]));
    } else if (arg == "--output-dir") {
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
  if (num_cycles == 0U) {
    std::cerr << "Invalid --cycles value: must be > 0\n";
    return 1;
  }
  std::filesystem::create_directories(output_dir);

  // 装配：共享场景状态 → World → 平台实体 + 5 组件（挂载序 = 步进序）。
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
    wp.position.altitude_m = kCruiseAltitudeM;
    wp.speed_mps = kCruiseSpeedMps;
    wp.radius_m = 500.0;
    route.push_back(wp);
  }
  platform.Attach(std::make_unique<ca::FlightComponent>(platform_origin, 90.0, kCruiseSpeedMps,
                                                        kCruiseAltitudeM,
                                                        std::move(route)));

  const ComponentAttachmentConfigs configs = LoadConfigs();
  platform.Attach(std::make_unique<ca::ArSensorComponent>(
      ar_session::ArSession::Create(configs.ar)));
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

  // 事件接线：日志器 + 决策监听器（订阅 World 信号）。
  EventLogger logger(output_dir);
  logger.Connect(world);
  DecisionListener decision(world);

  // 世界真值脚本：以平台初始 ECEF 为基准（消费方场景编排）。
  oneq::coordinate::EcefPositionM platform_ecef;
  if (!oneq::coordinate::TryLlaToEcef(platform_origin, &platform_ecef)) {
    std::cerr << "Invalid platform LLA\n";
    return 1;
  }
  std::vector<TargetEcefState> target_states = MakeTargetStates(platform_ecef, platform_origin);

  // 平台轨迹 CSV（周期级；事件流经 EventLogger 写 events.csv）。
  examples::CsvWriter platform_csv(output_dir + "/platform_track.csv",
                                   "cycle,t_sec,lat_deg,lon_deg,alt_m,heading_deg,speed_mps,wp_index");

  std::uint32_t validation_error_count = 0U;
  (void)validation_error_count;  // 冒烟断言改用事件/融合/行数指标（见下）
  std::size_t platform_rows = 0U;
  std::size_t max_fused_targets = 0U;
  for (std::uint32_t cycle = 1U; cycle <= num_cycles; ++cycle) {
    // 消费方每周期注入共享场景状态（周期号/时间/四通道世界真值）。
    scene.cycle = cycle;
    scene.t_sec = static_cast<double>(cycle) * kDtSec;
    scene.ar_targets = MakeArTargetInputs(target_states);
    scene.emitters = MakeEmitterTruths(target_states, scene.t_sec);
    scene.optical_targets = MakeOpticalTargets(target_states);
    scene.sbirs_targets = MakeSbirsTargetInputs(target_states);
    scene.sar_point_targets = MakeSarPointTargets(target_states);
    // 天基平台（卫星）位置：凝视模式，固定于目标群中心正上方 +500 km
    // （ECEF z 轴），目标始终位于星下点附近（SBIRS az/el 为 ECEF 极坐标，
    // 全向扫描 span 360° + 下视 el −90° 覆盖；消费方每周期注入世界模型）。
    constexpr double kSbirsSatelliteAltitudeM = 500000.0;
    scene.sbirs_satellite_position_ecef_m.x =
        0.5 * (target_states[0].position.x_m + target_states[1].position.x_m);
    scene.sbirs_satellite_position_ecef_m.y =
        0.5 * (target_states[0].position.y_m + target_states[1].position.y_m);
    scene.sbirs_satellite_position_ecef_m.z =
        0.5 * (target_states[0].position.z_m + target_states[1].position.z_m) +
        kSbirsSatelliteAltitudeM;

    world.Step(kDtSec);  // 按挂载序步进：Flight → AR → ESR → EOS → SBIRS → SAR → Fusion

    // 周期摘要 + 平台轨迹落盘。
    const auto* flight = platform.Find<ca::FlightComponent>();
    const auto* fusion = platform.Find<ca::FusionComponent>();
    max_fused_targets = std::max(max_fused_targets, fusion->targets().size());
    std::cout << "cycle=" << cycle
              << " plat[alt=" << flight->position().altitude_m
              << " hdg=" << flight->heading_deg() << " spd=" << flight->speed_mps()
              << " wp=" << flight->next_waypoint_index() << "/" << flight->route().size() << "]"
              << " fused=" << fusion->targets().size() << "\n";
    platform_csv.WriteRow(Fmt("%u,%.2f,%.7f,%.7f,%.1f,%.1f,%.1f,%zu", cycle,
                              scene.t_sec, flight->position().latitude_deg,
                              flight->position().longitude_deg, flight->position().altitude_m,
                              flight->heading_deg(), flight->speed_mps(),
                              flight->next_waypoint_index()));
    ++platform_rows;

    // 消费方世界模型推进（在 Step 之后，与 behavior_layer 周期语义一致）。
    AdvanceTargetStates(target_states, kDtSec);
  }

  logger.Flush();
  platform_csv.Flush();
  std::cout << "\n=== Component Attachment Summary ===\n"
            << "cycles=" << num_cycles
            << " entities=" << world.entity_count()
            << " components=" << platform.component_count()
            << " events=" << logger.event_count()
            << " sbirs_events=" << logger.sbirs_event_count()
            << " sar_products=" << logger.sar_product_event_count()
            << " command_issued=" << (decision.issued() ? "true" : "false") << "\n"
            << "csv output -> " << output_dir
            << " (platform_track.csv / events.csv)\n";

  // 冒烟断言：端到端链路必须有产出（每周期平台状态事件、SBIRS 探测事件、
  // SAR 图像产品事件、至少一个融合目标、平台轨迹行数 = 周期数），否则视为
  // 链路断裂（ctest 失败）。
  if (logger.event_count() < num_cycles || logger.sbirs_event_count() == 0U ||
      logger.sar_product_event_count() == 0U || max_fused_targets == 0U ||
      platform_rows != num_cycles) {
    std::cerr << "SMOKE FAILED: events=" << logger.event_count()
              << " (>= " << num_cycles << " required), sbirs_events="
              << logger.sbirs_event_count() << " (>0 required), sar_products="
              << logger.sar_product_event_count() << " (>0 required), max_fused="
              << max_fused_targets << " (>0 required), platform_rows=" << platform_rows
              << " (== " << num_cycles << " required)\n";
    return 1;
  }
  return 0;
}
