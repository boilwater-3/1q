/**
 * @file app/demo_config.cpp
 * @brief 演示常量与配置加载实现（见 app/demo_config.h）。
 */

#include "app/demo_config.h"

#include <cstdlib>
#include <iostream>
#include <string>

#include "config_loaders/airborne_radar/config_loader.h"
#include "config_loaders/electro_optical/config_loader.h"
#include "config_loaders/electronic_warfare/config_loader.h"
#include "config_loaders/remote_identification_radar/config_loader.h"
#include "config_loaders/sar/config_loader.h"
#include "config_loaders/sbirs_sensor/config_loader.h"
#include "components/ecm_sensor_component.h"

namespace component_attachment {
namespace app {

namespace {

void ResolveRirDatabasePath(remote_identification_radar::config::RirSessionConfig* config) {
  if (config == nullptr) {
    return;
  }
#if defined(CA_RIR_DATABASE_PATH)
  config->policy.recognition.database_path = CA_RIR_DATABASE_PATH;
#else
  std::string& path = config->policy.recognition.database_path;
  if (path.empty()) {
    return;
  }
  const bool absolute =
      path[0] == '/' ||
      (path.size() > 1U && path[1] == ':');
  if (!absolute) {
    path = std::string(SCENE_CONFIG_DIR) + "/" + path;
  }
#endif
}

}  // namespace

std::string SceneSlugFromPath(const std::string& scene_path) {
  std::string path = scene_path;
  for (std::size_t i = 0U; i < path.size(); ++i) {
    if (path[i] == '\\') {
      path[i] = '/';
    }
  }
  while (!path.empty() && path[path.size() - 1U] == '/') {
    path.erase(path.size() - 1U);
  }
  const std::size_t slash = path.find_last_of('/');
  const std::string parent = (slash == std::string::npos) ? std::string() : path.substr(0U, slash);
  const std::size_t parent_slash = parent.find_last_of('/');
  const std::string parent_name =
      (parent_slash == std::string::npos) ? parent : parent.substr(parent_slash + 1U);
  if (!parent_name.empty() && parent_name != "." && parent_name != "..") {
    return parent_name;
  }
  const std::string file = (slash == std::string::npos) ? path : path.substr(slash + 1U);
  const std::size_t dot = file.rfind('.');
  return (dot == std::string::npos || dot == 0U) ? file : file.substr(0U, dot);
}

void PrintUsage(const char* program) {
  std::cout << "Usage: " << program
            << " [--scene <path>] [--cycles <n>] [--view-every <n>] [--output-dir <dir>]\n"
            << "  --scene <path>      场景描述文件（默认 <场景目录>/baseline_takeoff_east.json）\n"
            << "  --cycles <n>        仿真周期数（覆盖场景文件，默认场景文件值）\n"
            << "  --view-every <n>    视图摘要间隔（周期求余，覆盖场景 view_log_every_cycles）\n"
            << "  --output-dir <dir>  日志+CSV 目录（默认 " << kDefaultOutputDir
            << "/<场景名>/）\n";
}

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
  if (!examples::LoadRirSessionConfigFromFile(
          SCENE_CONFIG_DIR "/remote_identification_radar.json", &configs.rir, &error)) {
    std::cerr << "Failed to load RIR config: " << error << "\n";
    std::exit(1);
  }
  ResolveRirDatabasePath(&configs.rir);
  configs.ecm = MakeDefaultEcmConfig();
  return configs;
}

electronic_countermeasure::config::EcmSessionConfig MakeDefaultEcmConfig() {
  electronic_countermeasure::config::EcmSessionConfig config;
  config.transmitter_equipment_id = component_attachment::kEcmTransmitterEquipmentId;
  config.channel_count = 1U;
  config.maximum_total_transmit_power_w = 1000.0;
  config.maximum_channel_transmit_power_w = 1000.0;
  config.default_technique = electronic_countermeasure::EcmTechnique::kSpot;
  return config;
}

void ApplySceneOverrides(const SceneData& scene, ComponentAttachmentConfigs* configs) {
  if (configs == nullptr) {
    return;
  }
  // 跨会话时间对齐与视场适配（业务层调参；数据源
  // 为场景文件 eos_scan 块）：
  // - EOS 周期校验要求 dt ≤ 10/frame_rate_hz（帧率 30 → 上限 0.33 s），
  //   演示按 1 s/周期推进 → 帧率覆写为 10 Hz；
  // - 原配置为下视地面监视（视轴下俯 45°），与空中目标场景不匹配 →
  //   覆写为水平扫描；目标在平台正北（平台局部系 az 90°）→ 扫描扇区
  //   由场景给出（基线 50°~130°）。
  configs->eos.mission.frame_rate_hz = scene.eos_frame_rate_hz;
  configs->eos.mission.scan_rate_deg_per_sec = scene.eos_scan_rate_deg_per_sec;
  configs->eos.mission.scan_start_az_deg = scene.eos_scan_start_az_deg;
  configs->eos.mission.scan_end_az_deg = scene.eos_scan_end_az_deg;
  configs->eos.mission.scan_center_el_deg = scene.eos_scan_center_el_deg;
  configs->eos.mission.boresight_depression_deg = scene.eos_boresight_depression_deg;
  // SAR 任务几何/链路覆写（数据源为场景文件 sar 块；sar.json 为 100 km
  // 斜距 / 180 m/s 的远程监视档，演示场景需覆写为低空巡航几何）：场景中心
  // → 目标群中心，落在 FD 巡航段的侧方（目标与平台同速东飞时平台飞越场景
  // 中心正南 → squint ≈ 0°，成像窗口覆盖巡航段；起飞/转弯段侧视几何不成立，
  // 被库内 squint 门控拒绝——预期行为）。目标 RCS 仅 2.2/1.4 m²，链路预算
  // 在 10 kW 峰值功率下 SNR ≈ −29 dB（低于 minimum_snr_db）→ 功率提升至
  // 1 MW、天线增益 30 → 40 dBi（SAR 常用量级），SNR ≈ +10 dB 过门限。
  configs->sar.hardware.peak_power_w = scene.sar_peak_power_w;
  configs->sar.hardware.antenna_gain_db = scene.sar_antenna_gain_db;
  configs->sar.policy.max_allowed_squint_angle_deg = scene.sar_max_squint_angle_deg;
  configs->sar.mission.scene_center_latitude_deg = scene.sar_scene_center_latitude_deg;
  configs->sar.mission.scene_center_longitude_deg = scene.sar_scene_center_longitude_deg;
  configs->sar.mission.scene_center_altitude_m = scene.sar_scene_center_altitude_m;
  configs->sar.mission.nominal_slant_range_m = scene.sar_nominal_slant_range_m;
  configs->sar.mission.platform_speed_mps = scene.sar_platform_speed_mps;
  // SBIRS 验收量覆写（数据源为场景文件 sbirs_satellite 块）：焦平面几何只进
  // [SbirsAccept] 验收日志的脱靶量映射（x=f·tanΔaz，米+像素双口径），连续
  // 命中门为宽→窄切换前置条件（缺省 1 = 单次命中即调度，行为不变）。验收
  // 事件流需 configure 加 -DONEQ_ENABLE_SBIRS_ACCEPTANCE_LOG=ON（默认关闭，
  // 关闭时宏与派生计算一并剪除），落盘于 1q_library.log。
  configs->sbirs.hardware.focal_length_m =
      static_cast<float>(scene.sbirs_focal_length_m);
  configs->sbirs.hardware.detector_pixel_pitch_m =
      static_cast<float>(scene.sbirs_detector_pixel_pitch_m);
  configs->sbirs.policy.scheduler.wide_to_narrow_required_consecutive_hits =
      scene.sbirs_wide_to_narrow_required_consecutive_hits;
}

}  // namespace app
}  // namespace component_attachment
