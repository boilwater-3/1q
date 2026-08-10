/**
 * @file demo_config.cpp
 * @brief 演示常量与配置加载实现（见 demo_config.h）。
 */

#include "demo_config.h"

#include <cstdlib>
#include <iostream>

#include "config_loaders/airborne_radar/config_loader.h"
#include "config_loaders/electro_optical/config_loader.h"
#include "config_loaders/electronic_warfare/config_loader.h"
#include "config_loaders/sar/config_loader.h"
#include "config_loaders/sbirs_sensor/config_loader.h"

namespace component_attachment {
namespace demo {

void PrintUsage(const char* program) {
  std::cout << "Usage: " << program
            << " [--scene <path>] [--cycles <n>] [--output-dir <dir>]\n"
            << "  --scene <path>      场景描述文件（默认 <场景目录>/baseline_takeoff_east.json）\n"
            << "  --cycles <n>        仿真周期数（覆盖场景文件，默认场景文件值）\n"
            << "  --output-dir <dir>  CSV 输出目录（默认 " << kDefaultOutputDir << "）\n";
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
  return configs;
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
}

}  // namespace demo
}  // namespace component_attachment
