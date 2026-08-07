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
  std::cout << "Usage: " << program << " [--cycles <n>] [--output-dir <dir>]\n"
            << "  --cycles <n>        仿真周期数（默认 " << kNumCycles << "）\n"
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
  //   scene_script kTargetScript），巡航段平台飞越场景中心正南时 squint
  //   ≈ 0°，成像窗口覆盖巡航段（10° 门限下 ±2.3 km 纵向窗口）；起飞/转弯
  //   段（cycle 1-330）侧视几何不成立，被库内 squint 门控拒绝（预期行为，
  //   产品事件只在窗口期产生）。目标 RCS 仅 2.2/1.4 m²，链路预算在 10 kW
  //   峰值功率下 SNR ≈ −29 dB（低于 minimum_snr_db）→ 功率提升至 1 MW、
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

}  // namespace demo
}  // namespace component_attachment
