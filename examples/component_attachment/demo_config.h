/**
 * @file demo_config.h
 * @brief 自定义实体-组件示例：演示常量与会话配置加载。
 *
 * 与主程序分离的"装配输入"侧：演示常量（周期数/步长/输出目录/巡航参数/
 * 决策门限）与五会话配置聚合 + JSON 加载（含 EOS/SAR 业务调参覆写）。
 * 主程序与 scene_script / demo_output 共用本文件的常量。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_DEMO_CONFIG_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_DEMO_CONFIG_H_

#include <cstdint>
#include <string>

#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/electro_optical_sensor/config/EosSessionConfig.h"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfig.h"
#include "1q/sar/config/SarSessionConfig.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"

namespace component_attachment {
namespace demo {

constexpr std::uint32_t kNumCycles = 400U;
constexpr double kDtSec = 1.0;
constexpr char kDefaultOutputDir[] = "/tmp/component_attachment_viz";
/// 平台巡航高度（m）：c172x 低空巡航量级；目标真值固定在此高度。
/// EOS 探测距离窗 ≈ 高度 / sin(俯仰角)（min/max 2°/1°）→ 400 m 时
/// [11.5, 22.9] km，目标斜距全程稳定在窗内（见 scene_script kTargetScript）。
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
void PrintUsage(const char* program);

/// 加载五份会话配置（复用各域 config_loader 与 examples/configs/ 同源 JSON）。
ComponentAttachmentConfigs LoadConfigs();

}  // namespace demo
}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_DEMO_CONFIG_H_
