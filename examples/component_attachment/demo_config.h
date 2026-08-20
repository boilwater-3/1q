/**
 * @file demo_config.h
 * @brief 自定义实体-组件示例：演示常量与会话配置加载。
 *
 * 与主程序分离的"装配输入"侧：演示常量（周期数/输出目录）与六会话配置
 * 聚合 + JSON 加载。场景业务数据（平台飞行脚本/目标脚本/天基平台/EOS
 * 扫描/SAR 覆写/融合配置/决策门限）已数据化到场景文件（scene_data.h），
 * 本文件保留：输出目录与周期数缺省常量、六会话 JSON 加载（LoadConfigs）、
 * 场景调参覆写应用（ApplySceneOverrides）。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_DEMO_CONFIG_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_DEMO_CONFIG_H_

#include <cstdint>
#include <string>

#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/electro_optical_sensor/config/EosSessionConfig.h"
#include "1q/electronic_countermeasure/EcmTypes.h"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfig.h"
#include "1q/remote_identification_radar/config/RirSessionConfig.h"
#include "1q/sar/config/SarSessionConfig.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"
#include "scene_data.h"

namespace component_attachment {
namespace demo {

constexpr std::uint32_t kNumCycles = 400U;
// 默认输出目录（日志 + CSV）：由 CMake 注入仓库内绝对路径
// （examples/component_attachment/log/，见 CMakeLists.txt 的 CA_DEFAULT_OUTPUT_DIR）；
// 未注入时的回退为相对路径（随运行目录）。
#if defined(CA_DEFAULT_OUTPUT_DIR)
constexpr char kDefaultOutputDir[] = CA_DEFAULT_OUTPUT_DIR;
#else
constexpr char kDefaultOutputDir[] = "examples/component_attachment/log";
#endif

/// 六会话配置聚合（消费方装配输入）。
struct ComponentAttachmentConfigs {
  airborne_radar::config::ArSessionConfig ar{};
  electronic_surveillance_radar::config::EsrSessionConfig esr{};
  electro_optical_sensor::config::EosSessionConfig eos{};
  sbirs_sensor::config::SbirsSessionConfig sbirs{};
  sar::config::SarSessionConfig sar{};
  remote_identification_radar::config::RirSessionConfig rir{};
  electronic_countermeasure::config::EcmSessionConfig ecm{};
};

/// 打印命令行用法。
void PrintUsage(const char* program);

/// 加载六份会话配置（复用各域 config_loader 与 examples/configs/ 同源 JSON）。
ComponentAttachmentConfigs LoadConfigs();

/// 演示层 ECM 默认配置（无独立 JSON；场景 ecm.enabled 时挂载）。
electronic_countermeasure::config::EcmSessionConfig MakeDefaultEcmConfig();

/// 场景业务调参覆写（EOS 扫描/帧率 + SAR 任务几何/链路）：在 LoadConfigs()
/// 之后应用，数据源为场景文件（scene_data.h），替代原 demo_config 内硬编码
/// 覆写。场景文件缺省块不覆写对应字段（默认值 = 历史覆写值）。
void ApplySceneOverrides(const SceneData& scene, ComponentAttachmentConfigs* configs);

}  // namespace demo
}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_DEMO_CONFIG_H_
