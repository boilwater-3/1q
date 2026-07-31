/**
 * @file EsrSessionConfigBuilder.h
 * @brief ESR 分域语义式会话配置构造器。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_BUILDER_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_BUILDER_H_

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfig.h"

namespace electronic_surveillance_radar {
namespace config {

/**
 * @brief EsrMissionProfile 表示 ESR 任务剖面语义档位。
 *
 * 选择剖面后 Builder 在 Build() 时自动填写 work_mode、scan_rate_hz
 * 及扫描角度边界，免去逐项手工配置。
 */
enum class ONEQ_API EsrMissionProfile {
  kElectronicOrderOfBattle = 0, /**< 电子战斗序列采集：ESM 模式，2Hz 快扫，±60° */
  kPrecisionEmitterAnalysis,    /**< 精确辐射源分析：HGESM 模式，0.5Hz 慢扫，±30° */
  kThreatWarning                /**< 威胁告警：RWR 模式，5Hz 快扫，±60° */
};

/**
 * @brief EsrSensitivityProfile 表示探测灵敏度语义档位。
 *
 * 选择档位后 Builder 在 Build() 时自动填写 SNR 门限、脉冲积累数、
 * 虚警概率、门限缩放系数和统计检测开关（enable_statistical_detection=true）。
 */
enum class ONEQ_API EsrSensitivityProfile {
  kStandard = 0,    /**< 均衡：min_snr=6dB，pulse=8，pfa=1e-6，统计检测开 */
  kHighSensitivity, /**< 高灵敏（远距弱信号）：min_snr=3dB，pulse=16，pfa=5e-6，统计检测开 */
  kRobust           /**< 抗干扰（复杂电磁环境）：min_snr=10dB，pulse=4，pfa=1e-7，统计检测开 */
};

/**
 * @brief EsrSessionConfigBuilder 提供初始化配置语义积木。
 * @note 推荐路径：
 *       会话初始化优先使用本构造器表达 mission/sensitivity/environment 语义；
 *       运行期热更新统一使用 EsrRuntimeConfigBuilder；
 *       需要直接编辑四域细项时使用直接字段赋值。
 */
class ONEQ_API EsrSessionConfigBuilder {
 public:
  class MissionEditor;
  class DetectionEditor;
  class EnvironmentEditor;

  explicit EsrSessionConfigBuilder(const config::EsrSessionConfig& config = {}) : config_(config) {}

  EsrSessionConfigBuilder& WithSessionConfig(const config::EsrSessionConfig& config) {
    config_ = config;
    return *this;
  }
  MissionEditor Mission();
  DetectionEditor Detection();
  EnvironmentEditor Environment();

  /**
   * @brief 将语义档位翻译为配置字段，生成最终会话配置。
   *
   * 如果通过 Editor 设置了 Profile 枚举，Build() 会将语义设定翻译为
   * mission / policy.detection 中的对应字段。
   *
   * @note Profile 会覆盖同一字段上的直接赋值。优先级：mission profile >
   *       sensitivity profile > WithSessionConfig 基础配置。
   *       WithSessionConfig() 不会重置已设置的 profile 标志。
   *
   * @return 构建完成的 `config::EsrSessionConfig`。
   */
  config::EsrSessionConfig Build() const;

 private:
  friend class MissionEditor;
  friend class DetectionEditor;
  friend class EnvironmentEditor;

  config::EsrSessionConfig config_{};
  EsrMissionProfile mission_profile_{EsrMissionProfile::kElectronicOrderOfBattle};
  EsrSensitivityProfile sensitivity_profile_{EsrSensitivityProfile::kStandard};
  bool mission_profile_dirty_{false};
  bool sensitivity_profile_dirty_{false};
};

class ONEQ_API EsrSessionConfigBuilder::MissionEditor {
 public:
  explicit MissionEditor(EsrSessionConfigBuilder* builder) : builder_(builder) {}

  /** @brief 设置任务剖面语义档位。Build() 时自动翻译为 mission 字段。 */
  MissionEditor& WithMissionProfile(EsrMissionProfile profile) {
    builder_->mission_profile_ = profile;
    builder_->mission_profile_dirty_ = true;
    return *this;
  }
  EsrSessionConfigBuilder& End() { return *builder_; }

 private:
  EsrSessionConfigBuilder* builder_;
};

class ONEQ_API EsrSessionConfigBuilder::DetectionEditor {
 public:
  explicit DetectionEditor(EsrSessionConfigBuilder* builder) : builder_(builder) {}

  /** @brief 设置探测灵敏度语义档位。Build() 时自动翻译为 detection policy 字段。 */
  DetectionEditor& WithSensitivityProfile(EsrSensitivityProfile profile) {
    builder_->sensitivity_profile_ = profile;
    builder_->sensitivity_profile_dirty_ = true;
    return *this;
  }
  EsrSessionConfigBuilder& End() { return *builder_; }

 private:
  EsrSessionConfigBuilder* builder_;
};

class ONEQ_API EsrSessionConfigBuilder::EnvironmentEditor {
 public:
  explicit EnvironmentEditor(EsrSessionConfigBuilder* builder) : builder_(builder) {}

  EnvironmentEditor& WithEnvironmentPreset(config::EsrEnvironmentPreset preset) {
    builder_->config_.environment.scenario_config.preset = preset;
    return *this;
  }
  EsrSessionConfigBuilder& End() { return *builder_; }

 private:
  EsrSessionConfigBuilder* builder_;
};

inline EsrSessionConfigBuilder::MissionEditor EsrSessionConfigBuilder::Mission() {
  return MissionEditor(this);
}

inline EsrSessionConfigBuilder::DetectionEditor EsrSessionConfigBuilder::Detection() {
  return DetectionEditor(this);
}

inline EsrSessionConfigBuilder::EnvironmentEditor EsrSessionConfigBuilder::Environment() {
  return EnvironmentEditor(this);
}

}  // namespace config
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SESSION_CONFIG_BUILDER_H_
