/**
 * @file EosSessionConfigBuilder.h
 * @brief EOS 分域语义式会话配置构造器。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_BUILDER_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_BUILDER_H_

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/config/EosSessionConfig.h"

namespace electro_optical_sensor {
namespace config {

/**
 * @brief EosMissionProfile 表示 EOS 任务剖面语义档位。
 *
 * 选择剖面后 Builder 在 Build() 时自动填写 FOV、扫描率、帧率、
 * 工作模式及探测信噪比门限，免去逐项手工配置。
 */
enum class ONEQ_API EosMissionProfile {
  kWideAreaSearch = 0,    /**< 大范围搜索：Fused，12°×8°，30°/s，15Hz，snr=6dB */
  kLongRangeSurveillance, /**< 远程监视：InfraredOnly，3°×2°，10°/s，10Hz，snr=3dB */
  kHighResolutionTrack    /**< 高精度跟踪：Fused，1.5°×1°，5°/s，60Hz，snr=2dB */
};

/**
 * @brief EosHardwareProfile 表示 EOS 硬件规格语义档位。
 *
 * 选择档位后 Builder 在 Build() 时自动填写波长范围、光学口径、
 * 探测器比探测率等硬件参数。
 */
enum class ONEQ_API EosHardwareProfile {
  kStandardMidWaveIR = 0,  /**< 标准中波红外：3-5μm，0.2m 口径，D*=1e10 */
  kLongRangeLargeAperture, /**< 远程大口径：3-5μm，0.4m 口径，D*=2e10 */
  kWideAreaCompact         /**< 广域紧凑：8-12μm，0.1m 口径，D*=5e9 */
};

/**
 * @brief EosSessionConfigBuilder 提供初始化配置语义积木。
 * @note 该构造器只表达 mission profile、hardware profile 与 environment
 *       preset/model 等高层语义。细粒度工程参数应直接编辑
 *       config::EosSessionConfig 四域字段。
 * @note 推荐路径：
 *       会话初始化优先使用本构造器；
 *       运行期热更新统一使用 EosRuntimeConfigBuilder。
 */
class ONEQ_API EosSessionConfigBuilder {
 public:
  class MissionEditor;
  class HardwareEditor;
  class EnvironmentEditor;
  class PolicyEditor;

  explicit EosSessionConfigBuilder(const config::EosSessionConfig& config = {}) noexcept
      : config_(config) {}

  EosSessionConfigBuilder& WithSessionConfig(const config::EosSessionConfig& config) noexcept {
    config_ = config;
    return *this;
  }

  MissionEditor Mission() noexcept;
  HardwareEditor Hardware() noexcept;
  EnvironmentEditor Environment() noexcept;
  PolicyEditor Policy() noexcept;

  /**
   * @brief 将语义档位翻译为配置字段，生成最终会话配置。
   *
   * 如果通过 Editor 设置了 Profile 枚举，Build() 会将语义设定翻译为
   * mission / hardware / policy.detection 中的对应字段。
   *
   * @note Profile 覆盖优先级：Build() 先整体拷贝 `config_`（Policy() 的直接赋值包含
   *       其中），随后才应用 Mission Profile 的跨域翻译，故 Mission Profile 对
   *       `policy.detection.minimum_snr_db` 的取值**始终覆盖** Policy() 的同字段赋值，
   *       与各 Editor 的调用顺序无关。若需独立设定 minimum_snr_db，不要在该次 Build()
   *       中设置 Mission Profile（或接受其带来的 snr 取值）。
   *
   * @return 构建完成的 `config::EosSessionConfig`。
   */
  config::EosSessionConfig Build() const noexcept;

 private:
  friend class MissionEditor;
  friend class HardwareEditor;
  friend class EnvironmentEditor;
  friend class PolicyEditor;

  config::EosSessionConfig config_{};
  EosMissionProfile mission_profile_{EosMissionProfile::kWideAreaSearch};
  EosHardwareProfile hardware_profile_{EosHardwareProfile::kStandardMidWaveIR};
  bool mission_profile_dirty_{false};
  bool hardware_profile_dirty_{false};
};

class ONEQ_API EosSessionConfigBuilder::MissionEditor {
 public:
  explicit MissionEditor(EosSessionConfigBuilder* builder) noexcept : builder_(builder) {}

  /** @brief 设置任务剖面语义档位。Build() 时自动翻译为 mission 及 policy.detection 字段。 */
  MissionEditor& WithMissionProfile(EosMissionProfile profile) noexcept {
    builder_->mission_profile_ = profile;
    builder_->mission_profile_dirty_ = true;
    return *this;
  }
  EosSessionConfigBuilder& End() noexcept { return *builder_; }

 private:
  EosSessionConfigBuilder* builder_;
};

class ONEQ_API EosSessionConfigBuilder::EnvironmentEditor {
 public:
  explicit EnvironmentEditor(EosSessionConfigBuilder* builder) noexcept : builder_(builder) {}

  EnvironmentEditor& WithEnvironmentPreset(config::EosEnvironmentPreset preset) noexcept {
    builder_->config_.environment.scenario_config.preset = preset;
    return *this;
  }
  EosSessionConfigBuilder& End() noexcept { return *builder_; }

 private:
  EosSessionConfigBuilder* builder_;
};

class ONEQ_API EosSessionConfigBuilder::HardwareEditor {
 public:
  explicit HardwareEditor(EosSessionConfigBuilder* builder) noexcept : builder_(builder) {}

  /** @brief 设置硬件规格语义档位。Build() 时自动翻译为 hardware 字段。 */
  HardwareEditor& WithHardwareProfile(EosHardwareProfile profile) noexcept {
    builder_->hardware_profile_ = profile;
    builder_->hardware_profile_dirty_ = true;
    return *this;
  }
  EosSessionConfigBuilder& End() noexcept { return *builder_; }

 private:
  EosSessionConfigBuilder* builder_;
};

/**
 * @brief Policy 编辑器，提供策略域字段的直接设置入口。
 *
 * 设计动机：Mission Profile 在 Build() 时会跨域覆写 `policy.detection.minimum_snr_db`，
 * 在此之前 EOS 缺少独立设置该门限的 Editor。本编辑器补齐该缺口，使其与其余模块
 * （AR DetectionEditor / ESR DetectionEditor / SAR ProcessingEditor / SBIRS WithPolicy）
 * 的策略域可达性一致。
 *
 * @note 直接写入 `config_`；Build() 先拷贝 `config_`、再应用 Mission Profile 跨域翻译，
 *       故若同一次 Build() 同时设置了 Mission Profile，其对 minimum_snr_db 的覆写最终
 *       生效（与调用顺序无关）。需要独立取值时，请不要在该次 Build() 中设置 Mission Profile。
 */
class ONEQ_API EosSessionConfigBuilder::PolicyEditor {
 public:
  explicit PolicyEditor(EosSessionConfigBuilder* builder) noexcept : builder_(builder) {}

  /** @brief 设置最小检测信噪比门限（单位：dB）。 */
  PolicyEditor& WithMinimumSnrDb(float minimum_snr_db) noexcept {
    builder_->config_.policy.detection.minimum_snr_db = minimum_snr_db;
    return *this;
  }
  EosSessionConfigBuilder& End() noexcept { return *builder_; }

 private:
  EosSessionConfigBuilder* builder_;
};

inline EosSessionConfigBuilder::MissionEditor EosSessionConfigBuilder::Mission() noexcept {
  return MissionEditor(this);
}

inline EosSessionConfigBuilder::HardwareEditor EosSessionConfigBuilder::Hardware() noexcept {
  return HardwareEditor(this);
}

inline EosSessionConfigBuilder::EnvironmentEditor EosSessionConfigBuilder::Environment() noexcept {
  return EnvironmentEditor(this);
}

inline EosSessionConfigBuilder::PolicyEditor EosSessionConfigBuilder::Policy() noexcept {
  return PolicyEditor(this);
}

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_BUILDER_H_
