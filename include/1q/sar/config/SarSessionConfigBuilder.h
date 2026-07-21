/**
 * @file SarSessionConfigBuilder.h
 * @brief SAR 分域语义式会话配置构造器。
 */

#ifndef ONEQ_SAR_CONFIG_SAR_SESSION_CONFIG_BUILDER_H_
#define ONEQ_SAR_CONFIG_SAR_SESSION_CONFIG_BUILDER_H_

#include "1q/api.hpp"
#include "1q/sar/config/SarSessionConfig.h"

namespace sar {
namespace config {

/**
 * @brief SarMissionProfile 表示 SAR 任务剖面语义档位。
 *
 * 选择剖面后 Builder 在 Build() 时自动填写标称斜距、平台速度、
 * 合成孔径时间、方位脉冲数与期望分辨率，免去逐项手工配置。
 */
enum class ONEQ_API SarMissionProfile {
  kStripmapSurvey = 0,       /**< 条带普查：斜距 15km，1.5m 分辨率，1024 脉冲 */
  kHighResolutionImaging,    /**< 高分辨成像：斜距 10km，0.5m 分辨率，2048 脉冲 */
  kLongRangeSurveillance     /**< 远程监视：斜距 50km，3.0m 分辨率，512 脉冲 */
};

/**
 * @brief SarProcessingProfile 表示 SAR 处理流水线语义档位。
 *
 * 选择档位后 Builder 在 Build() 时自动填写 policy 域的 raw echo /
 * range compression / L1 RDA / L2 运动补偿 / L3 BP 开关与图像保留策略。
 */
enum class ONEQ_API SarProcessingProfile {
  kRawEchoOnly = 0,       /**< 仅生回波：只开 raw echo generation */
  kRangeCompressedL1,     /**< 距离压缩+L1 RDA：开 raw echo + range compression + L1 */
  kL3Backprojection       /**< L3 BP 路径：开 raw echo + range compression + L3 BP */
};

/**
 * @brief SarSessionConfigBuilder 提供初始化配置语义积木。
 * @note 推荐路径：
 *       会话初始化优先使用本构造器表达 mission/processing 语义；
 *       运行期热更新统一使用 SarRuntimeConfigBuilder；
 *       需要直接编辑四域细项时使用 SarSessionConfig 直接字段赋值；
 *       最终配置校验使用 ValidateSarSessionConfig。
 */
class ONEQ_API SarSessionConfigBuilder {
 public:
  class MissionEditor;
  class ProcessingEditor;

  explicit SarSessionConfigBuilder(const config::SarSessionConfig& config = {})
      : config_(config) {}

  SarSessionConfigBuilder& WithSessionConfig(const config::SarSessionConfig& config) {
    config_ = config;
    return *this;
  }
  MissionEditor Mission();
  ProcessingEditor Processing();

  /**
   * @brief 将语义档位翻译为配置字段，生成最终会话配置。
   *
   * 如果通过 Editor 设置了 Profile 枚举，Build() 会将语义设定翻译为
   * mission / policy 中的对应字段。未由 Profile 管理的 baseline 字段保持原值。
   *
   * @return 构建完成的 `config::SarSessionConfig`。
   */
  config::SarSessionConfig Build() const noexcept;

 private:
  friend class MissionEditor;
  friend class ProcessingEditor;

  config::SarSessionConfig config_{};
  SarMissionProfile mission_profile_{SarMissionProfile::kStripmapSurvey};
  SarProcessingProfile processing_profile_{SarProcessingProfile::kRawEchoOnly};
  bool mission_profile_dirty_{false};
  bool processing_profile_dirty_{false};
};

class ONEQ_API SarSessionConfigBuilder::MissionEditor {
 public:
  explicit MissionEditor(SarSessionConfigBuilder* builder) : builder_(builder) {}

  /** @brief 设置任务剖面语义档位。Build() 时自动翻译为 mission 字段。 */
  MissionEditor& WithMissionProfile(SarMissionProfile profile) {
    builder_->mission_profile_ = profile;
    builder_->mission_profile_dirty_ = true;
    return *this;
  }
  SarSessionConfigBuilder& End() { return *builder_; }

 private:
  SarSessionConfigBuilder* builder_;
};

class ONEQ_API SarSessionConfigBuilder::ProcessingEditor {
 public:
  explicit ProcessingEditor(SarSessionConfigBuilder* builder) : builder_(builder) {}

  /** @brief 设置处理流水线语义档位。Build() 时自动翻译为 policy 字段。 */
  ProcessingEditor& WithProcessingProfile(SarProcessingProfile profile) {
    builder_->processing_profile_ = profile;
    builder_->processing_profile_dirty_ = true;
    return *this;
  }
  SarSessionConfigBuilder& End() { return *builder_; }

 private:
  SarSessionConfigBuilder* builder_;
};

inline SarSessionConfigBuilder::MissionEditor SarSessionConfigBuilder::Mission() {
  return MissionEditor(this);
}

inline SarSessionConfigBuilder::ProcessingEditor SarSessionConfigBuilder::Processing() {
  return ProcessingEditor(this);
}

}  // namespace config
}  // namespace sar

#endif  // ONEQ_SAR_CONFIG_SAR_SESSION_CONFIG_BUILDER_H_
