/**
 * @file SarSessionConfigBuilder.h
 * @brief SAR 分域语义式会话配置构造器。
 */

#ifndef ONEQ_SAR_CONFIG_SAR_SESSION_CONFIG_BUILDER_H_
#define ONEQ_SAR_CONFIG_SAR_SESSION_CONFIG_BUILDER_H_

#include <string>
#include <vector>

#include "1q/api.hpp"
#include "1q/sar/config/SarSessionConfig.h"

namespace sar {
namespace config {

/**
 * @brief ConfigValidationCode 表示构造器校验问题编码。
 */
enum class ConfigValidationCode {
  kNone = 0,
  kCarrierFrequencyNotPositive,    /**< 载频 <= 0 */
  kBandwidthNotPositive,           /**< 带宽 <= 0 */
  kPulseRepetitionFrequencyNotPositive, /**< PRF <= 0 */
  kSampleRateNotPositive,          /**< 采样率 <= 0 */
  kAntennaLengthNotPositive,       /**< 方位孔径长度 <= 0 */
  kNominalSlantRangeNotPositive,   /**< 标称斜距 <= 0 */
  kPlatformSpeedNotPositive,       /**< 平台速度 <= 0 */
  kAzimuthPulseCountZero,          /**< 方位脉冲数为 0 */
  kRangeSampleCountZero,           /**< 距离采样点数为 0 */
  kDesiredResolutionNotPositive    /**< 期望分辨率 <= 0 */
};

/**
 * @brief ConfigValidationIssue 描述一条构造器校验结果。
 */
struct ConfigValidationIssue {
  ConfigValidationCode code{ConfigValidationCode::kNone}; /**< 问题编码 */
  std::string field{};   /**< 关联字段名 */
  std::string message{}; /**< 简短说明 */
};

/** @brief ValidationIssueList 表示构造器校验问题列表。 */
using ValidationIssueList = std::vector<ConfigValidationIssue>;

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
  kFullPipelineL3         /**< 全流水线：开 raw echo + range compression + L2 + L3 */
};

/**
 * @brief SarSessionConfigBuilder 提供初始化配置语义积木。
 * @note 推荐路径：
 *       会话初始化优先使用本构造器表达 mission/processing 语义；
 *       运行期热更新统一使用 SarRuntimeConfigBuilder；
 *       需要直接编辑四域细项时使用直接字段赋值。
 */
class ONEQ_API SarSessionConfigBuilder {
 public:
  class MissionEditor;
  class ProcessingEditor;
  class EnvironmentEditor;

  explicit SarSessionConfigBuilder(const config::SarSessionConfig& config = {})
      : config_(config) {}

  SarSessionConfigBuilder& WithSessionConfig(const config::SarSessionConfig& config) {
    config_ = config;
    return *this;
  }
  MissionEditor Mission();
  ProcessingEditor Processing();
  EnvironmentEditor Environment();

  /**
   * @brief 将语义档位翻译为配置字段，生成最终会话配置。
   *
   * 如果通过 Editor 设置了 Profile 枚举，Build() 会将语义设定翻译为
   * mission / policy 中的对应字段。直接字段 setter 的赋值
   * 在 Profile 应用之后被覆盖——Profile 是高层语义入口，直接 setter
   * 适合在 Profile 未设置的场景下做精细调整。
   *
   * @return 构建完成的 `config::SarSessionConfig`。
   */
  config::SarSessionConfig Build() const noexcept;

  /**
   * @brief 校验当前 Builder 状态的合法性，用于构造完成前的早期反馈。
   *
   * 检查项包括：
   * - 载频、带宽、PRF、采样率为正；
   * - 方位孔径长度、标称斜距、平台速度为正；
   * - 方位脉冲数、距离采样点数非零；
   * - 期望分辨率（方位/地距）为正。
   *
   * @note 本校验仅为构造期早期反馈，完整的运行期校验仍由
   *       RuntimeConfigResolver 执行。`Build()` 的行为不受校验结果影响。
   * @return 按发现顺序返回的校验问题列表。
   */
  ValidationIssueList Validate() const noexcept;

 private:
  friend class MissionEditor;
  friend class ProcessingEditor;
  friend class EnvironmentEditor;

  config::SarSessionConfig config_{};
  SarMissionProfile mission_profile_{SarMissionProfile::kStripmapSurvey};
  SarProcessingProfile processing_profile_{SarProcessingProfile::kRawEchoOnly};
  bool mission_profile_dirty_{false};
  bool processing_profile_dirty_{false};
};

class ONEQ_API SarSessionConfigBuilder::MissionEditor {
 public:
  explicit MissionEditor(SarSessionConfigBuilder* builder) : builder_(builder) {}

  MissionEditor& WithSceneCenter(double lat_deg, double lon_deg, double alt_m) {
    builder_->config_.mission.scene_center_latitude_deg = lat_deg;
    builder_->config_.mission.scene_center_longitude_deg = lon_deg;
    builder_->config_.mission.scene_center_altitude_m = alt_m;
    return *this;
  }
  MissionEditor& WithNominalSlantRangeM(double value) {
    builder_->config_.mission.nominal_slant_range_m = value;
    return *this;
  }
  MissionEditor& WithSyntheticApertureTimeS(double value) {
    builder_->config_.mission.synthetic_aperture_time_s = value;
    return *this;
  }
  MissionEditor& WithPlatformSpeedMps(double value) {
    builder_->config_.mission.platform_speed_mps = value;
    return *this;
  }
  MissionEditor& WithAzimuthPulseCount(std::uint32_t value) {
    builder_->config_.mission.azimuth_pulse_count = value;
    return *this;
  }
  MissionEditor& WithRangeSampleCount(std::uint32_t value) {
    builder_->config_.mission.range_sample_count = value;
    return *this;
  }
  MissionEditor& WithDesiredGroundRangeResolutionM(double value) {
    builder_->config_.mission.desired_ground_range_resolution_m = value;
    return *this;
  }
  MissionEditor& WithDesiredAzimuthResolutionM(double value) {
    builder_->config_.mission.desired_azimuth_resolution_m = value;
    return *this;
  }
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

  ProcessingEditor& EnableRawEchoGeneration(bool enable) {
    builder_->config_.policy.enable_raw_echo_generation = enable;
    return *this;
  }
  ProcessingEditor& EnableRangeCompression(bool enable) {
    builder_->config_.policy.enable_range_compression = enable;
    return *this;
  }
  ProcessingEditor& EnableL1RdaImaging(bool enable) {
    builder_->config_.policy.enable_l1_rda_imaging = enable;
    return *this;
  }
  ProcessingEditor& EnableL2MotionCompensation(bool enable) {
    builder_->config_.policy.enable_l2_motion_compensation = enable;
    return *this;
  }
  ProcessingEditor& EnableL3BpImaging(bool enable) {
    builder_->config_.policy.enable_l3_bp_imaging = enable;
    return *this;
  }
  ProcessingEditor& RetainFocusedImage(bool retain) {
    builder_->config_.policy.retain_focused_image = retain;
    return *this;
  }
  ProcessingEditor& WithMinimumSnrDb(double value) {
    builder_->config_.policy.minimum_snr_db = value;
    return *this;
  }
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

class ONEQ_API SarSessionConfigBuilder::EnvironmentEditor {
 public:
  explicit EnvironmentEditor(SarSessionConfigBuilder* builder) : builder_(builder) {}

  EnvironmentEditor& WithEnvironmentDefault(const config::SarEnvironmentConfig& config) {
    builder_->config_.environment = config;
    return *this;
  }
  EnvironmentEditor& WithTerrainReferenceAltitudeM(double value) {
    builder_->config_.environment.terrain_reference_altitude_m = value;
    return *this;
  }
  EnvironmentEditor& WithAtmosphericLossDbPerKm(double value) {
    builder_->config_.environment.atmospheric_loss_db_per_km = value;
    return *this;
  }
  EnvironmentEditor& WithSurfaceBackscatterSigma0Db(double value) {
    builder_->config_.environment.surface_backscatter_sigma0_db = value;
    return *this;
  }
  EnvironmentEditor& EnableAtmosphericAttenuation(bool enable) {
    builder_->config_.environment.enable_atmospheric_attenuation = enable;
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

inline SarSessionConfigBuilder::EnvironmentEditor SarSessionConfigBuilder::Environment() {
  return EnvironmentEditor(this);
}

}  // namespace config
}  // namespace sar

#endif  // ONEQ_SAR_CONFIG_SAR_SESSION_CONFIG_BUILDER_H_
