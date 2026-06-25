/**
 * @file SarRuntimeConfigBuilder.h
 * @brief SAR 运行期补丁构造器。
 *
 * 补齐 SAR 域此前缺失的运行期补丁构造器：EOS/ESR/AR 三域均已提供链式 Builder，
 * 避免 SAR 用户被迫手写 `config::SarRuntimeConfigPatch` 的 `has_*` 标志位。
 * 风格对齐 EosRuntimeConfigBuilder（With* 动词、noexcept、以现有 patch 起步）。
 */

#ifndef ONEQ_SAR_CONFIG_SAR_RUNTIME_CONFIG_BUILDER_H_
#define ONEQ_SAR_CONFIG_SAR_RUNTIME_CONFIG_BUILDER_H_

#include "1q/sar/config/SarRuntimeConfigPatch.h"

namespace sar {
namespace config {

/**
 * @brief SarRuntimeConfigBuilder 提供运行期补丁的链式构造。
 * @note 推荐路径：会话创建后的参数热更新统一通过本构造器生成补丁，
 *       避免直接手写 `config::SarRuntimeConfigPatch` 的 `has_*` 标志位。
 */
class ONEQ_API SarRuntimeConfigBuilder {
 public:
  explicit SarRuntimeConfigBuilder(
      const config::SarRuntimeConfigPatch& patch = {}) noexcept : patch_(patch) {}

  SarRuntimeConfigBuilder& WithRuntimeConfigPatch(
      const config::SarRuntimeConfigPatch& patch) noexcept {
    patch_ = patch;
    return *this;
  }

  SarRuntimeConfigBuilder& WithEnableRawEchoGeneration(bool value) noexcept {
    patch_.has_enable_raw_echo_generation = true;
    patch_.enable_raw_echo_generation = value;
    return *this;
  }

  SarRuntimeConfigBuilder& WithEnableRangeCompression(bool value) noexcept {
    patch_.has_enable_range_compression = true;
    patch_.enable_range_compression = value;
    return *this;
  }

  SarRuntimeConfigBuilder& WithEnableL1RdaImaging(bool value) noexcept {
    patch_.has_enable_l1_rda_imaging = true;
    patch_.enable_l1_rda_imaging = value;
    return *this;
  }

  SarRuntimeConfigBuilder& WithRetainRawPhaseHistory(bool value) noexcept {
    patch_.has_retain_raw_phase_history = true;
    patch_.retain_raw_phase_history = value;
    return *this;
  }

  SarRuntimeConfigBuilder& WithRetainFocusedImage(bool value) noexcept {
    patch_.has_retain_focused_image = true;
    patch_.retain_focused_image = value;
    return *this;
  }

  SarRuntimeConfigBuilder& WithMinimumSnrDb(double value) noexcept {
    patch_.has_minimum_snr_db = true;
    patch_.minimum_snr_db = value;
    return *this;
  }

  config::SarRuntimeConfigPatch Build() const noexcept { return patch_; }

 private:
  config::SarRuntimeConfigPatch patch_{};
};

}  // namespace config
}  // namespace sar

#endif  // ONEQ_SAR_CONFIG_SAR_RUNTIME_CONFIG_BUILDER_H_
