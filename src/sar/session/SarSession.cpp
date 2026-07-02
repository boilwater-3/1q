#include "1q/sar/session/SarSession.h"

#include <algorithm>
#include <deque>
#include <cmath>
#include <memory>
#include <utility>

#include "sar/geometry/SarGeometry.h"
#include "sar/runtime/PulseRingBuffer.h"
#include "sar/session/SarFocusedImageAssembler.h"
#include "sar/session/SarImagingExecutor.h"
#include "sar/session/SarDiagnosticUtils.h"
#include "sar/session/SarRawHistoryBuilder.h"
#include "sar/session/SarRuntimeConfigResolver.h"
#include "sar/session/SarRuntimeConfigValidation.h"
#include "sar/signal/SarWaveform.h"

namespace sar {
namespace session {

namespace {

void ApplyDiagnosticsPolicy(const config::SarPolicyConfig& policy, SarCycleResult* result) {
  if (policy.enable_diagnostics || result == nullptr) {
    return;
  }
  SarDiagnosticIssueList errors;
  for (const SarDiagnosticIssue& issue : result->diagnostics) {
    if (issue.severity == SarDiagnosticSeverity::kError) {
      errors.push_back(issue);
    }
  }
  result->diagnostics.swap(errors);
}

// 检测聚焦图像峰值退化：若本周期产出了聚焦图像但峰值功率 <= 0，说明回波/聚焦完全失败
// （如目标全部落在采样窗外、斜距严重错配导致回波空、采样窗口根本装不下脉冲宽度）。
// 防御 SarRawHistoryBuilder 已发的 sar.raw_echo_clipping / sar.slant_range_mismatch
// warning 的下游后果。退化图像不应被下游消费，故 abort（语义与 snr_below_minimum 一致）。
//
// 排除两类合法的非退化场景：
//   - is_placeholder=true：成像器产出了信号，只是 retain_focused_image=false 未保留像素。
//   - 空场景（无目标）：合法的信号缺失（与 EmptySceneDoesNotTripMinSnrGate 一致）。
bool HasDegenerateImagePeak(const SarCycleResult& result, const SarCycleInput& input) {
  if (result.focused_image.is_placeholder) {
    return false;
  }
  if (input.point_targets.empty()) {
    return false;
  }
  if (result.output_frame.has_l1_image || result.output_frame.has_l3_bp_image) {
    for (std::size_t i = 0U; i < result.focused_image.real_values.size(); ++i) {
      const double power = result.focused_image.real_values[i] * result.focused_image.real_values[i] +
                           result.focused_image.imaginary_values[i] *
                               result.focused_image.imaginary_values[i];
      if (power > 0.0) {
        return false;  // 至少一个非零像素 → 非退化
      }
    }
    return true;  // 有目标但全部像素功率为 0 → 退化
  }
  return false;
}

}  // namespace

struct SarSession::Impl {
  explicit Impl(const config::SarSessionConfig& initial_config)
      : runtime_config(initial_config),
        raw_pulse_buffer(std::max<std::size_t>(initial_config.mission.azimuth_pulse_count, 1U)) {}

  config::SarSessionConfig runtime_config;
  runtime::PulseRingBuffer raw_pulse_buffer;
  std::deque<geometry::PlatformPulseState> ideal_trajectory_buffer;
  std::deque<geometry::PlatformPulseState> actual_trajectory_buffer;
  std::uint64_t next_pulse_id{0U};
  double pulse_fraction_carry{0.0};
  SarOutputFrame previous_output{};
  bool has_previous_output{false};
};

SarSession::SarSession(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

SarSession::SarSession() : impl_(new Impl(config::SarSessionConfig{})) {}

SarSession::~SarSession() noexcept = default;
SarSession::SarSession(SarSession&&) noexcept = default;
SarSession& SarSession::operator=(SarSession&&) noexcept = default;

SarSession SarSession::Create(const config::SarSessionConfig& config) {
  return SarSession(std::unique_ptr<SarSession::Impl>(new SarSession::Impl(config)));
}

SarSession SarSession::CreateWithValidation(const config::SarSessionConfig& config,
                                            config::ValidationIssueList* issues) {
  const config::ValidationIssueList found = config::ValidateSarSessionConfig(config);
  if (issues != nullptr) {
    *issues = found;
  }
  return Create(config);
}

SarOutputFrame SarSession::Step(const SarCycleInput& input) {
  return StepWithResult(input).output_frame;
}

SarCycleResult SarSession::StepWithResult(const SarCycleInput& input) {
  SarCycleResult result;
  result.input_cycle_index = input.cycle_index;
  result.output_frame.cycle_index = input.cycle_index;
  const auto finish = [this, &result]() -> SarCycleResult {
    ApplyDiagnosticsPolicy(impl_->runtime_config.policy, &result);
    return result;
  };

  if (input.dt_sec <= 0.0) {
    RecordAbort(&result, "invalid_dt_sec", "SAR cycle dt_sec must be positive.");
    if (impl_->has_previous_output) {
      result.output_frame = impl_->previous_output;
      result.reused_previous_output = true;
    }
    return finish();
  }

  InitializeOutputFrameMetadata(impl_->runtime_config, &result.output_frame);

  const bool has_external_raw_iq = HasExternalRawIq(input);
  if (!ValidateRuntimeConfigForStep(impl_->runtime_config, has_external_raw_iq, &result)) {
    if (impl_->has_previous_output) {
      result.output_frame = impl_->previous_output;
      result.reused_previous_output = true;
    }
    return finish();
  }

  signal::LfmWaveform waveform;
  signal::ComplexVector matched_filter;
  if (!BuildWaveformAndFilter(impl_->runtime_config, &waveform, &matched_filter)) {
    RecordAbort(&result, "waveform_generation_failed", "SAR failed to generate LFM waveform.");
    return finish();
  }

  signal::ComplexMatrix raw_history;
  if (impl_->runtime_config.policy.enable_raw_echo_generation) {
    if (has_external_raw_iq) {
      if (!BuildExternalRawIqHistory(impl_->runtime_config, input, &raw_history,
                                     &impl_->ideal_trajectory_buffer,
                                     &impl_->actual_trajectory_buffer, &result)) {
        return finish();
      }
    } else {
      if (!BuildRawPulseHistory(
              impl_->runtime_config, input, waveform.samples, &impl_->raw_pulse_buffer,
              &impl_->next_pulse_id, &impl_->pulse_fraction_carry, &raw_history,
              &impl_->ideal_trajectory_buffer, &impl_->actual_trajectory_buffer, &result)) {
        return finish();
      }
    }
    const double estimated_snr_db = EstimateRawHistorySnrDb(raw_history);
    MarkRawEchoStage(&result.output_frame, estimated_snr_db);
    if (std::isfinite(result.output_frame.estimated_snr_db) &&
        result.output_frame.estimated_snr_db < impl_->runtime_config.policy.minimum_snr_db) {
      RecordAbort(&result, "snr_below_minimum",
                  "SAR estimated SNR is below the configured minimum valid SNR.");
      return finish();
    }
  }
  if (impl_->runtime_config.policy.enable_range_compression) {
    MarkRangeCompressionStage(&result.output_frame);
  }
  if (impl_->runtime_config.policy.enable_l1_rda_imaging) {
    if (!ExecuteL1RdaImaging(impl_->runtime_config, raw_history, matched_filter,
                             impl_->ideal_trajectory_buffer, impl_->actual_trajectory_buffer,
                             &result)) {
      return finish();
    }
  }
  if (impl_->runtime_config.policy.enable_l3_bp_imaging) {
    if (!ExecuteL3BpImaging(impl_->runtime_config, raw_history, matched_filter,
                            impl_->actual_trajectory_buffer, &result)) {
      return finish();
    }
  }

  // 峰值退化检测：聚焦图像全零功率说明回波/聚焦完全失败（回波落窗外、采样窗口装不下
  // 脉冲宽度、或斜距严重错配）。退化图像不应被下游消费，abort 并复用上一帧。
  if (HasDegenerateImagePeak(result, input)) {
    RecordAbort(&result, "degenerate_image_peak",
                "SAR focused image has zero peak power; the echo/focusing pipeline produced no "
                "signal. Check sar.raw_echo_clipping and sar.slant_range_mismatch diagnostics.");
    return finish();
  }

  result.executed_this_cycle = true;
  impl_->previous_output = result.output_frame;
  impl_->has_previous_output = true;
  return finish();
}

void SarSession::ApplyRuntimeConfig(const config::SarRuntimeConfigPatch& patch) {
  (void)TryApplyRuntimeConfig(patch);
}

bool SarSession::TryApplyRuntimeConfig(const config::SarRuntimeConfigPatch& patch) {
  // 立即提交类（见 docs/common/contract.md「运行期配置提交策略」）：调用即生效、单向
  // 落定、无 session 层回滚。SAR 无跨周期累积状态（每 Step 全量重建），无需回滚；
  // 执行期合法性由 ValidateRuntimeConfigForStep 在 Step 内 gate。
  const SarRuntimeConfigResolveResult resolved =
      ResolveSarRuntimeConfigPatch(impl_->runtime_config, patch);
  if (!resolved.has_requested_update || !resolved.is_valid) {
    return false;
  }
  impl_->runtime_config = resolved.next_config;
  return true;
}

}  // namespace session
}  // namespace sar
