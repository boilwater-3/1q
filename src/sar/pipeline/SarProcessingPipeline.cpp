#include "sar/pipeline/SarProcessingPipeline.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>

#include "common/logging/ProjectLog.h"

#include "1q/sar/session/SarIssueCodes.h"
#include "sar/session/SarDiagnosticUtils.h"
#include "sar/session/SarFocusedImageAssembler.h"
#include "sar/session/SarImagingExecutor.h"
#include "sar/session/SarRawHistoryBuilder.h"
#include "sar/signal/SarWaveform.h"
#include "common/numerics/Constants.h"

namespace sar {
namespace pipeline {

namespace {
using oneq::common::numerics::DegToRad;
using oneq::common::numerics::RadToDeg;

constexpr std::uint32_t kPipelineRuntimeStateSchemaVersion = 1U;
constexpr double kEarthRadiusM = 6378137.0;

bool HasDegenerateImagePeak(const session::SarCycleResult& result,
                            const session::SarCycleInput& input) {
  if (result.product.focused_image.is_placeholder) {
    return false;
  }
  if (input.point_targets.empty()) {
    return false;
  }
  // focused_image 是两个独立 vector 的公共 DTO；长度不一致时无法判定峰值退化，
  // 保守跳过检测而不是越界读取 imaginary_values。
  if (result.product.focused_image.real_values.size() !=
      result.product.focused_image.imaginary_values.size()) {
    return false;
  }
  if (result.product.output_frame.has_l1_image || result.product.output_frame.has_l3_bp_image) {
    for (std::size_t i = 0U; i < result.product.focused_image.real_values.size(); ++i) {
      const double power =
          result.product.focused_image.real_values[i] * result.product.focused_image.real_values[i] +
          result.product.focused_image.imaginary_values[i] *
              result.product.focused_image.imaginary_values[i];
      if (power > 0.0) {
        return false;
      }
    }
    return true;
  }
  return false;
}

/**
 * @brief 计算单脉冲平台状态下的孔径 squint 角（deg）。
 * @param[in] pulse 单脉冲平台状态（ENU 局部坐标，position 相对局部原点）。
 * @param[in] scene_center_local_z_m 场景中心在当前局部坐标系中的高度。默认 0
 *            （局部原点即场景中心，如外部输入/回退脉冲路径）。内部生成路径使用
 *            terrain_reference 高度基准，场景中心位于
 *            scene_center_altitude_m - terrain_reference_altitude_m，须显式传入，
 *            否则 LOS 的 z 分量被基准面差偏斜（squint 门限失真）。
 * @return squint = asin(|v·LOS| / (|v|·|LOS|))：视线（平台→场景中心）偏离
 *         正侧视的角度——正侧视（LOS ⊥ 航迹）为 0°、前视/后视（LOS ∥ 航迹）
 *         趋近 90°；速度或距离非正时返回 0（无有效几何，不参与门控）。
 * @note 本定义是"偏离正侧视的角度"，与"视线-航迹夹角"互补为 90°（多数文献
 *       采用后者）；门限语义与场景编排约定见 docs/sar/algorithms.md。
 */
double ComputeSquintAngleDeg(const geometry::PlatformPulseState& pulse,
                             double scene_center_local_z_m = 0.0) {
  const double speed = std::sqrt(pulse.velocity_x_mps * pulse.velocity_x_mps +
                                 pulse.velocity_y_mps * pulse.velocity_y_mps +
                                 pulse.velocity_z_mps * pulse.velocity_z_mps);
  const double los_x = -pulse.position_m.x_m;
  const double los_y = -pulse.position_m.y_m;
  const double los_z = scene_center_local_z_m - pulse.position_m.z_m;
  const double range = std::sqrt(los_x * los_x + los_y * los_y + los_z * los_z);
  if (speed <= 0.0 || range <= 0.0) {
    return 0.0;
  }
  const double along_track_cosine =
      std::abs((pulse.velocity_x_mps * los_x + pulse.velocity_y_mps * los_y +
                pulse.velocity_z_mps * los_z) /
               (speed * range));
  return RadToDeg(std::asin(std::min(1.0, along_track_cosine)));
}

geometry::PlatformPulseState BuildCurrentPlatformPulse(
    const config::SarMissionConfig& mission, const session::SarPlatformState& platform) {
  const double reference_latitude_rad = DegToRad(mission.scene_center_latitude_deg);
  geometry::PlatformPulseState pulse;
  pulse.position_m.x_m =
      oneq::common::numerics::DegToRad(platform.longitude_deg - mission.scene_center_longitude_deg) *
      std::cos(reference_latitude_rad) * kEarthRadiusM;
  pulse.position_m.y_m =
      oneq::common::numerics::DegToRad(platform.latitude_deg - mission.scene_center_latitude_deg) *
      kEarthRadiusM;
  pulse.position_m.z_m = platform.altitude_m - mission.scene_center_altitude_m;
  pulse.velocity_x_mps = platform.velocity_east_mps;
  pulse.velocity_y_mps = platform.velocity_north_mps;
  pulse.velocity_z_mps = -platform.velocity_down_mps;
  return pulse;
}

}  // namespace

struct SarProcessingPipeline::Impl {
  explicit Impl(const config::SarSessionConfig& initial_config)
      : raw_pulse_buffer(std::max<std::size_t>(initial_config.mission.azimuth_pulse_count, 1U)) {}

  runtime::PulseRingBuffer raw_pulse_buffer;
  std::deque<geometry::PlatformPulseState> ideal_trajectory_buffer;
  std::deque<geometry::PlatformPulseState> actual_trajectory_buffer;
  std::uint64_t next_pulse_id{0U};
  double pulse_fraction_carry{0.0};
};

SarProcessingPipeline::SarProcessingPipeline(const config::SarSessionConfig& initial_config)
    : impl_(new Impl(initial_config)) {}

SarProcessingPipeline::~SarProcessingPipeline() = default;

bool SarProcessingPipeline::RunCycle(const config::SarSessionConfig& config,
                                     const session::SarCycleInput& input,
                                     session::SarCycleResult* result) {
  if (result == nullptr) {
    return false;
  }

  signal::LfmWaveform waveform;
  signal::ComplexVector matched_filter;
  if (!session::BuildWaveformAndFilter(config, &waveform, &matched_filter)) {
    session::RecordAbort(result, session::SarPipelineAbortReason::kPipelineExecutionFailed,
                         session::codes::kWaveformGenerationFailed,
                         "SAR failed to generate LFM waveform.");
    return false;
  }

  // ---------------------------------------------------------------------------
  // 成像 squint 门控前置：在 raw echo 生成之前执行，被拒周期不生成 echo。
  // 计算语义与重构前完全一致（docs/sar/algorithms.md §RDA 聚焦）：
  //  - 外部 raw IQ 路径：取输入脉冲状态的最大 squint；
  //  - 内部生成路径：取积累窗孔径轨迹（上一周期缓冲 + 本周期新脉冲，裁剪到
  //    azimuth_pulse_count）的最大 squint——轨迹由 PrepareCycleTrajectory 预生成
  //    （纯函数、µs 级），echo 阶段复用同一轨迹，不二次生成；
  //  - 退化配置（echo 关闭且无外部 IQ）：回退当前平台单脉冲状态。
  // 被拒周期输出帧仅含元数据（cycle_index），不带 raw echo 标记，符合
  // docs/sar/boundaries.md 非执行周期契约；跨周期状态由调用方快照恢复。
  // ---------------------------------------------------------------------------
  bool trajectory_prepared = false;
  std::vector<geometry::PlatformPulseState> prebuilt_ideal_pulses;
  std::vector<geometry::PlatformPulseState> prebuilt_actual_pulses;
  if (config.policy.enable_l1_rda_imaging || config.policy.enable_l3_bp_imaging) {
    double maximum_squint_angle_deg = 0.0;
    if (session::HasExternalRawIq(input) && !input.raw_iq.pulse_states.empty()) {
      for (const session::SarRawIqFrame::PulseState& state : input.raw_iq.pulse_states) {
        geometry::PlatformPulseState pulse;
        pulse.position_m.x_m = state.position_x_m;
        pulse.position_m.y_m = state.position_y_m;
        pulse.position_m.z_m = state.position_z_m;
        pulse.velocity_x_mps = state.velocity_x_mps;
        pulse.velocity_y_mps = state.velocity_y_mps;
        pulse.velocity_z_mps = state.velocity_z_mps;
        maximum_squint_angle_deg =
            std::max(maximum_squint_angle_deg, ComputeSquintAngleDeg(pulse));
      }
    } else if (config.policy.enable_raw_echo_generation) {
      const geometry::PlatformPulseState* previous_actual =
          impl_->actual_trajectory_buffer.empty() ? nullptr
                                                  : &impl_->actual_trajectory_buffer.back();
      if (!session::PrepareCycleTrajectory(config, input, impl_->next_pulse_id,
                                           &impl_->pulse_fraction_carry,
                                           impl_->raw_pulse_buffer.size(), previous_actual,
                                           &prebuilt_ideal_pulses, &prebuilt_actual_pulses,
                                           result)) {
        return false;
      }
      trajectory_prepared = true;
      // 内部生成路径的脉冲高度以 terrain_reference 为基准；场景中心在该坐标系
      // 中位于 scene_center - terrain_reference 高度处，squint 视线须相对真实
      // 场景中心计算（其余分支的局部原点即场景中心，保持默认 0）。
      const double scene_center_local_z_m =
          config.mission.scene_center_altitude_m -
          config.environment.terrain_reference_altitude_m;
      // 积累窗孔径候选 = 上一周期缓冲 + 本周期新脉冲（裁剪到孔径长度），
      // 与重构前 BuildRawPulseHistory 之后的 actual_trajectory_buffer 逐脉冲一致。
      std::deque<geometry::PlatformPulseState> aperture_candidate =
          impl_->actual_trajectory_buffer;
      for (const geometry::PlatformPulseState& pulse : prebuilt_actual_pulses) {
        aperture_candidate.push_back(pulse);
      }
      while (aperture_candidate.size() > config.mission.azimuth_pulse_count) {
        aperture_candidate.pop_front();
      }
      for (const geometry::PlatformPulseState& pulse : aperture_candidate) {
        maximum_squint_angle_deg =
            std::max(maximum_squint_angle_deg,
                     ComputeSquintAngleDeg(pulse, scene_center_local_z_m));
      }
    } else {
      maximum_squint_angle_deg =
          ComputeSquintAngleDeg(BuildCurrentPlatformPulse(config.mission, input.platform));
    }
    if (maximum_squint_angle_deg > config.policy.max_allowed_squint_angle_deg) {
      // 中译：SAR 成像门控拒绝（周期号）：孔径 squint 超限，本周期不生成 echo 与成像。
      // 标识：执行中止路径——输出帧仅带元数据（cycle_index），无任何成像产物；
      //       跨周期状态（缓冲/分数余量）由调用方快照恢复，不被拒绝周期污染。
      session::RecordAbort(result, session::SarPipelineAbortReason::kPipelineExecutionFailed,
                           session::codes::kSquintAngleExceedsLimit,
                           "SAR aperture squint angle exceeds the configured imaging limit.");
      return false;
    }
  }

  signal::ComplexMatrix raw_history;
  if (config.policy.enable_raw_echo_generation) {
    if (session::HasExternalRawIq(input)) {
      if (!session::BuildExternalRawIqHistory(config, input, &raw_history,
                                              &impl_->ideal_trajectory_buffer,
                                              &impl_->actual_trajectory_buffer, result)) {
        return false;
      }
      result->issues.push_back(session::MakeInfoDiagnostic(
          session::codes::kExternalRawIqSnrUnavailable,
          "External raw IQ is already receiver-domain data; hardware link budget and minimum "
          "SNR gating are not reapplied without signal/noise metadata."));
      // 外部 IQ 是接收机域数据，无可估 SNR：以不可估计值标记 raw echo 阶段。
      session::MarkRawEchoStage(&result->product.output_frame,
                                -std::numeric_limits<double>::infinity());
    } else {
      double estimated_snr_db = -std::numeric_limits<double>::infinity();
      if (!session::BuildRawPulseHistory(
              config, input, waveform.samples, &impl_->raw_pulse_buffer, &impl_->next_pulse_id,
              &impl_->pulse_fraction_carry, &raw_history, &impl_->ideal_trajectory_buffer,
              &impl_->actual_trajectory_buffer, &estimated_snr_db, result,
              trajectory_prepared ? &prebuilt_ideal_pulses : nullptr,
              trajectory_prepared ? &prebuilt_actual_pulses : nullptr)) {
        return false;
      }
      session::MarkRawEchoStage(&result->product.output_frame, estimated_snr_db);
      if (std::isfinite(estimated_snr_db) && estimated_snr_db < config.policy.minimum_snr_db) {
        session::RecordAbort(result, session::SarPipelineAbortReason::kPipelineExecutionFailed,
                             session::codes::kSnrBelowMinimum,
                             "SAR estimated SNR is below the configured minimum valid SNR.");
        return false;
      }
    }
  }
  if (config.policy.enable_l1_rda_imaging) {
    if (!session::ExecuteL1RdaImaging(config, raw_history, matched_filter,
                                      impl_->ideal_trajectory_buffer,
                                      impl_->actual_trajectory_buffer, result)) {
      return false;
    }
    result->product.output_frame.has_range_compressed_echo = true;
  }
  if (config.policy.enable_l3_bp_imaging) {
    if (!session::ExecuteL3BpImaging(config, raw_history, matched_filter,
                                     impl_->actual_trajectory_buffer, result)) {
      return false;
    }
    result->product.output_frame.has_range_compressed_echo = true;
  }

  if (HasDegenerateImagePeak(*result, input)) {
    session::RecordAbort(
        result, session::SarPipelineAbortReason::kPipelineExecutionFailed,
        session::codes::kDegenerateImagePeak,
        "SAR focused image has zero peak power; the echo/focusing pipeline produced no "
        "signal. Check sar.raw_echo_clipping and sar.slant_range_mismatch diagnostics.");
    return false;
  }

  // 中译：周期执行摘要（周期号、完成处理阶段、L1/L3 成像标志、估计信噪比、场景目标数）。
  // 标识：规则 13a 周期级执行摘要日志——每周期 SAR 处理概况与成像产出，
  //       供宏观核对"零产品"排查；仅人读，不用于状态判断（规则 3）。
  PROJECT_LOG_INFO("[SarPipeline] cycle_index={} completed_stage={} has_l1={} has_l3={} "
                   "estimated_snr_db={:.2f} targets={}",
                   input.cycle_index,
                   static_cast<int>(result->product.output_frame.completed_stage),
                   result->product.output_frame.has_l1_image,
                   result->product.output_frame.has_l3_bp_image,
                   result->product.output_frame.estimated_snr_db, input.point_targets.size());

  result->status = session::SarCycleStatus::kCompleted;
  return true;
}

SarProcessingPipelineRuntimeState SarProcessingPipeline::CaptureRuntimeState() const {
  SarProcessingPipelineRuntimeState state;
  state.owner_identity = this;
  state.schema_version = kPipelineRuntimeStateSchemaVersion;
  state.raw_pulse_buffer_state = impl_->raw_pulse_buffer.CaptureRuntimeState();
  state.ideal_trajectory_buffer = impl_->ideal_trajectory_buffer;
  state.actual_trajectory_buffer = impl_->actual_trajectory_buffer;
  state.next_pulse_id = impl_->next_pulse_id;
  state.pulse_fraction_carry = impl_->pulse_fraction_carry;
  return state;
}

bool SarProcessingPipeline::RestoreRuntimeState(const SarProcessingPipelineRuntimeState& state) {
  if (state.owner_identity != this || state.schema_version != kPipelineRuntimeStateSchemaVersion ||
      !impl_->raw_pulse_buffer.RestoreRuntimeState(state.raw_pulse_buffer_state)) {
    return false;
  }
  impl_->ideal_trajectory_buffer = state.ideal_trajectory_buffer;
  impl_->actual_trajectory_buffer = state.actual_trajectory_buffer;
  impl_->next_pulse_id = state.next_pulse_id;
  impl_->pulse_fraction_carry = state.pulse_fraction_carry;
  return true;
}

}  // namespace pipeline
}  // namespace sar
