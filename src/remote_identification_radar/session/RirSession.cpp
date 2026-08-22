/**
 * @file RirSession.cpp
 * @brief 远程识别雷达会话门面实现。
 *
 * 周期语义对齐 AR `ArSession`（审计基线 96de367c）：非执行周期不复用上一帧、
 * 校验拒绝返回 `kRejectedInvalidInput` + 明细、关机 `kPoweredOff` 只推进时间、
 * 运行期补丁在下一次成功周期边界提交。
 */

#include "1q/remote_identification_radar/session/RirSession.h"

#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

#include "1q/remote_identification_radar/config/RirRuntimeConfigPatch.h"
#include "1q/remote_identification_radar/session/RirInputValidation.h"
#include "common/numerics/Constants.h"
#include "common/radar/ScanScheduleRuntime.h"
#include "remote_identification_radar/dwell/RirBeamControl.h"
#include "remote_identification_radar/runtime/RirAcceptanceLog.h"
#include "remote_identification_radar/runtime/RirAcceptanceRecords.h"
#include "remote_identification_radar/runtime/RirController.h"
#include "remote_identification_radar/session/RirReplayCycleRecord.h"

namespace remote_identification_radar {
namespace session {

namespace {

/**
 * @brief 指定识别任务生命周期阶段（镜像 AR designation 生命周期骨架；
 *        会话级跨周期状态，随 patch 原子提交）。
 */
enum class RirDesignationPhase : std::uint8_t {
  kNone = 0,     /**< 未指定/未初始化（新指令待首个处理周期开窗）。 */
  kPending = 1,  /**< 限时窗口内：驻留对准指定目标等待识别达成。 */
  kAcquired = 2, /**< 识别已达成：任务完成，回到扫描（终态，直到外部重新指定）。 */
  kExpired = 3   /**< 窗口耗尽仍未识别：任务作废，回到扫描（终态，直到外部重新指定）。 */
};

struct RirDesignationPhaseAdvance {
  RirDesignationPhase phase{RirDesignationPhase::kNone};
  std::uint32_t deadline_cycle_index{0U}; /**< 窗口截止周期（0 = 无限期）。 */
  bool expired_edge{false}; /**< 本周期由 kPending 转移至 kExpired（作废沿）。 */
};

struct RirDesignationExpiry {
  bool expired{false};      /**< 任务已作废（kExpired 终态）。 */
  bool expiry_cycle{false}; /**< 本周期是作废沿（报告 kAcquisitionTimeout）。 */
};

/**
 * @brief 推进指定识别任务生命周期（镜像 AR AdvanceDesignationPhase）。
 *
 * 状态机：kNone → kPending → kAcquired | kExpired（kAcquired/kExpired 为终态）：
 *   - 指令未消费（未指定 / work_mode 非 kIdentify）：阶段恒为 kNone，窗口不运行；
 *   - kNone：开窗——deadline = cycle + duration（duration == 0 表示无限期）；
 *   - kPending：上一周期识别达成（kCategoryConfirmed/kModelConfirmed）→ kAcquired
 *     （任务完成回扫描）；否则 cycle >= deadline 时 → kExpired（任务作废回扫描）；
 *   - kAcquired / kExpired：不再转移。
 * @note 识别判定与驻留同源，使用"上一周期航迹快照"口径（滞后一周期）。
 */
RirDesignationPhaseAdvance AdvanceDesignationPhase(
    RirDesignationPhase phase, std::uint32_t deadline_cycle_index,
    std::uint32_t designation_duration_cycles, std::uint32_t cycle_index,
    bool designation_consumed, bool recognized) {
  RirDesignationPhaseAdvance advance;
  advance.phase = phase;
  advance.deadline_cycle_index = deadline_cycle_index;
  if (!designation_consumed) {
    advance.phase = RirDesignationPhase::kNone;
    advance.deadline_cycle_index = 0U;
    return advance;
  }
  if (phase == RirDesignationPhase::kNone) {
    advance.phase = RirDesignationPhase::kPending;
    if (designation_duration_cycles > 0U) {
      // 饱和加法：周期号接近 2³² 时不回绕为 0（0 = 无限期语义）。
      const std::uint64_t deadline =
          static_cast<std::uint64_t>(cycle_index) + designation_duration_cycles;
      advance.deadline_cycle_index = static_cast<std::uint32_t>(
          std::min<std::uint64_t>(deadline, std::numeric_limits<std::uint32_t>::max()));
    } else {
      advance.deadline_cycle_index = 0U;
    }
    return advance;
  }
  if (phase == RirDesignationPhase::kPending) {
    if (recognized) {
      advance.phase = RirDesignationPhase::kAcquired;
    } else if (advance.deadline_cycle_index != 0U &&
               cycle_index >= advance.deadline_cycle_index) {
      advance.phase = RirDesignationPhase::kExpired;
      advance.expired_edge = true;  // 作废沿 = 转移发生的周期。
    }
  }
  return advance;
}

RirDesignationExpiry ResolveDesignationExpiry(RirDesignationPhase phase, bool expired_edge) {
  RirDesignationExpiry expiry;
  expiry.expired = phase == RirDesignationPhase::kExpired;
  expiry.expiry_cycle = expired_edge;
  return expiry;
}

const session::RirSceneTarget* FindSceneTarget(const session::RirSceneTargetList& targets,
                                               std::uint64_t external_target_id) {
  if (external_target_id == 0U) {
    return nullptr;
  }
  for (const session::RirSceneTarget& target : targets) {
    if (target.external_target_id == external_target_id) {
      return &target;
    }
  }
  return nullptr;
}

/** @brief 目标视线角（与 RirController::ComputeLookAngles 同口径；位置退化用 range_m）。 */
config::RirAzimuthElevationDeg TargetLookAngles(const session::RirSceneTarget& target) {
  float px = target.position_x;
  float py = target.position_y;
  float pz = target.position_z;
  if (std::sqrt(px * px + py * py + pz * pz) <= 0.0f) {
    px = target.range_m;
    py = 0.0f;
    pz = 0.0f;
  }
  const float range_hypot = std::sqrt(px * px + py * py);
  config::RirAzimuthElevationDeg look;
  look.az_deg = oneq::common::numerics::RadToDeg(std::atan2(py, px));
  look.el_deg = oneq::common::numerics::RadToDeg(std::atan2(pz, range_hypot));
  return look;
}

/** @brief 目标视线角是否在 scan_center + 可扫描体积内（az 相对、el 绝对）。 */
bool TargetWithinSteerableVolume(const config::RirAzimuthElevationDeg& look,
                                 const config::RirAzimuthElevationLimitsDeg& volume,
                                 const config::RirAzimuthElevationDeg& scan_center) {
  const float delta_az_deg = oneq::common::radar::NormalizeAzimuthDeltaDeg(
      look.az_deg - scan_center.az_deg);
  return delta_az_deg >= volume.az_min_deg && delta_az_deg <= volume.az_max_deg &&
         look.el_deg >= volume.el_min_deg && look.el_deg <= volume.el_max_deg;
}

/** @brief 由相对可扫描体积 + scan_center 构建绝对 ENU 波位序列。 */
std::vector<oneq::common::radar::AzimuthElevationDeg> BuildAbsoluteScanWaves(
    const config::RirSessionConfig& config) {
  const dwell::RirEffectiveBeamwidthDeg beamwidth =
      dwell::RirResolveEffectiveBeamwidth(config.hardware.antenna);
  const config::RirScanConfig& scan = config.mission.scan;
  const config::RirAzimuthElevationLimitsDeg& volume = config.orientation.steerable_volume_deg;
  const float az_step = beamwidth.az_beamwidth_deg * scan.step_scale;
  const float el_step = beamwidth.el_beamwidth_deg * scan.step_scale;
  const std::vector<oneq::common::radar::AzimuthElevationDeg> relative_pattern =
      oneq::common::radar::BuildScanPattern(
          volume.az_min_deg, volume.az_max_deg, volume.el_min_deg, volume.el_max_deg, az_step,
          el_step, scan.scan_start_position, scan.scan_sequence);
  std::vector<oneq::common::radar::AzimuthElevationDeg> absolute_pattern;
  absolute_pattern.reserve(relative_pattern.size());
  const config::RirAzimuthElevationDeg& center = config.mission.scan_center_deg;
  for (const oneq::common::radar::AzimuthElevationDeg& wave : relative_pattern) {
    oneq::common::radar::AzimuthElevationDeg absolute;
    absolute.az_deg = oneq::common::radar::NormalizeAzimuthDeg(center.az_deg + wave.az_deg);
    absolute.el_deg = wave.el_deg;
    absolute_pattern.push_back(absolute);
  }
  return absolute_pattern;
}

/**
 * @brief 扫描策略波位（相对体积 + center 平移 + 方位归一化；非法体积/步长回退 scan_center）。
 * @note 第 N 周期取第 (N-1) % size 个波位。
 */
config::RirAzimuthElevationDeg ResolveScanWavePosition(const config::RirSessionConfig& config,
                                                       std::uint32_t cycle_index) {
  const std::vector<oneq::common::radar::AzimuthElevationDeg> pattern =
      BuildAbsoluteScanWaves(config);
  if (pattern.empty()) {
    return config.mission.scan_center_deg;
  }
  const std::uint64_t zero_based_cycle =
      cycle_index > 0U ? static_cast<std::uint64_t>(cycle_index - 1U) : 0U;
  const oneq::common::radar::AzimuthElevationDeg& wave =
      pattern[static_cast<std::size_t>(zero_based_cycle %
                                       static_cast<std::uint64_t>(pattern.size()))];
  return config::RirAzimuthElevationDeg{wave.az_deg, wave.el_deg};
}

}  // namespace

struct RirSession::Impl {
  config::RirSessionConfig config{};
  runtime::RirController controller{};
  config::RirRuntimeConfigPatch pending_patch{};
  bool has_pending_patch{false};
  // 指定识别任务状态（会话级，镜像 AR designation；随 patch 原子提交；
  // 任一指定相关字段变更视为新指令，窗口重新起算）。
  std::uint64_t designated_external_target_id{0U};
  std::uint32_t designation_duration_cycles{0U};
  RirDesignationPhase designation_phase{RirDesignationPhase::kNone};
  std::uint32_t designation_deadline_cycle_index{0U};
  std::uint64_t next_batch_id{1U};
  // [RirAccept] 波位排列表已按当前扫描配置输出（mission 配置变更后重置重发）。
  bool acceptance_scan_pattern_logged{false};

  explicit Impl(const config::RirSessionConfig& session_config) : config(session_config) {
    controller.SetHardware(config.hardware);
    controller.SetSensorPlatformId(config.sensor_platform_id);
    controller.UpdateRuntime(config.mission, config.policy);
    controller.UpdateEnvironment(config.environment);
  }
};

RirSession::RirSession() : impl_(new Impl(config::RirSessionConfig{})) {}
RirSession::RirSession(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
RirSession::~RirSession() = default;
RirSession::RirSession(RirSession&&) noexcept = default;
RirSession& RirSession::operator=(RirSession&&) noexcept = default;

RirOutputFrame RirSession::Step(const RirCycleInput& input) {
  return StepWithResult(input).output_frame;
}

RirCycleResult RirSession::StepWithResult(const RirCycleInput& input) {
  RirCycleResult result;
  result.input_cycle_index = input.input_cycle_index;
  result.output_frame.input_cycle_index = input.input_cycle_index;
  result.output_frame.batch_id = impl_->next_batch_id;

  // 关机：非执行周期，只记录状态，不推进识别状态（tracker 状态不被触碰）。
  if (!impl_->config.sensor_enabled) {
    result.status = RirCycleStatus::kPoweredOff;
    result.abort_reason = RirCycleAbortReason::kPoweredOff;
    return result;
  }

  // 校验拒绝：不执行流水线，问题明细入 issues。
  const RirIssueList validation_issues =
      ValidateRirCycleInput(input, impl_->config.mission.recognition_dwell_sec);
  if (HasValidationError(validation_issues)) {
    result.status = RirCycleStatus::kRejectedInvalidInput;
    result.abort_reason = RirCycleAbortReason::kValidationRejected;
    result.issues = validation_issues;
    return result;
  }

  // 补丁提交（下一个成功周期边界）：任务域/电源/识别策略整域；叶子 work_mode 覆盖整域。
  if (impl_->has_pending_patch) {
    const config::RirRuntimeConfigPatch& patch = impl_->pending_patch;
    if (patch.has_mission) {
      impl_->config.mission = patch.mission;
      impl_->acceptance_scan_pattern_logged = false;
    }
    if (patch.has_work_mode) {
      impl_->config.mission.work_mode = patch.work_mode;
    }
    if (patch.has_scan_center) {
      impl_->config.mission.scan_center_deg = patch.scan_center_deg;
      impl_->acceptance_scan_pattern_logged = false;
    }
    if (patch.has_policy) {
      impl_->config.policy = patch.policy;
    }
    if (patch.has_environment) {
      impl_->config.environment = patch.environment;
      impl_->controller.UpdateEnvironment(impl_->config.environment);
    }
    if (patch.has_sensor_enabled) {
      impl_->config.sensor_enabled = patch.sensor_enabled;
    }
    // 指定识别任务：任一指定相关字段变更（含仅改时长）都视为新指令，
    // 生命周期阶段重置，窗口在指令生效后首个周期重新起算。
    if (patch.has_designated_target_id || patch.has_designation_duration_cycles) {
      if (patch.has_designated_target_id) {
        impl_->designated_external_target_id = patch.designated_external_target_id;
      }
      if (patch.has_designation_duration_cycles) {
        impl_->designation_duration_cycles = patch.designation_duration_cycles;
      }
      impl_->designation_phase = RirDesignationPhase::kNone;
      impl_->designation_deadline_cycle_index = 0U;
    }
    impl_->controller.UpdateRuntime(impl_->config.mission, impl_->config.policy);
    impl_->has_pending_patch = false;
  }

  // 指定识别任务生命周期推进（镜像 AR 限时锁定语义）：
  //   识别达成（上一周期口径）→ 任务完成回到扫描；窗口耗尽仍未识别 →
  //   任务作废（作废沿报告 kAcquisitionTimeout）→ 回到扫描。
  const bool designation_consumed =
      impl_->config.mission.work_mode == config::RirWorkMode::kIdentify &&
      impl_->designated_external_target_id != 0U;
  const RirDesignationPhaseAdvance advance = AdvanceDesignationPhase(
      impl_->designation_phase, impl_->designation_deadline_cycle_index,
      impl_->designation_duration_cycles, input.input_cycle_index, designation_consumed,
      designation_consumed &&
          impl_->controller.IsTargetRecognized(impl_->designated_external_target_id));
  impl_->designation_phase = advance.phase;
  impl_->designation_deadline_cycle_index = advance.deadline_cycle_index;
  const RirDesignationExpiry expiry =
      ResolveDesignationExpiry(advance.phase, advance.expired_edge);

  // 驻留中心（库内驻留调度器）：任务窗口内对准指定目标（在场景且在可扫描体积内）；
  // 其余情况按扫描策略逐周期推进。
  const session::RirSceneTarget* designated_target = FindSceneTarget(
      input.scene_targets, impl_->designated_external_target_id);
  const bool target_in_scene = designated_target != nullptr;
  const bool target_in_volume =
      target_in_scene &&
      TargetWithinSteerableVolume(TargetLookAngles(*designated_target),
                                  impl_->config.orientation.steerable_volume_deg,
                                  impl_->config.mission.scan_center_deg);
  const bool dwelling_on_target =
      advance.phase == RirDesignationPhase::kPending && target_in_volume;
  const config::RirAzimuthElevationDeg dwell_center =
      dwelling_on_target ? TargetLookAngles(*designated_target)
                         : ResolveScanWavePosition(impl_->config, input.input_cycle_index);

  // 验收事件 beam_pattern（3.2.2.4.2.1）：完整波位排列表按当前扫描配置一次性
  // 输出（与逐周期取位同源；mission/orientation 配置变更后重发）。
  if (RIR_ACCEPTANCE_LOG_ENABLED() && !impl_->acceptance_scan_pattern_logged) {
    const dwell::RirEffectiveBeamwidthDeg beamwidth =
        dwell::RirResolveEffectiveBeamwidth(impl_->config.hardware.antenna);
    const std::vector<oneq::common::radar::AzimuthElevationDeg> pattern =
        BuildAbsoluteScanWaves(impl_->config);
    const std::string csv_path = "rir_antenna_pattern.csv";
    runtime::TryExportRirAntennaPatternCsv(impl_->config.hardware.antenna, csv_path.c_str());
    runtime::WriteRirAntennaPatternSummary(input.sim_time_sec, input.input_cycle_index,
                                           impl_->config.hardware.antenna.main_beam_gain_db,
                                           beamwidth.az_beamwidth_deg, beamwidth.el_beamwidth_deg,
                                           csv_path);
    runtime::WriteRirOncePerSession(input.sim_time_sec, input.input_cycle_index);
    impl_->acceptance_scan_pattern_logged = true;
    (void)pattern;
  }
  if (RIR_ACCEPTANCE_LOG_ENABLED()) {
    const std::vector<oneq::common::radar::AzimuthElevationDeg> pattern =
        BuildAbsoluteScanWaves(impl_->config);
    runtime::WriteRirBeamScan(input.sim_time_sec, input.input_cycle_index, pattern,
                              dwell_center.az_deg, dwell_center.el_deg, dwelling_on_target);
    runtime::WriteRirCycleRunCount(input.sim_time_sec, input.input_cycle_index);
  }

  impl_->controller.RunCycle(input, &result.output_frame, impl_->next_batch_id, dwell_center);
  result.status = RirCycleStatus::kCompleted;
  result.abort_reason = RirCycleAbortReason::kNone;
  ++impl_->next_batch_id;
  if (impl_->controller.HasLatestSummary()) {
    result.has_recognition_summary = true;
    result.recognition_summary = impl_->controller.GetLatestSummary();
  }
  // 航迹归属视图回填（结果层）：仅已执行周期携带，非执行路径早退保持空列表。
  result.track_attributions = impl_->controller.LatestTrackAttributions();
  result.emission_frame = impl_->controller.LatestEmissionFrame();

  // 指定识别任务结果回填（镜像 AR designation_* 形状）：
  //   kPending + 目标在体积内 → active；kPending + 目标缺席 → kNotRecognized；
  //   kPending + 目标在场景但越界 → kOutsideSteerableVolume（阶段保持 kPending）；
  //   作废沿 → kAcquisitionTimeout；任务完成/作废后 → 指定清零。
  const bool pending = advance.phase == RirDesignationPhase::kPending;
  result.designated_target_id =
      (pending || expiry.expiry_cycle) ? impl_->designated_external_target_id : 0U;
  result.designation_active = pending && target_in_volume;
  result.designation_reverted_to_scan =
      expiry.expired ? expiry.expiry_cycle : (pending && !target_in_volume);
  if (expiry.expiry_cycle) {
    result.designation_revert_reason = session::RirDesignationRevertReason::kAcquisitionTimeout;
  } else if (result.designation_reverted_to_scan) {
    result.designation_revert_reason =
        target_in_scene && !target_in_volume
            ? session::RirDesignationRevertReason::kOutsideSteerableVolume
            : session::RirDesignationRevertReason::kNotRecognized;
  } else {
    result.designation_revert_reason = session::RirDesignationRevertReason::kNone;
  }
  result.dwell_center_deg = dwell_center;
  return result;
}

bool RirSession::TryApplyRuntimeConfig(const config::RirRuntimeConfigPatch& patch) {
  impl_->pending_patch = patch;
  impl_->has_pending_patch = true;
  return true;
}

bool RirSession::HasLatestRecognitionSummary() const {
  return impl_->controller.HasLatestSummary();
}

const RirRecognitionCycleSummary& RirSession::GetLatestRecognitionSummary() const {
  return impl_->controller.GetLatestSummary();
}

double RirSession::LastRecognitionDatabaseLoadMs() const {
  return impl_ != nullptr ? impl_->controller.LastDatabaseLoadMs() : 0.0;
}

RirSession RirSession::Create(const config::RirSessionConfig& config) {
  return RirSession(std::unique_ptr<Impl>(new Impl(config)));
}

RirSession RirSession::CreateWithDiagnostics(const config::RirSessionConfig& config,
                                             RirIssueList* issues) {
  if (issues != nullptr) {
    *issues = config::ValidateRirSessionConfig(config);
  }
  return RirSession(std::unique_ptr<Impl>(new Impl(config)));
}

RirSessionReplayState RirSessionReplayAccess::CaptureSessionState(const RirSession& session) {
  RirSessionReplayState replay_state;
  replay_state.active_database_version = session.impl_->controller.ActiveDatabaseVersion();
  replay_state.detection_random_seed = session.impl_->controller.DetectionRandomSeed();
  return replay_state;
}

}  // namespace session
}  // namespace remote_identification_radar
